import 'dart:io';
import 'dart:typed_data';

import 'package:flutter/material.dart';
import 'package:share_plus/share_plus.dart';

import '../dataset/sensor_event.dart';
import '../dataset/session_reader.dart';
import '../dataset/session_store.dart';

/// One saved session: metadata, measured rates and counts, the first image,
/// and rename / export / delete.
class SessionDetailScreen extends StatefulWidget {
  const SessionDetailScreen({super.key, required this.store, required this.directory});

  final SessionStore store;
  final Directory directory;

  @override
  State<SessionDetailScreen> createState() => _SessionDetailScreenState();
}

class _SessionDetailScreenState extends State<SessionDetailScreen> {
  late Directory _dir = widget.directory;
  late Future<SessionData> _data = SessionReader.read(_dir);
  bool _busy = false;

  Future<void> _rename(SessionData data) async {
    final label = await showDialog<String>(
      context: context,
      builder: (_) => _RenameDialog(initial: data.meta.settings.label),
    );
    if (label == null || label.isEmpty) return;
    await _run(() async {
      final dir = await widget.store.rename(_dir, label);
      setState(() {
        _dir = dir;
        _data = SessionReader.read(dir);
      });
    });
  }

  Future<void> _export(Rect origin) => _run(() async {
    final zip = await widget.store.exportZip(_dir);
    await SharePlus.instance.share(ShareParams(files: [XFile(zip.path)], sharePositionOrigin: origin));
  });

  Future<void> _delete() async {
    final ok = await showDialog<bool>(
      context: context,
      builder: (context) => AlertDialog(
        title: const Text('Delete session?'),
        content: const Text('The folder and all its images are removed from the phone. This cannot be undone.'),
        actions: [
          TextButton(onPressed: () => Navigator.pop(context, false), child: const Text('Cancel')),
          FilledButton(onPressed: () => Navigator.pop(context, true), child: const Text('Delete')),
        ],
      ),
    );
    if (ok != true) return;
    await _run(() async {
      await widget.store.delete(_dir);
      if (mounted) Navigator.pop(context);
    });
  }

  Future<void> _run(Future<void> Function() action) async {
    setState(() => _busy = true);
    try {
      await action();
    } catch (e) {
      if (mounted) ScaffoldMessenger.of(context).showSnackBar(SnackBar(content: Text('$e')));
    } finally {
      if (mounted) setState(() => _busy = false);
    }
  }

  @override
  Widget build(BuildContext context) {
    return Scaffold(
      appBar: AppBar(title: const Text('Session')),
      body: FutureBuilder<SessionData>(
        future: _data,
        builder: (context, snapshot) {
          if (snapshot.hasError) {
            return ListView(
              padding: const EdgeInsets.all(16),
              children: [
                Text('Could not read this session: ${snapshot.error}'),
                const SizedBox(height: 16),
                OutlinedButton.icon(
                  onPressed: _busy ? null : _delete,
                  icon: const Icon(Icons.delete_outline),
                  label: const Text('Delete'),
                ),
              ],
            );
          }
          if (!snapshot.hasData) return const Center(child: CircularProgressIndicator());
          final data = snapshot.data!;
          return ListView(
            padding: const EdgeInsets.fromLTRB(12, 8, 12, 24),
            children: [
              if (data.frames.isNotEmpty) _FirstFrame(frame: data.frames.first),
              const SizedBox(height: 8),
              _InfoCard(rows: _rows(data)),
              const SizedBox(height: 12),
              if (_busy) const LinearProgressIndicator(),
              const SizedBox(height: 8),
              Wrap(
                spacing: 8,
                runSpacing: 8,
                children: [
                  OutlinedButton.icon(
                    onPressed: _busy ? null : () => _rename(data),
                    icon: const Icon(Icons.edit_outlined),
                    label: const Text('Rename'),
                  ),
                  Builder(
                    builder: (context) => OutlinedButton.icon(
                      onPressed: _busy
                          ? null
                          : () {
                              final box = context.findRenderObject() as RenderBox?;
                              final origin = box == null ? Rect.zero : box.localToGlobal(Offset.zero) & box.size;
                              _export(origin);
                            },
                      icon: const Icon(Icons.ios_share),
                      label: const Text('Export'),
                    ),
                  ),
                  OutlinedButton.icon(
                    onPressed: _busy ? null : _delete,
                    icon: const Icon(Icons.delete_outline),
                    label: const Text('Delete'),
                  ),
                ],
              ),
            ],
          );
        },
      ),
    );
  }

  static List<(String, String)> _rows(SessionData data) {
    final m = data.meta;
    final c = m.counts;
    String hz(double v) => '${v.toStringAsFixed(1)} Hz';
    String secs(int us) => '${(us / 1e6).toStringAsFixed(2)} s';
    final k = data.frames.isEmpty ? null : data.frames.first.event.intrinsics;
    final imuTimes = data.imu.map((e) => e.tUs);
    final frameTimes = data.frames.map((f) => f.event.tUs);
    return [
      ('Folder', data.directory.uri.pathSegments.where((s) => s.isNotEmpty).last),
      ('Label', m.settings.label),
      ('Experiment', m.settings.experiment.label),
      ('Created', m.created.toString()),
      ('Duration', secs(m.duration.inMicroseconds)),
      ('Device', '${m.device.model}, ${m.device.os}'),
      ('App', '${m.device.appVersion} (${m.device.appBuild})'),
      ('Markers', '${m.settings.markerDictionary}, side ${(m.settings.markerSideM * 100).toStringAsFixed(1)} cm'),
      if (m.settings.notes.isNotEmpty) ('Notes', m.settings.notes),
      (
        'Image',
        '${m.imageWidth ?? '—'}×${m.imageHeight ?? '—'} from ${m.sourceWidth ?? '—'}×${m.sourceHeight ?? '—'}, '
            'JPEG q ${m.jpegQuality?.toStringAsFixed(2) ?? '—'}',
      ),
      if (k != null)
        (
          'Intrinsics',
          'fx ${k.fx.toStringAsFixed(1)} fy ${k.fy.toStringAsFixed(1)} '
              'cx ${k.cx.toStringAsFixed(1)} cy ${k.cy.toStringAsFixed(1)} (ARKit, first frame)',
        ),
      (
        'Frames',
        '${data.frames.length} at ${hz(data.frameHz)} (requested ${m.requestedFrameHz.toStringAsFixed(0)} Hz), '
            '${c.frameGaps} gaps, ${c.framesWithoutImage} without image',
      ),
      (
        'ARKit poses',
        '${data.poses.length} (${data.poses.where((p) => p.tracking).length} tracked) at ${hz(data.poseHz)}',
      ),
      ('Accelerometer', '${data.count(ImuSensor.accelerometer)} at ${hz(data.imuHz(ImuSensor.accelerometer))}'),
      ('Gyroscope', '${data.count(ImuSensor.gyroscope)} at ${hz(data.imuHz(ImuSensor.gyroscope))}'),
      if (imuTimes.isNotEmpty && frameTimes.isNotEmpty)
        (
          'Time span',
          'frames ${secs(frameTimes.last - frameTimes.first)}, IMU ${secs(imuTimes.last - imuTimes.first)}, '
              'IMU starts ${secs((frameTimes.first - imuTimes.first).abs())} '
              '${frameTimes.first >= imuTimes.first ? 'before' : 'after'} the first frame',
        ),
    ];
  }
}

class _FirstFrame extends StatelessWidget {
  const _FirstFrame({required this.frame});

  final FrameRecord frame;

  @override
  Widget build(BuildContext context) {
    return FutureBuilder<Uint8List?>(
      future: frame.event.loadJpeg(),
      builder: (context, snapshot) {
        final bytes = snapshot.data;
        if (bytes == null) return const SizedBox.shrink();
        return Column(
          crossAxisAlignment: CrossAxisAlignment.start,
          children: [
            ClipRRect(borderRadius: BorderRadius.circular(8), child: Image.memory(bytes, gaplessPlayback: true)),
            const SizedBox(height: 4),
            Text('First frame (${frame.file}), sensor orientation', style: Theme.of(context).textTheme.bodySmall),
          ],
        );
      },
    );
  }
}

class _InfoCard extends StatelessWidget {
  const _InfoCard({required this.rows});

  final List<(String, String)> rows;

  @override
  Widget build(BuildContext context) {
    final theme = Theme.of(context);
    return Card(
      margin: EdgeInsets.zero,
      child: Padding(
        padding: const EdgeInsets.all(10),
        child: Column(
          children: [
            for (final (label, value) in rows)
              Padding(
                padding: const EdgeInsets.symmetric(vertical: 2),
                child: Row(
                  crossAxisAlignment: CrossAxisAlignment.start,
                  children: [
                    SizedBox(
                      width: 104,
                      child: Text(
                        label,
                        style: theme.textTheme.labelMedium?.copyWith(color: theme.colorScheme.onSurfaceVariant),
                      ),
                    ),
                    Expanded(child: SelectableText(value, style: theme.textTheme.bodySmall)),
                  ],
                ),
              ),
          ],
        ),
      ),
    );
  }
}

/// Owns its text controller so the controller lives exactly as long as the
/// dialog (including its closing animation).
class _RenameDialog extends StatefulWidget {
  const _RenameDialog({required this.initial});

  final String initial;

  @override
  State<_RenameDialog> createState() => _RenameDialogState();
}

class _RenameDialogState extends State<_RenameDialog> {
  late final _controller = TextEditingController(text: widget.initial);

  @override
  void dispose() {
    _controller.dispose();
    super.dispose();
  }

  @override
  Widget build(BuildContext context) {
    return AlertDialog(
      title: const Text('Rename session'),
      content: TextField(
        controller: _controller,
        autofocus: true,
        decoration: const InputDecoration(labelText: 'Label'),
      ),
      actions: [
        TextButton(onPressed: () => Navigator.pop(context), child: const Text('Cancel')),
        FilledButton(onPressed: () => Navigator.pop(context, _controller.text.trim()), child: const Text('Rename')),
      ],
    );
  }
}
