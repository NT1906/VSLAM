import 'dart:io';
import 'dart:typed_data';

import 'package:flutter/material.dart';
import 'package:share_plus/share_plus.dart';
import 'package:vector_math/vector_math_64.dart' show Matrix3;

import '../calibration/camera_calibration.dart';
import '../calibration/imu_alignment.dart';
import '../calibration/session_alignment.dart';
import '../dataset/sensor_event.dart';
import '../dataset/session_reader.dart';
import '../dataset/session_store.dart';
import '../estimation/keyframe_selector.dart';
import '../estimation/session_baselines.dart';
import '../mapping/marker_map.dart';
import '../vision/aruco_detector.dart';
import '../vision/session_detection.dart';

/// One saved session: metadata, measured rates and counts, the first image,
/// and rename / export / delete.
class SessionDetailScreen extends StatefulWidget {
  const SessionDetailScreen({
    super.key,
    required this.store,
    required this.directory,
    this.detector = const ArucoMarkerDetector(),
  });

  final SessionStore store;
  final Directory directory;

  /// Used by Detect markers and by Build map when detection has not run.
  final MarkerDetector detector;

  @override
  State<SessionDetailScreen> createState() => _SessionDetailScreenState();
}

class _SessionDetailScreenState extends State<SessionDetailScreen> {
  late Directory _dir = widget.directory;
  late Future<SessionData> _data = SessionReader.read(_dir);
  bool _busy = false;
  ImuAlignment? _alignment;
  DetectionSummary? _detections;
  BaselineRun? _baselines;
  String? _savedMap;
  double? _progress;

  Future<void> _rename(SessionData data) async {
    final label = await showDialog<String>(
      context: context,
      builder: (_) => _TextDialog(title: 'Rename session', label: 'Label', initial: data.meta.settings.label, action: 'Rename'),
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

  Future<void> _align(SessionData data) => _run(() async {
    final result = alignSession(data);
    await saveAlignment(_dir, result);
    if (mounted) setState(() => _alignment = result);
  });

  /// Runs marker detection on every saved frame and writes
  /// `results/detections.csv` and `results/detections.json`.
  Future<void> _detect(SessionData data) => _run(() async {
    final calibration = await CalibrationResult.load(widget.store.root);
    final summary = await detectSession(
      data,
      calibration: calibration,
      detector: widget.detector,
      onProgress: (done, total) {
        if (mounted) setState(() => _progress = done / total);
      },
    );
    if (mounted) setState(() => _detections = summary);
  });

  /// WP4: keyframes, baseline 1 (ARKit), baseline 2 (IMU) and the first
  /// marker map; runs marker detection first when it has not been run.
  Future<void> _buildMap(SessionData data) => _run(() async {
    final calibration = await CalibrationResult.load(widget.store.root);
    final run = await processSessionBaselines(
      data,
      calibration: calibration,
      detector: widget.detector,
      onProgress: (f) {
        if (mounted) setState(() => _progress = f);
      },
    );
    if (mounted) {
      setState(() {
        _baselines = run;
        _savedMap = null;
      });
    }
  });

  /// Saves the map just built as `maps/<room>.json`.
  Future<void> _saveMap(BaselineRun run) async {
    final room = await showDialog<String>(
      context: context,
      builder: (_) => const _TextDialog(title: 'Save map', label: 'Room name', initial: 'room', action: 'Save'),
    );
    if (room == null || room.isEmpty) return;
    await _run(() async {
      final file = await MapStore(widget.store.root).save(room, run.map);
      if (mounted) setState(() => _savedMap = 'maps/${file.uri.pathSegments.last}');
    });
  }

  Future<void> _run(Future<void> Function() action) async {
    setState(() => _busy = true);
    try {
      await action();
    } catch (e) {
      if (mounted) ScaffoldMessenger.of(context).showSnackBar(SnackBar(content: Text('$e')));
    } finally {
      if (mounted) {
        setState(() {
          _busy = false;
          _progress = null;
        });
      }
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
              if (_alignment case final a?) ...[
                const SizedBox(height: 8),
                _InfoCard(rows: _alignmentRows(a)),
              ],
              if (_detections case final d?) ...[
                const SizedBox(height: 8),
                _InfoCard(rows: _detectionRows(d)),
              ],
              if (_baselines case final b?) ...[
                const SizedBox(height: 8),
                _InfoCard(rows: _baselineRows(b, _savedMap)),
              ],
              const SizedBox(height: 12),
              if (_busy) LinearProgressIndicator(value: _progress),
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
                  OutlinedButton.icon(
                    onPressed: _busy ? null : () => _align(data),
                    icon: const Icon(Icons.sync_alt),
                    label: const Text('IMU alignment'),
                  ),
                  OutlinedButton.icon(
                    onPressed: _busy ? null : () => _detect(data),
                    icon: const Icon(Icons.qr_code_2),
                    label: const Text('Detect markers'),
                  ),
                  OutlinedButton.icon(
                    onPressed: _busy ? null : () => _buildMap(data),
                    icon: const Icon(Icons.map_outlined),
                    label: const Text('Build map'),
                  ),
                  if (_baselines case final b?)
                    OutlinedButton.icon(
                      onPressed: _busy ? null : () => _saveMap(b),
                      icon: const Icon(Icons.save_outlined),
                      label: const Text('Save map'),
                    ),
                ],
              ),
            ],
          );
        },
      ),
    );
  }

  static List<(String, String)> _detectionRows(DetectionSummary d) {
    final k = d.intrinsics;
    return [
      ('Frames', '${d.framesWithImage} of ${d.frames} with an image; ${d.framesWithAccepted} with an accepted marker'),
      (
        'Accepted',
        d.accepted.isEmpty ? 'none' : [for (final e in d.accepted.entries) 'ID ${e.key}: ${e.value}'].join(', '),
      ),
      (
        'Rejected',
        d.rejected.isEmpty ? 'none' : [for (final e in d.rejected.entries) '${e.key}: ${e.value}'].join(', '),
      ),
      ('Marker side', '${(d.sideM * 100).toStringAsFixed(1)} cm (session settings)'),
      (
        'Intrinsics',
        '${d.intrinsicsSource == IntrinsicsSource.calibration ? 'calibration' : 'ARKit per frame'}'
            '${k == null ? '' : ': $k'}',
      ),
      ('Time per frame', '${d.meanDetectMs.toStringAsFixed(1)} ms mean, ${d.maxDetectMs.toStringAsFixed(1)} ms max'),
      ('Saved', 'results/$detectionsFile, results/$detectionsSummaryFile'),
    ];
  }

  static List<(String, String)> _baselineRows(BaselineRun b, String? savedMap) {
    String cm(double m) => '${(m * 100).toStringAsFixed(1)} cm';
    final reasons = b.reasonCounts;
    final imu = b.imu;
    final end = b.arkit.last.translation;
    return [
      (
        'Keyframes',
        '${b.keyframes.length} from ${b.trackedFrames} tracked of ${b.frames} frames '
            '(move ${reasons[KeyframeReason.translation]}, turn ${reasons[KeyframeReason.rotation]}, '
            'marker ${reasons[KeyframeReason.marker]}); ${b.trackingGaps} tracking gaps',
      ),
      (
        'ARKit (b1)',
        'path ${b.pathLengthM.toStringAsFixed(2)} m, end at '
            '(${end.x.toStringAsFixed(2)}, ${end.y.toStringAsFixed(2)}, ${end.z.toStringAsFixed(2)}) m in W',
      ),
      if (imu == null)
        ('IMU (b2)', b.imuMissing ?? 'not run')
      else ...[
        (
          'IMU rest',
          '${imu.restFound ? 'still' : 'NOT still (biases set to 0)'}: '
              '${((imu.rest.t0Us - b.keyframes.first.tUs) / 1e6).toStringAsFixed(1)}–'
              '${((imu.rest.t1Us - b.keyframes.first.tUs) / 1e6).toStringAsFixed(1)} s after the first keyframe, '
              'gyro σ ${imu.rest.gyroStd.toStringAsFixed(4)} rad/s, acc σ ${imu.rest.accStd.toStringAsFixed(3)} m/s²',
        ),
        (
          'IMU biases',
          'b_g (${[for (final v in imu.rest.bg.storage) v.toStringAsFixed(4)].join(', ')}) rad/s, '
              'b_a (${[for (final v in imu.rest.ba.storage) v.toStringAsFixed(3)].join(', ')}) m/s²',
        ),
        (
          'IMU vs ARKit',
          b.imuDriftRows.isEmpty
              ? 'no keyframe after the IMU start'
              : [for (final (s, d) in b.imuDriftRows) '${s.toStringAsFixed(1)} s: ${cm(d)}'].join(', '),
        ),
      ],
      (
        'Markers',
        b.map.markers.isEmpty
            ? 'none placed'
            : [
                for (final m in b.map.markers)
                  'ID ${m.id}: (${m.wTm.translation.x.toStringAsFixed(2)}, ${m.wTm.translation.y.toStringAsFixed(2)}, '
                      '${m.wTm.translation.z.toStringAsFixed(2)}) m, ${m.keyframeObservations} kf obs',
              ].join('\n'),
      ),
      if (b.unplacedIds.isNotEmpty) ('Not placed', 'IDs ${b.unplacedIds.join(', ')} (no accepted detection in a keyframe)'),
      ('Saved', 'results/$trajArkitFile, $trajImuFile, $mapFile, $baselinesSummaryFile${savedMap == null ? '' : '; $savedMap'}'),
    ];
  }

  static List<(String, String)> _alignmentRows(ImuAlignment a) {
    String row(int r) => [for (var c = 0; c < 3; c++) a.rCI.entry(r, c).toStringAsFixed(3).padLeft(6)].join(' ');
    String mat(Matrix3 m) => [for (var r = 0; r < 3; r++) [for (var c = 0; c < 3; c++) m.entry(r, c).round()].join(' ')].join(' / ');
    return [
      ('Time offset', '${(a.offsetUs / 1000).toStringAsFixed(1)} ms (gyro stamps later than camera when positive)'),
      ('R_ci', '${row(0)}\n${row(1)}\n${row(2)}'),
      ('Apple axes', '${mat(appleCameraFromDevice)}, differs by ${a.angleToDeg(appleCameraFromDevice).toStringAsFixed(2)}°'),
      (
        'Residual',
        '${a.rmsResidual.toStringAsFixed(3)} rad/s RMS (${a.rmsResidualIdentityOffset.toStringAsFixed(3)} with zero offset), '
            '${a.samples} pairs, mean rate ${a.meanRate.toStringAsFixed(2)} rad/s',
      ),
      ('Saved', 'results/$imuAlignmentFile'),
    ];
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
class _TextDialog extends StatefulWidget {
  const _TextDialog({required this.title, required this.label, required this.initial, required this.action});

  final String title;
  final String label;
  final String initial;
  final String action;

  @override
  State<_TextDialog> createState() => _TextDialogState();
}

class _TextDialogState extends State<_TextDialog> {
  late final _controller = TextEditingController(text: widget.initial);

  @override
  void dispose() {
    _controller.dispose();
    super.dispose();
  }

  @override
  Widget build(BuildContext context) {
    return AlertDialog(
      title: Text(widget.title),
      content: TextField(
        controller: _controller,
        autofocus: true,
        decoration: InputDecoration(labelText: widget.label),
      ),
      actions: [
        TextButton(onPressed: () => Navigator.pop(context), child: const Text('Cancel')),
        FilledButton(onPressed: () => Navigator.pop(context, _controller.text.trim()), child: Text(widget.action)),
      ],
    );
  }
}
