import 'dart:async';
import 'dart:io';

import 'package:flutter/material.dart';
import 'package:path_provider/path_provider.dart';
import 'package:provider/provider.dart';

import '../calibration/board.dart';
import '../calibration/camera_calibration.dart';
import '../slam/slam_service.dart';

/// Camera calibration: capture 15–20 views of the printed chessboard from
/// the live camera, run OpenCV calibration, compare with ARKit, save.
///
/// Shown in a bottom sheet over the home screen so the camera view above
/// stays visible for aiming.
class CalibrationPanel extends StatefulWidget {
  const CalibrationPanel({super.key, this.documents});

  /// Where `calibration.json` lives; the app's Documents folder by default.
  final Future<Directory> Function()? documents;

  /// The spec asks for 15 to 20 views of the board.
  static const minViews = 15;
  static const maxViews = 20;

  @override
  State<CalibrationPanel> createState() => _CalibrationPanelState();
}

class _CalibrationPanelState extends State<CalibrationPanel> {
  final _views = <BoardView>[];
  late final _square = TextEditingController(text: '25.0');
  CalibrationResult? _result;
  CalibrationResult? _saved;
  String _status = 'Hold the board fully in view, then Capture. Vary distance, tilt and position.';
  bool _busy = false;

  Future<Directory> _documents() => (widget.documents ?? getApplicationDocumentsDirectory)();

  /// The square side typed in, or the printed 25 mm when the text is not a
  /// positive number.
  double get _squareMm {
    final mm = double.tryParse(_square.text.trim().replaceAll(',', '.'));
    return mm != null && mm > 0 ? mm : 25.0;
  }

  CalibrationBoard get _board => CalibrationBoard.standard.withSquare(_squareMm / 1000);

  @override
  void initState() {
    super.initState();
    _documents().then(CalibrationResult.load).then((r) {
      if (mounted) setState(() => _saved = r);
    }).catchError((Object e) {
      if (mounted) setState(() => _status = 'Could not read the saved calibration: $e');
    });
  }

  @override
  void dispose() {
    _square.dispose();
    super.dispose();
  }

  Future<void> _capture() => _run(() async {
    final slam = context.read<SlamService>();
    final frame = await slam.frames.first.timeout(const Duration(seconds: 2));
    final jpeg = frame.jpeg;
    if (jpeg == null) {
      _status = 'No image in that frame, try again.';
      return;
    }
    final corners = await detectBoard(jpeg, _board);
    if (corners == null) {
      _status = 'Board not found (all ${_board.cols}×${_board.rows} inner corners must be visible).';
      return;
    }
    _views.add(BoardView(corners: corners, arkit: frame.intrinsics));
    _result = null;
    _status = 'View ${_views.length}: ${corners.length} corners.';
  });

  Future<void> _calibrate() => _run(() async {
    final r = await calibrate(_views, _board);
    await r.save(await _documents());
    _result = r;
    _saved = r;
    _status = 'Calibrated and saved to calibration.json.';
  });

  Future<void> _run(Future<void> Function() action) async {
    setState(() => _busy = true);
    try {
      await action();
    } on TimeoutException {
      _status = 'No camera frame arrived. Is the camera running?';
    } catch (e) {
      _status = '$e';
    } finally {
      if (mounted) setState(() => _busy = false);
    }
  }

  @override
  Widget build(BuildContext context) {
    final theme = Theme.of(context);
    final shown = _result ?? _saved;
    final w = _views.isEmpty ? 960 : _views.first.arkit.width;
    final h = _views.isEmpty ? 720 : _views.first.arkit.height;
    return Column(
      crossAxisAlignment: CrossAxisAlignment.stretch,
      mainAxisSize: MainAxisSize.min,
      children: [
        Row(
          children: [
            Expanded(child: Text('Camera calibration', style: theme.textTheme.titleMedium)),
            if (Navigator.of(context).canPop())
              TextButton(onPressed: () => Navigator.of(context).maybePop(), child: const Text('Close')),
          ],
        ),
        const SizedBox(height: 8),
        Row(
          children: [
            Expanded(
              child: TextField(
                controller: _square,
                enabled: _views.isEmpty,
                keyboardType: const TextInputType.numberWithOptions(decimal: true),
                textInputAction: TextInputAction.done,
                // The iOS number pad has no Done key: the check button and a
                // tap anywhere else close it.
                onTapOutside: (_) => FocusScope.of(context).unfocus(),
                onChanged: (_) => setState(() {}),
                onSubmitted: (_) => setState(() {}),
                decoration: InputDecoration(
                  labelText: 'Measured square side (mm)',
                  isDense: true,
                  helperText: _views.isEmpty
                      ? 'Captures will use ${_squareMm.toStringAsFixed(1)} mm'
                      : 'Locked at ${_squareMm.toStringAsFixed(1)} mm for this calibration',
                  suffixIcon: IconButton(
                    icon: const Icon(Icons.check),
                    tooltip: 'Done',
                    onPressed: _views.isEmpty
                        ? () {
                            FocusScope.of(context).unfocus();
                            setState(() {});
                          }
                        : null,
                  ),
                ),
              ),
            ),
            const SizedBox(width: 12),
            Text('${_views.length} views (${CalibrationPanel.minViews}–${CalibrationPanel.maxViews})\n'
                'coverage ${(coverage(_views, w, h) * 100).round()} %'),
          ],
        ),
        const SizedBox(height: 8),
        Text(_status, style: theme.textTheme.bodySmall),
        if (_busy) const Padding(padding: EdgeInsets.only(top: 6), child: LinearProgressIndicator()),
        const SizedBox(height: 8),
        Wrap(
          spacing: 8,
          children: [
            FilledButton.icon(
              onPressed: _busy || _views.length >= CalibrationPanel.maxViews ? null : _capture,
              icon: const Icon(Icons.camera_alt_outlined),
              label: const Text('Capture'),
            ),
            OutlinedButton(
              onPressed: _busy || _views.isEmpty ? null : () => setState(() => _views.removeLast()),
              child: const Text('Undo'),
            ),
            OutlinedButton(
              onPressed: _busy || _views.length < CalibrationPanel.minViews ? null : _calibrate,
              child: const Text('Calibrate'),
            ),
          ],
        ),
        if (shown != null) ...[
          const SizedBox(height: 12),
          _ComparisonTable(result: shown),
        ],
      ],
    );
  }
}

class _ComparisonTable extends StatelessWidget {
  const _ComparisonTable({required this.result});

  final CalibrationResult result;

  @override
  Widget build(BuildContext context) {
    final c = result.intrinsics;
    final a = result.arkitMean;
    final style = Theme.of(context).textTheme.bodySmall;
    TableRow row(String name, double calib, double? arkit, {int digits = 1}) => TableRow(children: [
          Text(name, style: style),
          Text(calib.toStringAsFixed(digits), style: style, textAlign: TextAlign.right),
          Text(arkit?.toStringAsFixed(digits) ?? '—', style: style, textAlign: TextAlign.right),
          Text(arkit == null ? '' : (calib - arkit).toStringAsFixed(digits), style: style, textAlign: TextAlign.right),
        ]);
    return Column(
      crossAxisAlignment: CrossAxisAlignment.start,
      children: [
        Text(
          '${result.views} views, RMS ${result.rms.toStringAsFixed(3)} px, ${c.width}×${c.height}, '
          'square ${(result.board.squareM * 1000).toStringAsFixed(1)} mm, ${result.created.toLocal()}',
          style: style,
        ),
        const SizedBox(height: 4),
        Table(
          children: [
            TableRow(children: [
              for (final t in ['', 'OpenCV', 'ARKit', 'diff']) Text(t, style: style?.copyWith(fontWeight: FontWeight.bold), textAlign: TextAlign.right),
            ]),
            row('fx', c.fx, a.fx),
            row('fy', c.fy, a.fy),
            row('cx', c.cx, a.cx),
            row('cy', c.cy, a.cy),
            row('k1', c.k1, null, digits: 4),
            row('k2', c.k2, null, digits: 4),
          ],
        ),
      ],
    );
  }
}
