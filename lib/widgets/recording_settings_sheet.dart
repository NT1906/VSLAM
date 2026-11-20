import 'package:flutter/material.dart';

import '../dataset/session_meta.dart';

/// Form for the settings stored with the next recording: label, experiment,
/// marker dictionary and side length, notes.
class RecordingSettingsSheet extends StatefulWidget {
  const RecordingSettingsSheet({super.key, required this.initial});

  final RecordingSettings initial;

  /// Shows the sheet; returns the new settings, or null when cancelled.
  static Future<RecordingSettings?> show(BuildContext context, RecordingSettings initial) =>
      showModalBottomSheet<RecordingSettings>(
        context: context,
        isScrollControlled: true,
        showDragHandle: true,
        builder: (_) => RecordingSettingsSheet(initial: initial),
      );

  @override
  State<RecordingSettingsSheet> createState() => _RecordingSettingsSheetState();
}

class _RecordingSettingsSheetState extends State<RecordingSettingsSheet> {
  late final _label = TextEditingController(text: widget.initial.label);
  late final _sideCm = TextEditingController(text: (widget.initial.markerSideM * 100).toStringAsFixed(1));
  late final _notes = TextEditingController(text: widget.initial.notes);
  late ExperimentType _experiment = widget.initial.experiment;
  String? _sideError;

  @override
  void dispose() {
    _label.dispose();
    _sideCm.dispose();
    _notes.dispose();
    super.dispose();
  }

  void _save() {
    final cm = double.tryParse(_sideCm.text.replaceAll(',', '.'));
    if (cm == null || cm <= 0 || cm > 100) {
      setState(() => _sideError = 'Enter the printed side length in cm (0–100)');
      return;
    }
    Navigator.pop(
      context,
      widget.initial.copyWith(
        label: _label.text.trim().isEmpty ? 'run' : _label.text.trim(),
        experiment: _experiment,
        markerSideM: cm / 100,
        notes: _notes.text.trim(),
      ),
    );
  }

  @override
  Widget build(BuildContext context) {
    return Padding(
      padding: EdgeInsets.fromLTRB(16, 0, 16, 16 + MediaQuery.viewInsetsOf(context).bottom),
      child: SingleChildScrollView(
        child: Column(
          crossAxisAlignment: CrossAxisAlignment.stretch,
          mainAxisSize: MainAxisSize.min,
          children: [
            Text('Next recording', style: Theme.of(context).textTheme.titleMedium),
            const SizedBox(height: 12),
            TextField(
              controller: _label,
              decoration: const InputDecoration(labelText: 'Label (folder name)', border: OutlineInputBorder()),
            ),
            const SizedBox(height: 12),
            DropdownButtonFormField<ExperimentType>(
              initialValue: _experiment,
              decoration: const InputDecoration(labelText: 'Experiment', border: OutlineInputBorder()),
              items: [for (final e in ExperimentType.values) DropdownMenuItem(value: e, child: Text(e.label))],
              onChanged: (e) => setState(() => _experiment = e ?? _experiment),
            ),
            const SizedBox(height: 12),
            Row(
              crossAxisAlignment: CrossAxisAlignment.start,
              children: [
                Expanded(
                  child: InputDecorator(
                    decoration: const InputDecoration(labelText: 'Marker dictionary', border: OutlineInputBorder()),
                    child: Text(widget.initial.markerDictionary),
                  ),
                ),
                const SizedBox(width: 12),
                Expanded(
                  child: TextField(
                    controller: _sideCm,
                    keyboardType: const TextInputType.numberWithOptions(decimal: true),
                    decoration: InputDecoration(
                      labelText: 'Marker side (cm)',
                      border: const OutlineInputBorder(),
                      errorText: _sideError,
                      errorMaxLines: 2,
                    ),
                  ),
                ),
              ],
            ),
            const SizedBox(height: 12),
            TextField(
              controller: _notes,
              minLines: 2,
              maxLines: 4,
              decoration: const InputDecoration(labelText: 'Notes', border: OutlineInputBorder()),
            ),
            const SizedBox(height: 16),
            FilledButton(onPressed: _save, child: const Text('Save settings')),
          ],
        ),
      ),
    );
  }
}
