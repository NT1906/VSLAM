import 'package:flutter/material.dart';
import 'package:provider/provider.dart';

import '../dataset/session_store.dart';
import '../services/session_controller.dart';
import '../slam/slam_service.dart';
import '../widgets/ar_camera_view.dart';
import '../widgets/diagnostics_panel.dart';
import '../widgets/recording_settings_sheet.dart';
import '../widgets/sensor_status_panel.dart';
import 'session_detail_screen.dart';
import 'sessions_screen.dart';

class HomeScreen extends StatefulWidget {
  const HomeScreen({super.key});

  @override
  State<HomeScreen> createState() => _HomeScreenState();
}

class _HomeScreenState extends State<HomeScreen> with WidgetsBindingObserver {
  String? _shownError;
  String? _shownSaved;

  @override
  void initState() {
    super.initState();
    WidgetsBinding.instance.addObserver(this);
    context.read<SlamService>().initialize();
  }

  @override
  void dispose() {
    WidgetsBinding.instance.removeObserver(this);
    super.dispose();
  }

  @override
  void didChangeAppLifecycleState(AppLifecycleState state) {
    // Re-check after the user returns from the ARCore installer or the
    // system permission settings.
    if (state == AppLifecycleState.resumed) {
      final slam = context.read<SlamService>();
      if (!slam.isReady) slam.initialize();
    }
  }

  @override
  Widget build(BuildContext context) {
    final session = context.watch<SessionController>();
    final store = context.read<SessionStore?>();
    _maybeShowError(session.lastError);
    if (store != null) _maybeShowSaved(session, store);

    return Scaffold(
      appBar: AppBar(
        title: const Text('VSLAM'),
        actions: [
          if (store != null)
            IconButton(
              icon: const Icon(Icons.folder_open_outlined),
              tooltip: 'Sessions',
              onPressed: () =>
                  Navigator.push<void>(context, MaterialPageRoute(builder: (_) => SessionsScreen(store: store))),
            ),
          IconButton(
            icon: const Icon(Icons.monitor_heart_outlined),
            tooltip: 'Diagnostics',
            onPressed: () => showModalBottomSheet<void>(
              context: context,
              showDragHandle: true,
              builder: (_) =>
                  const SingleChildScrollView(padding: EdgeInsets.fromLTRB(12, 0, 12, 24), child: DiagnosticsPanel()),
            ),
          ),
          IconButton(icon: const Icon(Icons.help_outline), tooltip: 'How to use', onPressed: () => _showHelp(context)),
        ],
      ),
      body: const SafeArea(
        child: Padding(
          padding: EdgeInsets.fromLTRB(12, 0, 12, 0),
          child: Column(
            children: [
              Expanded(child: ArCameraView()),
              SizedBox(height: 6),
              _RecordingBar(),
              SizedBox(height: 6),
              SensorStatusPanel(),
            ],
          ),
        ),
      ),
      bottomNavigationBar: SafeArea(
        child: Padding(
          padding: const EdgeInsets.fromLTRB(12, 8, 12, 12),
          child: Row(
            children: [
              IconButton.outlined(
                icon: const Icon(Icons.restart_alt),
                tooltip: 'Clear',
                onPressed: session.isRunning || !session.hasData ? null : session.reset,
              ),
              const SizedBox(width: 8),
              Expanded(
                child: session.isRunning
                    ? FilledButton.icon(
                        style: FilledButton.styleFrom(
                          backgroundColor: Theme.of(context).colorScheme.error,
                          foregroundColor: Theme.of(context).colorScheme.onError,
                          minimumSize: const Size.fromHeight(52),
                        ),
                        icon: const Icon(Icons.stop),
                        label: const Text('Stop'),
                        onPressed: session.stop,
                      )
                    : FilledButton.icon(
                        style: FilledButton.styleFrom(minimumSize: const Size.fromHeight(52)),
                        icon: const Icon(Icons.play_arrow),
                        label: const Text('Start'),
                        onPressed: session.start,
                      ),
              ),
            ],
          ),
        ),
      ),
    );
  }

  void _maybeShowError(String? error) {
    if (error == null || error == _shownError) return;
    _shownError = error;
    WidgetsBinding.instance.addPostFrameCallback((_) {
      if (!mounted) return;
      ScaffoldMessenger.of(context).showSnackBar(SnackBar(content: Text(error)));
    });
  }

  void _maybeShowSaved(SessionController session, SessionStore store) {
    final saved = session.lastSaved;
    if (saved == null || saved.directory.path == _shownSaved) return;
    _shownSaved = saved.directory.path;
    final counts = saved.meta?.counts;
    WidgetsBinding.instance.addPostFrameCallback((_) {
      if (!mounted) return;
      ScaffoldMessenger.of(context).showSnackBar(
        SnackBar(
          content: Text(
            'Saved ${saved.name}: ${counts?.frames ?? 0} frames, '
            '${(counts?.accelerometer ?? 0) + (counts?.gyroscope ?? 0)} IMU samples',
          ),
          action: SnackBarAction(
            label: 'Open',
            onPressed: () => Navigator.push<void>(
              context,
              MaterialPageRoute(
                builder: (_) => SessionDetailScreen(store: store, directory: saved.directory),
              ),
            ),
          ),
        ),
      );
    });
  }

  void _showHelp(BuildContext context) {
    showDialog<void>(
      context: context,
      builder: (context) => AlertDialog(
        title: const Text('How to use VSLAM'),
        content: const SingleChildScrollView(
          child: Text(
            '1. Allow camera access. Hold the phone upright and pan slowly until the badge says SLAM tracking '
            '(ARKit is tracking image features).\n'
            '2. Tap the settings row above the tiles to set the label, experiment, marker size and notes.\n'
            '3. Press Start. Camera frames, ARKit poses and the accelerometer/gyroscope are saved to a new '
            'session folder.\n'
            '4. The tiles show the camera tracking state, frame rate, AR pose count and IMU rates. '
            'The heart-monitor icon opens the full diagnostics.\n'
            '5. Press Stop. The folder icon lists saved sessions: open, rename, export or delete them. '
            'Sessions are also visible in the Files app under VSLAM.',
          ),
        ),
        actions: [TextButton(onPressed: () => Navigator.pop(context), child: const Text('Got it'))],
      ),
    );
  }
}

/// The settings of the next recording (tap to edit), or what is being
/// recorded right now.
class _RecordingBar extends StatelessWidget {
  const _RecordingBar();

  @override
  Widget build(BuildContext context) {
    final session = context.watch<SessionController>();
    final s = session.settings;
    final theme = Theme.of(context);
    final String text;
    if (session.isRunning) {
      text = session.store == null
          ? 'Recording (not saved: no storage)'
          : 'Recording "${s.label}": ${session.recordedFrames} frames saved';
    } else if (session.isSaving) {
      text = 'Finishing files…';
    } else {
      text = '${s.label} · ${s.experiment.label} · marker ${(s.markerSideM * 100).toStringAsFixed(1)} cm';
    }
    return Material(
      color: theme.colorScheme.surfaceContainerHighest,
      borderRadius: BorderRadius.circular(12),
      child: InkWell(
        borderRadius: BorderRadius.circular(12),
        onTap: session.isRunning || session.isSaving
            ? null
            : () async {
                final updated = await RecordingSettingsSheet.show(context, s);
                if (updated != null) await session.updateSettings(updated);
              },
        child: Padding(
          padding: const EdgeInsets.symmetric(horizontal: 12, vertical: 8),
          child: Row(
            children: [
              Icon(
                session.isRunning ? Icons.fiber_manual_record : Icons.tune,
                size: 18,
                color: session.isRunning ? theme.colorScheme.error : theme.colorScheme.onSurfaceVariant,
              ),
              const SizedBox(width: 8),
              Expanded(
                child: Text(text, style: theme.textTheme.bodyMedium, maxLines: 1, overflow: TextOverflow.ellipsis),
              ),
              if (!session.isRunning && !session.isSaving)
                Icon(
                  Icons.edit_outlined,
                  size: 18,
                  color: theme.colorScheme.onSurfaceVariant,
                  semanticLabel: 'Recording settings',
                ),
            ],
          ),
        ),
      ),
    );
  }
}
