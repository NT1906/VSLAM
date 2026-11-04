import 'package:flutter/material.dart';
import 'package:provider/provider.dart';

import '../services/session_controller.dart';
import '../slam/slam_service.dart';
import '../widgets/ar_camera_view.dart';
import '../widgets/diagnostics_panel.dart';
import '../widgets/sensor_status_panel.dart';

class HomeScreen extends StatefulWidget {
  const HomeScreen({super.key});

  @override
  State<HomeScreen> createState() => _HomeScreenState();
}

class _HomeScreenState extends State<HomeScreen> with WidgetsBindingObserver {
  String? _shownError;

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
    _maybeShowError(session.lastError);

    return Scaffold(
      appBar: AppBar(
        title: const Text('VSLAM'),
        actions: [
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

  void _showHelp(BuildContext context) {
    showDialog<void>(
      context: context,
      builder: (context) => AlertDialog(
        title: const Text('How to use VSLAM'),
        content: const SingleChildScrollView(
          child: Text(
            'Mid-evaluation build: camera and IMU acquisition.\n\n'
            '1. Allow camera access. Hold the phone upright and pan slowly until the badge says SLAM tracking '
            '(ARCore/ARKit is tracking image features).\n'
            '2. Press Start. The AR camera session and the accelerometer/gyroscope start recording.\n'
            '3. The tiles show the camera tracking state, frame rate, AR pose count and IMU rates. '
            'The heart-monitor icon opens the full diagnostics.\n'
            '4. Press Stop to end the session.',
          ),
        ),
        actions: [TextButton(onPressed: () => Navigator.pop(context), child: const Text('Got it'))],
      ),
    );
  }
}
