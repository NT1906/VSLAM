import 'package:flutter/foundation.dart';
import 'package:flutter/gestures.dart';
import 'package:flutter/material.dart';
import 'package:flutter/rendering.dart';
import 'package:flutter/services.dart';
import 'package:provider/provider.dart';

import '../slam/ar_pose.dart';
import '../slam/slam_service.dart';

/// Live camera feed from the native AR session, with a tracking-state badge.
/// Falls back to an explanation card when AR can't run.
class ArCameraView extends StatelessWidget {
  const ArCameraView({super.key});

  static const viewType = 'vslam/ar_view';

  @override
  Widget build(BuildContext context) {
    final slam = context.watch<SlamService>();
    return ClipRRect(
      borderRadius: BorderRadius.circular(16),
      child: ColoredBox(
        color: Colors.black,
        child: switch (slam.availability) {
          ArAvailability.checking => const Center(child: CircularProgressIndicator()),
          ArAvailability.unsupported => const _Message(
              icon: Icons.videocam_off_outlined,
              title: 'SLAM not available',
              body: 'This device does not support ARCore/ARKit. '
                  'Dead reckoning still works on its own.',
            ),
          ArAvailability.needsInstall => _Message(
              icon: Icons.download_outlined,
              title: 'Google Play Services for AR needed',
              body: 'Install it to enable visual SLAM.',
              action: FilledButton(
                onPressed: slam.requestInstall,
                child: const Text('Install'),
              ),
            ),
          ArAvailability.supported when !slam.cameraGranted => _Message(
              icon: Icons.photo_camera_outlined,
              title: 'Camera access needed',
              body: 'Visual SLAM tracks the phone using the camera.',
              action: FilledButton(
                onPressed: slam.requestCameraPermission,
                child: const Text('Allow camera'),
              ),
            ),
          ArAvailability.supported => Stack(
              fit: StackFit.expand,
              children: [
                const _NativeArView(),
                Positioned(
                  left: 8,
                  top: 8,
                  child: _TrackingBadge(
                    state: slam.trackingState,
                    reason: slam.error ?? slam.trackingReason,
                    recording: slam.isRecording,
                  ),
                ),
              ],
            ),
        },
      ),
    );
  }
}

class _NativeArView extends StatefulWidget {
  const _NativeArView();

  @override
  State<_NativeArView> createState() => _NativeArViewState();
}

class _NativeArViewState extends State<_NativeArView> {
  late final SlamService _slam = context.read<SlamService>();

  @override
  void initState() {
    super.initState();
    _slam.attach();
  }

  @override
  void dispose() {
    _slam.detach();
    super.dispose();
  }

  @override
  Widget build(BuildContext context) {
    switch (defaultTargetPlatform) {
      case TargetPlatform.android:
        // Hybrid composition: the native GLSurfaceView renders the camera.
        return PlatformViewLink(
          viewType: ArCameraView.viewType,
          surfaceFactory: (context, controller) => AndroidViewSurface(
            controller: controller as AndroidViewController,
            gestureRecognizers: const <Factory<OneSequenceGestureRecognizer>>{},
            hitTestBehavior: PlatformViewHitTestBehavior.transparent,
          ),
          onCreatePlatformView: (params) => PlatformViewsService.initExpensiveAndroidView(
            id: params.id,
            viewType: ArCameraView.viewType,
            layoutDirection: TextDirection.ltr,
            creationParamsCodec: const StandardMessageCodec(),
            onFocus: () => params.onFocusChanged(true),
          )
            ..addOnPlatformViewCreatedListener(params.onPlatformViewCreated)
            ..create(),
        );
      case TargetPlatform.iOS:
        return const UiKitView(
          viewType: ArCameraView.viewType,
          creationParamsCodec: StandardMessageCodec(),
        );
      default:
        return const _Message(
          icon: Icons.devices_other,
          title: 'Unsupported platform',
          body: 'Run on an Android or iOS phone.',
        );
    }
  }
}

class _TrackingBadge extends StatelessWidget {
  const _TrackingBadge({required this.state, required this.recording, this.reason});

  final ArTrackingState state;
  final bool recording;
  final String? reason;

  @override
  Widget build(BuildContext context) {
    final (color, label) = switch (state) {
      ArTrackingState.tracking => (Colors.greenAccent, recording ? 'SLAM recording' : 'SLAM tracking'),
      ArTrackingState.initializing => (Colors.amberAccent, 'Move the phone slowly…'),
      ArTrackingState.limited => (Colors.orangeAccent, reason ?? 'Tracking limited'),
      ArTrackingState.stopped => (Colors.redAccent, reason ?? 'AR stopped'),
    };
    return DecoratedBox(
      decoration: BoxDecoration(
        color: Colors.black.withValues(alpha: 0.6),
        borderRadius: BorderRadius.circular(20),
      ),
      child: Padding(
        padding: const EdgeInsets.symmetric(horizontal: 10, vertical: 6),
        child: Row(
          mainAxisSize: MainAxisSize.min,
          children: [
            Icon(Icons.circle, size: 10, color: color),
            const SizedBox(width: 6),
            Text(label, style: const TextStyle(color: Colors.white, fontSize: 12)),
          ],
        ),
      ),
    );
  }
}

class _Message extends StatelessWidget {
  const _Message({required this.icon, required this.title, required this.body, this.action});

  final IconData icon;
  final String title;
  final String body;
  final Widget? action;

  @override
  Widget build(BuildContext context) {
    return Center(
      child: Padding(
        padding: const EdgeInsets.all(16),
        child: Column(
          mainAxisSize: MainAxisSize.min,
          children: [
            Icon(icon, color: Colors.white70, size: 36),
            const SizedBox(height: 8),
            Text(
              title,
              textAlign: TextAlign.center,
              style: const TextStyle(color: Colors.white, fontWeight: FontWeight.w600),
            ),
            const SizedBox(height: 4),
            Text(
              body,
              textAlign: TextAlign.center,
              style: const TextStyle(color: Colors.white70, fontSize: 13),
            ),
            if (action != null) ...[const SizedBox(height: 12), action!],
          ],
        ),
      ),
    );
  }
}
