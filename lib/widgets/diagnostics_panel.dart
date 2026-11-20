import 'package:flutter/material.dart';
import 'package:provider/provider.dart';
import 'package:vector_math/vector_math_64.dart' show Vector3;

import '../services/imu_service.dart';
import '../services/session_controller.dart';
import '../slam/slam_service.dart';
import 'periodic_rebuild.dart';

/// Detailed camera and IMU status for the person running the test.
class DiagnosticsPanel extends StatelessWidget {
  const DiagnosticsPanel({super.key});

  @override
  Widget build(BuildContext context) {
    return PeriodicRebuild(
      builder: (context) {
        final slam = context.read<SlamService>();
        final imu = context.read<ImuService>();
        final session = context.read<SessionController>();
        final k = slam.lastIntrinsics;
        String f(double? v, [int d = 1]) => v == null ? '—' : v.toStringAsFixed(d);
        String vec(Vector3? v) => v == null ? '—' : '(${f(v.x, 2)}, ${f(v.y, 2)}, ${f(v.z, 2)})';

        final rows = <(String, String)>[
          (
            'AR session',
            switch (slam.availability) {
              ArAvailability.checking => 'checking…',
              ArAvailability.supported => slam.cameraGranted ? 'supported' : 'camera permission needed',
              ArAvailability.needsInstall => 'needs Google Play Services for AR',
              ArAvailability.unsupported => 'not supported on this device',
            },
          ),
          ('Tracking', '${slam.trackingState.name}${slam.trackingReason == null ? '' : ' (${slam.trackingReason})'}'),
          ('Frames', '${f(slam.frameHz)} Hz in (requested ${f(slam.frameRate, 0)} Hz)'),
          ('Image', k == null ? '—' : '${k.width}×${k.height}, AR SDK intrinsics'),
          if (k != null) ('Intrinsics', 'fx ${f(k.fx)} fy ${f(k.fy)} cx ${f(k.cx)} cy ${f(k.cy)}'),
          ('Accelerometer', '${f(imu.accelerometerHz, 0)} Hz, ${imu.accelerometerSamples} samples'),
          ('  latest', '${vec(imu.lastAccel?.value)} m/s²'),
          ('Gyroscope', '${f(imu.gyroscopeHz, 0)} Hz, ${imu.gyroscopeSamples} samples'),
          ('  latest', '${vec(imu.lastGyro?.value)} rad/s'),
          (
            'Recording',
            session.isRunning
                ? 'running, ${session.poses} AR poses, ${session.recordedFrames} frames saved'
                : 'stopped',
          ),
          ('Device', '${session.device.model}, ${session.device.os}'),
          if (slam.error != null) ('AR error', slam.error!),
          if (imu.error != null) ('IMU error', imu.error!),
        ];
        final theme = Theme.of(context);
        return Card(
          margin: EdgeInsets.zero,
          child: Padding(
            padding: const EdgeInsets.all(10),
            child: Column(
              crossAxisAlignment: CrossAxisAlignment.start,
              children: [
                for (final (label, value) in rows)
                  Padding(
                    padding: const EdgeInsets.symmetric(vertical: 1.5),
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
                        Expanded(child: Text(value, style: theme.textTheme.bodySmall)),
                      ],
                    ),
                  ),
                const SizedBox(height: 4),
                Text(
                  'Camera frames, ARKit poses and IMU samples are timestamped on one clock (boot time). '
                  'The residual camera–IMU delay is estimated from a recording in the calibration step.',
                  style: theme.textTheme.bodySmall,
                ),
              ],
            ),
          ),
        );
      },
    );
  }
}
