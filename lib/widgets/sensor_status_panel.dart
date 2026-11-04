import 'package:flutter/material.dart';
import 'package:provider/provider.dart';

import '../services/imu_service.dart';
import '../services/session_controller.dart';
import '../slam/ar_pose.dart';
import '../slam/slam_service.dart';
import 'periodic_rebuild.dart';

/// Live status tiles for the camera (AR session) and the IMU.
class SensorStatusPanel extends StatelessWidget {
  const SensorStatusPanel({super.key});

  @override
  Widget build(BuildContext context) {
    return PeriodicRebuild(
      builder: (context) {
        final slam = context.watch<SlamService>();
        final imu = context.watch<ImuService>();
        final session = context.watch<SessionController>();
        final scheme = Theme.of(context).colorScheme;

        final (camColor, camLabel) = switch (slam.trackingState) {
          _ when !slam.isReady => (scheme.outline, 'off'),
          ArTrackingState.tracking => (Colors.green, 'tracking'),
          ArTrackingState.initializing => (Colors.amber.shade700, 'initialising'),
          ArTrackingState.limited => (Colors.orange, 'limited'),
          ArTrackingState.stopped => (scheme.error, 'stopped'),
        };
        final imuColor = imu.error != null
            ? scheme.error
            : imu.isRunning
            ? Colors.green
            : scheme.outline;

        final tiles = [
          _Stat('Camera', camLabel, color: camColor),
          _Stat('Frame rate', '${slam.frameHz.toStringAsFixed(1)} Hz'),
          _Stat('AR poses', '${session.trackedPoses}/${session.poses}'),
          _Stat('IMU', imu.error != null ? 'error' : (imu.isRunning ? 'streaming' : 'idle'), color: imuColor),
          _Stat('Accel / gyro', '${imu.accelerometerHz.toStringAsFixed(0)} / ${imu.gyroscopeHz.toStringAsFixed(0)} Hz'),
          _Stat('Elapsed', _clock(session.elapsed), color: session.isRunning ? scheme.error : null),
        ];

        return LayoutBuilder(
          builder: (context, constraints) => GridView.count(
            crossAxisCount: constraints.maxWidth > 520 ? 6 : 3,
            shrinkWrap: true,
            physics: const NeverScrollableScrollPhysics(),
            mainAxisSpacing: 6,
            crossAxisSpacing: 6,
            childAspectRatio: 2.3,
            children: tiles,
          ),
        );
      },
    );
  }

  static String _clock(Duration d) {
    final m = d.inMinutes.toString().padLeft(2, '0');
    final s = (d.inSeconds % 60).toString().padLeft(2, '0');
    return '$m:$s';
  }
}

class _Stat extends StatelessWidget {
  const _Stat(this.label, this.value, {this.color});

  final String label;
  final String value;
  final Color? color;

  @override
  Widget build(BuildContext context) {
    final theme = Theme.of(context);
    return DecoratedBox(
      decoration: BoxDecoration(
        color: theme.colorScheme.surfaceContainerHighest,
        borderRadius: BorderRadius.circular(12),
      ),
      child: Padding(
        padding: const EdgeInsets.symmetric(horizontal: 10, vertical: 4),
        child: Column(
          crossAxisAlignment: CrossAxisAlignment.start,
          mainAxisAlignment: MainAxisAlignment.center,
          children: [
            Text(
              label,
              style: theme.textTheme.labelSmall?.copyWith(color: theme.colorScheme.onSurfaceVariant),
              maxLines: 1,
              overflow: TextOverflow.ellipsis,
            ),
            FittedBox(
              fit: BoxFit.scaleDown,
              alignment: Alignment.centerLeft,
              child: Text(
                value,
                style: theme.textTheme.titleMedium?.copyWith(fontWeight: FontWeight.w600, color: color),
              ),
            ),
          ],
        ),
      ),
    );
  }
}
