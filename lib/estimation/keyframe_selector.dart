import 'dart:math' as math;

import '../core/se3.dart';
import 'config.dart';

/// Why a frame became a keyframe (spec 4.4). A keyframe can have several.
enum KeyframeReason {
  /// The first tracked frame of the run.
  first('first'),

  /// Moved at least [SlamConfig.keyframeTranslationM] since the previous one.
  translation('move'),

  /// Turned at least [SlamConfig.keyframeRotationDeg] since the previous one.
  rotation('turn'),

  /// A usable marker detection at least [SlamConfig.keyframeMarkerGapS]
  /// after the previous keyframe.
  marker('marker');

  const KeyframeReason(this.code);

  /// Short name used in `traj_arkit.csv`.
  final String code;

  static KeyframeReason fromCode(String code) =>
      values.firstWhere((r) => r.code == code, orElse: () => throw FormatException('Unknown keyframe reason "$code"'));
}

/// A frame chosen as a keyframe, with the ARKit pose of that frame.
class Keyframe {
  const Keyframe({
    required this.index,
    required this.frameIndex,
    required this.tUs,
    required this.vTc,
    required this.reasons,
    required this.markerIds,
    required this.trackingGapBefore,
  });

  /// Position in the keyframe list (0, 1, 2, ...).
  final int index;

  /// Index of the frame in the session (`frames.csv` order, 0-based).
  final int frameIndex;
  final int tUs;

  /// ARKit pose of this image: OpenCV camera in the VIO world (`vTc`).
  final Pose3 vTc;
  final Set<KeyframeReason> reasons;

  /// IDs of the usable marker detections in this frame (may be empty).
  final List<int> markerIds;

  /// True when at least one frame between the previous keyframe and this
  /// one had no ARKit tracking. The odometry across such a gap is less
  /// trustworthy (ARKit may have relocalized); WP5 inflates its noise.
  final bool trackingGapBefore;
}

/// Picks keyframes from a stream of frames, one decision per frame, so the
/// same code serves batch processing and a live run.
///
/// A frame without an ARKit pose is never a keyframe: there would be no
/// odometry to connect it. The first tracked frame is always a keyframe.
/// After that a tracked frame becomes one when, relative to the previous
/// keyframe, the camera has moved ≥ `keyframeTranslationM`, or turned
/// ≥ `keyframeRotationDeg`, or the frame has a usable marker detection and
/// ≥ `keyframeMarkerGapS` have passed. Distances are measured to the last
/// keyframe, not accumulated along the path, so standing still and
/// swaying never triggers a keyframe by itself.
class KeyframeSelector {
  KeyframeSelector([this.config = const SlamConfig()]);

  final SlamConfig config;
  final List<Keyframe> keyframes = [];
  bool _gapSinceLast = false;

  Keyframe? get last => keyframes.isEmpty ? null : keyframes.last;

  /// Offers one frame; returns the new keyframe or null. [vTc] is null when
  /// ARKit was not tracking. [markerIds] are the IDs of the usable marker
  /// detections in the frame.
  Keyframe? offer({required int frameIndex, required int tUs, required Pose3? vTc, List<int> markerIds = const []}) {
    if (vTc == null) {
      if (keyframes.isNotEmpty) _gapSinceLast = true;
      return null;
    }
    final reasons = <KeyframeReason>{};
    final prev = last;
    if (prev == null) {
      reasons.add(KeyframeReason.first);
    } else {
      final rel = prev.vTc.inverse() * vTc;
      if (rel.translation.length >= config.keyframeTranslationM) reasons.add(KeyframeReason.translation);
      if (rotationAngle(rel.rotation) * 180 / math.pi >= config.keyframeRotationDeg) {
        reasons.add(KeyframeReason.rotation);
      }
      if (markerIds.isNotEmpty && tUs - prev.tUs >= (config.keyframeMarkerGapS * 1e6).round()) {
        reasons.add(KeyframeReason.marker);
      }
    }
    if (reasons.isEmpty) return null;
    final kf = Keyframe(
      index: keyframes.length,
      frameIndex: frameIndex,
      tUs: tUs,
      vTc: vTc,
      reasons: reasons,
      markerIds: List.unmodifiable(markerIds),
      trackingGapBefore: _gapSinceLast,
    );
    keyframes.add(kf);
    _gapSinceLast = false;
    return kf;
  }
}

/// ARKit odometry between two consecutive keyframes (spec 4.4):
/// `Z_k = (vTc_k)⁻¹ · vTc_k+1`, the motion of camera k+1 seen from camera k.
/// WP5 turns each into an odometry factor; baseline 1 chains them.
class OdometryEdge {
  OdometryEdge(Keyframe from, Keyframe to)
      : from = from.index,
        to = to.index,
        z = from.vTc.inverse() * to.vTc,
        trackingGap = to.trackingGapBefore;

  final int from;
  final int to;
  final Pose3 z;
  final bool trackingGap;

  /// Length of the step, metres (drives the translation noise in WP5).
  double get stepM => z.translation.length;

  /// Rotation of the step, degrees.
  double get rotationDeg => rotationAngle(z.rotation) * 180 / math.pi;
}

/// The edges between consecutive [keyframes].
List<OdometryEdge> odometryEdges(List<Keyframe> keyframes) => [
      for (var k = 0; k + 1 < keyframes.length; k++) OdometryEdge(keyframes[k], keyframes[k + 1]),
    ];
