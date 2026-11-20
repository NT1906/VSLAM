import 'dart:io';
import 'dart:math' as math;

import 'package:vector_math/vector_math_64.dart';
import 'package:vslam/calibration/imu_alignment.dart';
import 'package:vslam/core/se3.dart';
import 'package:vslam/dataset/sensor_event.dart';
import 'package:vslam/dataset/session_files.dart';
import 'package:vslam/dataset/session_meta.dart';
import 'package:vslam/dataset/session_writer.dart';
import 'package:vslam/estimation/config.dart';
import 'package:vslam/slam/ar_pose.dart';
import 'package:vslam/vision/aruco_detector.dart';
import 'package:vslam/vision/camera_model.dart';
import 'package:vslam/vision/marker_observation.dart';
import 'package:vslam/vision/session_detection.dart';

/// The calibrated iPhone 15 camera of WP2 at 960 × 720 (k1, k2 included).
const simIntrinsics = CameraIntrinsics(
  fx: 716.5,
  fy: 714.8,
  cx: 473.0,
  cy: 357.4,
  width: 960,
  height: 720,
  k1: 0.0715,
  k2: -0.1411,
);

/// Settings of one simulated recording. Defaults: perfect odometry and a
/// perfect IMU; [SimConfig.drifting] gives the drift of spec 4.5.
class SimConfig {
  const SimConfig({
    this.seed = 1,
    this.laps = 2,
    this.speed = 0.5,
    this.restS = 3.0,
    this.initializingS = 0.5,
    this.endRestS = 1.0,
    this.rampS = 1.0,
    this.cornerSigmaPx = 0.5,
    this.scaleError = 0,
    this.yawBiasDegPerM = 0,
    this.stepNoiseM = 0,
    this.stepNoiseYawDeg = 0,
    this.accBias = (0, 0, 0),
    this.gyroBias = (0, 0, 0),
    this.accNoise = 0,
    this.gyroNoise = 0,
    this.lostS,
    this.markerSideM = 0.15,
    this.intrinsics = simIntrinsics,
    this.gravity = 9.80665,
    this.vYawDeg = 37,
    this.vOffset = (0.4, -1.2, 0.1),
  });

  /// Spec 4.5's drifting odometry: 1 % scale error and a yaw bias; 0.06° per
  /// 0.25 m keyframe step is 0.24° per metre.
  const SimConfig.drifting({int seed = 1, int laps = 2})
      : this(seed: seed, laps: laps, scaleError: 0.01, yawBiasDegPerM: 0.24, stepNoiseM: 2e-4, stepNoiseYawDeg: 0.01);

  final int seed;
  final int laps;

  /// Walking speed along the path, m/s (approximate on the ellipse).
  final double speed;

  /// Still at the start (s), of which the first [initializingS] have no
  /// ARKit pose (ARKit "initializing"), and still at the end.
  final double restS;
  final double initializingS;
  final double endRestS;

  /// Length of the smooth speed-up and slow-down, s.
  final double rampS;

  /// Corner noise, pixels (Gaussian per coordinate).
  final double cornerSigmaPx;

  /// ARKit drift: translation increments scaled by 1 + [scaleError], and a
  /// heading error growing by [yawBiasDegPerM] per metre walked, plus white
  /// noise per 30 Hz step.
  final double scaleError;
  final double yawBiasDegPerM;
  final double stepNoiseM;
  final double stepNoiseYawDeg;

  /// IMU biases (device axes) and white noise per sample (σ).
  final (double, double, double) accBias;
  final (double, double, double) gyroBias;
  final double accNoise;
  final double gyroNoise;

  /// ARKit reports no pose in this interval (seconds from the start).
  final (double, double)? lostS;

  final double markerSideM;
  final CameraIntrinsics intrinsics;
  final double gravity;

  /// How ARKit's frame V sits relative to the true room frame.
  final double vYawDeg;
  final (double, double, double) vOffset;
}

/// A marker on a wall of the true room frame.
class SimMarker {
  const SimMarker(this.id, this.wTm);

  final int id;
  final Pose3 wTm;
}

/// One simulated 10 Hz frame.
class SimFrame {
  const SimFrame(this.tUs, this.truth, this.vTc, this.observations);

  final int tUs;

  /// True camera pose in the room frame.
  final Pose3 truth;

  /// ARKit's (drifting) pose, or null while ARKit is not tracking.
  final Pose3? vTc;

  /// Marker detections with noisy corners and real IPPE solutions.
  final List<MarkerObservation> observations;
}

Vector3 _v((double, double, double) t) => Vector3(t.$1, t.$2, t.$3);

/// A 6 × 5 m room with eight 15 cm markers on the walls and a person
/// walking an ellipse around its middle, looking outward at the walls with
/// a slow side-to-side sweep (spec section 6 simulator).
///
/// Produces what the phone would record: 10 Hz frames with ARKit poses,
/// 30 Hz ARKit poses, 100 Hz accelerometer and gyroscope (specific force,
/// +g up, device axes rotated from the camera by `appleCameraFromDevice`),
/// and the marker detections of each frame. Corners are the true
/// projections through the camera model plus Gaussian noise; the IPPE
/// solutions and the rejection rules are the app's own code
/// ([solveMarkerPnp], [MarkerObservation.evaluate]). Fixed seeds make every
/// run identical.
class RoomSimulation {
  RoomSimulation([this.config = const SimConfig()]) {
    _rng = math.Random(config.seed);
    _buildMarkers();
    final omega = config.speed / ((_a + _b) / 2);
    final total = 2 * math.pi * config.laps;
    _omega = omega;
    _cruiseS = (total - omega * config.rampS) / omega;
    durationS = config.restS + 2 * config.rampS + _cruiseS + config.endRestS;
    vTw = Pose3.yawTranslation(config.vYawDeg * math.pi / 180, _v(config.vOffset));
    _simulateArkit();
    _simulateImu();
  }

  final SimConfig config;
  late final math.Random _rng;

  static const t0Us = 1763300000000000;
  static const _centre = (3.0, 2.5);
  static const _a = 2.0;
  static const _b = 1.4;
  static const _phase = 0.3;

  late final double _omega;
  late final double _cruiseS;
  late final double durationS;

  /// ARKit's frame V seen from the true room frame.
  late final Pose3 vTw;

  final List<SimMarker> markers = [];
  final List<SimFrame> frames = [];
  final List<PoseEvent> poses = [];
  final List<ImuEvent> imu = [];

  /// `R_ci` used to generate the IMU (the app's default).
  Matrix3 get rCI => appleCameraFromDevice;

  double gaussian() {
    final u1 = 1 - _rng.nextDouble();
    final u2 = _rng.nextDouble();
    return math.sqrt(-2 * math.log(u1)) * math.cos(2 * math.pi * u2);
  }

  void _buildMarkers() {
    const w = 6.0;
    const h = 5.0;
    for (var j = 0; j < 8; j++) {
      final alpha = (j * 45 + 10) * math.pi / 180;
      final d = Vector3(math.cos(alpha), math.sin(alpha), 0);
      // Ray from the centre to the first wall it meets.
      final tx = d.x > 0 ? (w - _centre.$1) / d.x : (d.x < 0 ? -_centre.$1 / d.x : double.infinity);
      final ty = d.y > 0 ? (h - _centre.$2) / d.y : (d.y < 0 ? -_centre.$2 / d.y : double.infinity);
      final s = math.min(tx, ty);
      final n = tx < ty ? Vector3(d.x > 0 ? -1 : 1, 0, 0) : Vector3(0, d.y > 0 ? -1 : 1, 0); // into the room
      final p = Vector3(_centre.$1 + s * d.x, _centre.$2 + s * d.y, 1.35 + 0.15 * (j % 2));
      // Marker frame (spec 4.3): z out of the face (into the room), y up,
      // x = y × z (to the right as seen by a viewer facing the wall).
      final z = n;
      final y = Vector3(0, 0, 1);
      final x = y.cross(z);
      markers.add(SimMarker(j, Pose3(Matrix3.columns(x, y, z), p - n * 0.001)));
    }
  }

  /// Walked angle along the ellipse at [t] seconds: still, smooth speed-up
  /// (quintic smoothstep in speed), cruise, smooth slow-down, still.
  double phi(double t) {
    final r = config.rampS;
    final u = t - config.restS;
    double big(double x) => math.pow(x, 6) - 3 * math.pow(x, 5) + 2.5 * math.pow(x, 4); // ∫ smoothstep
    final total = 2 * math.pi * config.laps;
    if (u <= 0) return 0;
    if (u < r) return _omega * r * big(u / r);
    if (u < r + _cruiseS) return _omega * r / 2 + _omega * (u - r);
    if (u < 2 * r + _cruiseS) return total - _omega * r * big((2 * r + _cruiseS - u) / r);
    return total;
  }

  /// True camera pose `wTc` (OpenCV camera) in the room frame at [t] s.
  Pose3 truth(double t) {
    final f = phi(t);
    final alpha = f + _phase;
    final p = Vector3(_centre.$1 + _a * math.cos(alpha), _centre.$2 + _b * math.sin(alpha), 1.4 + 0.03 * math.sin(3 * f));
    final yaw = alpha + 0.35 * math.sin(1.5 * f);
    final pitch = 0.05 + 0.08 * math.sin(2 * f);
    final fwd = Vector3(math.cos(pitch) * math.cos(yaw), math.cos(pitch) * math.sin(yaw), math.sin(pitch));
    final right = fwd.cross(Vector3(0, 0, 1))..normalize();
    final down = fwd.cross(right);
    return Pose3(Matrix3.columns(right, down, fwd), p);
  }

  static int usAt(double t) => t0Us + (t * 1e6).round();

  bool _lost(double t) {
    final l = config.lostS;
    return t < config.initializingS || (l != null && t >= l.$1 && t < l.$2);
  }

  void _simulateArkit() {
    final n = (durationS * 30).floor();
    Pose3? drifted;
    var yawErr = 0.0;
    Pose3? prevTruth;
    final yawBias = config.yawBiasDegPerM * math.pi / 180;
    for (var i = 0; i <= n; i++) {
      final t = i / 30;
      final truthNow = truth(t);
      if (drifted == null) {
        if (!_lost(t)) drifted = truthNow.clone();
      } else {
        final dp = truthNow.translation - prevTruth!.translation;
        yawErr += yawBias * dp.length + config.stepNoiseYawDeg * math.pi / 180 * gaussian();
        final step = rotZ(yawErr).transformed(dp) * (1 + config.scaleError) +
            Vector3(gaussian(), gaussian(), gaussian()) * config.stepNoiseM;
        drifted = Pose3(rotZ(yawErr).multiplied(truthNow.rotation), drifted.translation + step);
      }
      prevTruth = truthNow;
      final tracked = drifted != null && !_lost(t);
      final vTc = tracked ? vTw * drifted : null;
      final tUs = t0Us + (i * 1e6 / 30).round();
      final ar = vTc == null ? null : vioToArPose(vTc);
      final state = tracked ? ArTrackingState.tracking : (t < config.initializingS ? ArTrackingState.initializing : ArTrackingState.limited);
      poses.add(PoseEvent(tUs, state, translation: ar?.translation, rotation: ar?.rotation));
      if (i % 3 == 0) frames.add(SimFrame(tUs, truthNow, vTc, _observe(truthNow)));
    }
  }

  List<MarkerObservation> _observe(Pose3 wTc) {
    final k = config.intrinsics;
    final out = <MarkerObservation>[];
    final cTw = wTc.inverse();
    for (final m in markers) {
      final cTm = cTw * m.wTm;
      if (cTm.translation.length > 5) continue;
      if (MarkerObservation.viewAngleOf(cTm) > 80) continue;
      final corners = <Vector2>[];
      for (final p in markerObjectPoints(config.markerSideM)) {
        final c = cTm.transform(p);
        if (c.z < 0.05) break;
        final px = k.project(c)!;
        if (px.x < 3 || px.y < 3 || px.x > k.width - 4 || px.y > k.height - 4) break;
        corners.add(px + Vector2(gaussian(), gaussian()) * config.cornerSigmaPx);
      }
      if (corners.length < 4) continue;
      out.add(MarkerObservation.evaluate(
        id: m.id,
        corners: corners,
        solutions: solveMarkerPnp(corners, k, config.markerSideM),
        config: const SlamConfig(),
      ));
    }
    return out;
  }

  void _simulateImu() {
    const h = 1e-3;
    final n = (durationS * 100).floor();
    final rCI = this.rCI;
    Matrix3 rWb(double t) => truth(t).rotation.multiplied(rCI);
    for (var i = 0; i <= n; i++) {
      final t = i / 100;
      final tUs = t0Us + i * 10000;
      final w = so3Log(rWb(t - h).transposed().multiplied(rWb(t + h))) / (2 * h);
      final a = (truth(t + h).translation - truth(t).translation * 2 + truth(t - h).translation) / (h * h);
      final f = rWb(t).transposed().transformed(a + Vector3(0, 0, config.gravity));
      final noiseA = Vector3(gaussian(), gaussian(), gaussian()) * config.accNoise;
      final noiseG = Vector3(gaussian(), gaussian(), gaussian()) * config.gyroNoise;
      imu.add(ImuEvent(tUs, ImuSensor.accelerometer, f + _v(config.accBias) + noiseA));
      imu.add(ImuEvent(tUs, ImuSensor.gyroscope, w + _v(config.gyroBias) + noiseG));
    }
  }

  /// The true room frame expressed in the map frame W that the pipeline
  /// builds from ARKit (given its `wTv`): `wTv · vTw`. Exact at the first
  /// tracked pose, where the simulated drift is still zero.
  Pose3 roomInW(Pose3 wTv) => wTv * vTw;

  /// Writes the recording as a session folder under [root], with
  /// `results/detections.csv` as the Detect markers step would have
  /// written it. Frames carry no image.
  Future<Directory> writeSession(Directory root, {String label = 'sim'}) async {
    final k = config.intrinsics;
    final pinhole = CameraIntrinsics(fx: k.fx, fy: k.fy, cx: k.cx, cy: k.cy, width: k.width, height: k.height);
    final writer = SessionWriter.create(
      root,
      SessionMeta(
        settings: RecordingSettings(label: label, experiment: ExperimentType.closedLoop, markerSideM: config.markerSideM),
        device: const DeviceInfo(model: 'simulator'),
        created: DateTime(2026, 11, 16, 12),
      ),
    );
    for (final f in frames) {
      final ar = f.vTc == null ? null : vioToArPose(f.vTc!);
      writer.addFrame(
        FrameEvent(
          f.tUs,
          intrinsics: pinhole,
          state: ar == null ? ArTrackingState.limited : ArTrackingState.tracking,
          translation: ar?.translation,
          rotation: ar?.rotation,
          loadJpeg: () async => null,
        ),
        null,
      );
    }
    poses.forEach(writer.addPose);
    imu.forEach(writer.addImu);
    await writer.close();
    final csv = StringBuffer()..writeln(MarkerObservation.csvHeader);
    for (final f in frames) {
      for (final m in f.observations) {
        csv.writeln(m.toCsvRow(f.tUs));
      }
    }
    await File('${writer.directory.path}/${SessionFiles.results}/$detectionsFile').writeAsString(csv.toString());
    return writer.directory;
  }
}
