import 'dart:math' as math;

import 'package:vector_math/vector_math_64.dart';

import 'config.dart';

/// A time-ordered series of 3-vectors (one IMU sensor) that can be read at
/// any time inside its span by linear interpolation.
class ImuSeries {
  ImuSeries(this.tUs, this.values) : assert(tUs.length == values.length);

  final List<int> tUs;
  final List<Vector3> values;

  bool get isEmpty => tUs.isEmpty;
  int get length => tUs.length;
  int get firstUs => tUs.first;
  int get lastUs => tUs.last;

  /// Index of the last sample with time ≤ [t] (−1 when before the first).
  int indexAtOrBefore(int t) {
    var lo = 0;
    var hi = tUs.length - 1;
    if (tUs.isEmpty || t < tUs[0]) return -1;
    if (t >= tUs[hi]) return hi;
    while (hi - lo > 1) {
      final mid = (lo + hi) >> 1;
      if (tUs[mid] <= t) {
        lo = mid;
      } else {
        hi = mid;
      }
    }
    return lo;
  }

  /// Linear interpolation at [t] (µs, may be fractional), clamped to the
  /// first / last sample outside the span.
  Vector3 at(double t) {
    if (t <= tUs.first) return values.first.clone();
    if (t >= tUs.last) return values.last.clone();
    final i = indexAtOrBefore(t.floor());
    final a = tUs[i];
    final b = tUs[i + 1];
    final f = b == a ? 0.0 : (t - a) / (b - a);
    return values[i] * (1 - f) + values[i + 1] * f;
  }
}

/// The dead-reckoning state of the handout, `[p, v, q, b_a, b_g]`:
///
/// - [p]: position of the device (IMU) in the world W, metres;
/// - [v]: its velocity in W, m/s;
/// - [q]: orientation `q_wb` (Hamilton, maps device vectors into W);
/// - [ba]: accelerometer bias, device axes, m/s²;
/// - [bg]: gyroscope bias, device axes, rad/s.
///
/// W is gravity aligned with z up.
class ImuState {
  ImuState({required this.tUs, required this.p, required this.v, required this.q, required this.ba, required this.bg});

  int tUs;
  Vector3 p;
  Vector3 v;
  Quaternion q;
  Vector3 ba;
  Vector3 bg;

  Matrix3 get rWb => q.asRotationMatrix();

  ImuState copy() =>
      ImuState(tUs: tUs, p: p.clone(), v: v.clone(), q: Quaternion.copy(q), ba: ba.clone(), bg: bg.clone());
}

/// Unit quaternion of the rotation vector [w] (axis × angle), exact for any
/// angle, including zero.
Quaternion quatExp(Vector3 w) {
  final theta = w.length;
  if (theta < 1e-12) return Quaternion(w.x / 2, w.y / 2, w.z / 2, 1)..normalize();
  final s = math.sin(theta / 2) / theta;
  return Quaternion(w.x * s, w.y * s, w.z * s, math.cos(theta / 2));
}

/// Strapdown integration of raw accelerometer and gyroscope samples.
///
/// Measurement model (device axes, specific force with +g up at rest, as
/// recorded in `imu.csv`):
///
///     ω_m = ω + b_g + noise
///     f_m = R_wbᵀ (a + g·e_z) + b_a + noise
///
/// so the world acceleration is `a = R_wb (f_m − b_a) − g·e_z`.
///
/// One step from t to t + Δt uses the mean of the two interpolated samples
/// at the ends of the step (trapezoid in the sensor readings):
///
///     ω̄ = ½(ω_m(t) + ω_m(t+Δt)) − b_g
///     q ← q ⊗ Exp(ω̄ Δt)                          (exact for constant ω)
///     a = R_wb(t + Δt/2) (f̄ − b_a) − g e_z,  R_wb(t + Δt/2) = R_wb(t) Exp(ω̄ Δt/2)
///     p ← p + v Δt + ½ a Δt²,  v ← v + a Δt     (exact for constant a)
///
/// The step boundaries are the gyroscope sample times; the accelerometer is
/// interpolated onto them. Biases are held constant (no bias random walk:
/// over a run of a minute they are dominated by the initial estimate).
class ImuPropagator {
  ImuPropagator({required this.acc, required this.gyro, this.gravity = 9.80665});

  final ImuSeries acc;
  final ImuSeries gyro;
  final double gravity;

  /// Advances [s] to time [toUs] (≥ `s.tUs`) in place.
  void propagate(ImuState s, int toUs) {
    if (toUs < s.tUs) throw ArgumentError('propagate backwards: ${s.tUs} → $toUs');
    var t = s.tUs;
    var i = gyro.indexAtOrBefore(t) + 1; // next gyro sample after t
    while (t < toUs) {
      final next = i < gyro.length && gyro.tUs[i] < toUs ? gyro.tUs[i] : toUs;
      if (next > t) step(s, t, next);
      t = next;
      i++;
    }
    s.tUs = toUs;
  }

  /// One integration step from [t0] to [t1] (µs).
  void step(ImuState s, int t0, int t1) {
    final dt = (t1 - t0) * 1e-6;
    final w = (gyro.at(t0.toDouble()) + gyro.at(t1.toDouble())) * 0.5 - s.bg;
    final f = (acc.at(t0.toDouble()) + acc.at(t1.toDouble())) * 0.5 - s.ba;
    final rMid = (s.q * quatExp(w * (dt / 2))).asRotationMatrix();
    final a = rMid.transformed(f)..z -= gravity;
    s.p = s.p + s.v * dt + a * (0.5 * dt * dt);
    s.v = s.v + a * dt;
    s.q = (s.q * quatExp(w * dt))..normalize();
    s.tUs = t1;
  }
}

/// Mean and largest per-axis standard deviation of the samples of [series]
/// in [t0, t1] (µs).
({Vector3 mean, double maxStd, int count}) windowStats(ImuSeries series, int t0, int t1) {
  final start = math.max(0, series.indexAtOrBefore(t0 - 1) + 1);
  var n = 0;
  final sum = Vector3.zero();
  final sum2 = Vector3.zero();
  for (var i = start; i < series.length && series.tUs[i] <= t1; i++) {
    final v = series.values[i];
    sum.add(v);
    sum2.add(Vector3(v.x * v.x, v.y * v.y, v.z * v.z));
    n++;
  }
  if (n == 0) return (mean: Vector3.zero(), maxStd: double.infinity, count: 0);
  final mean = sum / n.toDouble();
  double sd(double s2, double m) => math.sqrt(math.max(0, s2 / n - m * m));
  final maxStd = [sd(sum2.x, mean.x), sd(sum2.y, mean.y), sd(sum2.z, mean.z)].reduce(math.max);
  return (mean: mean, maxStd: maxStd, count: n);
}

/// Biases estimated from a window in which the phone was still.
class RestEstimate {
  const RestEstimate({
    required this.t0Us,
    required this.t1Us,
    required this.atRest,
    required this.ba,
    required this.bg,
    required this.gyroStd,
    required this.accStd,
    required this.samples,
    required this.accMean,
  });

  final int t0Us;
  final int t1Us;

  /// True when both standard deviations were under the configured limits.
  /// When false the biases are set to zero: a window with motion in it says
  /// nothing reliable about them.
  final bool atRest;

  /// Accelerometer bias `b_a = mean(f_m) − R_wbᵀ g e_z` (device axes).
  final Vector3 ba;

  /// Gyroscope bias `b_g = mean(ω_m)` (device axes).
  final Vector3 bg;

  /// Largest per-axis standard deviations in the window.
  final double gyroStd;
  final double accStd;

  /// Gyroscope samples in the window.
  final int samples;

  /// Mean accelerometer reading, device axes (|·| ≈ g at rest).
  final Vector3 accMean;

  Map<String, dynamic> toJson() => {
        't0_us': t0Us,
        't1_us': t1Us,
        'at_rest': atRest,
        'b_a': [ba.x, ba.y, ba.z],
        'b_g': [bg.x, bg.y, bg.z],
        'gyro_std_rad_s': gyroStd,
        'acc_std_m_s2': accStd,
        'samples': samples,
        'acc_mean': [accMean.x, accMean.y, accMean.z],
      };
}

/// Estimates the biases from the window [t0Us, t0Us + restSeconds], with
/// the device orientation [rWb] (from ARKit) during it.
///
/// At rest the gyroscope measures only its bias, and the accelerometer
/// measures gravity in device axes plus its bias. The orientation is needed
/// to know which part of the mean accelerometer reading is gravity; any
/// small error in it (camera–IMU rotation, ARKit tilt) ends up in `b_a`,
/// which is what makes the propagation start without a spurious
/// acceleration.
RestEstimate estimateRestBiases(ImuSeries acc, ImuSeries gyro, int t0Us, Matrix3 rWb, SlamConfig config) {
  final t1Us = t0Us + (config.imuRestSeconds * 1e6).round();
  final g = windowStats(gyro, t0Us, t1Us);
  final a = windowStats(acc, t0Us, t1Us);
  final enough = g.count >= 20 && a.count >= 20;
  final atRest = enough && g.maxStd < config.imuRestMaxGyroStd && a.maxStd < config.imuRestMaxAccStd;
  final gravityInDevice = rWb.transposed().transformed(Vector3(0, 0, config.gravity));
  return RestEstimate(
    t0Us: t0Us,
    t1Us: t1Us,
    atRest: atRest,
    ba: atRest ? a.mean - gravityInDevice : Vector3.zero(),
    bg: atRest ? g.mean : Vector3.zero(),
    gyroStd: g.maxStd,
    accStd: a.maxStd,
    samples: g.count,
    accMean: a.mean,
  );
}

/// The first window of `imuRestSeconds` that starts at or after [fromUs]
/// (scanned in 0.1 s steps up to `imuRestSearchSeconds` later) in which
/// both sensors are still. Null when there is none.
int? findRestWindow(ImuSeries acc, ImuSeries gyro, int fromUs, SlamConfig config) {
  final len = (config.imuRestSeconds * 1e6).round();
  final lastStart = fromUs + (config.imuRestSearchSeconds * 1e6).round();
  for (var t = fromUs; t <= lastStart && t + len <= math.min(acc.lastUs, gyro.lastUs); t += 100000) {
    final g = windowStats(gyro, t, t + len);
    final a = windowStats(acc, t, t + len);
    if (g.count >= 20 && a.count >= 20 && g.maxStd < config.imuRestMaxGyroStd && a.maxStd < config.imuRestMaxAccStd) {
      return t;
    }
  }
  return null;
}
