import ARKit
import AVFoundation
import CoreMotion
import Flutter
import ImageIO
import UIKit
import UniformTypeIdentifiers

/// Bridges ARKit to Dart.
///
/// - `vslam/ar` (method channel): availability, camera permission, frame rate.
/// - `vslam/ar_pose` (event channel): camera poses from the active AR view.
/// - `vslam/ar_frames` (event channel): grayscale camera images (raw luma and
///   a JPEG of the same pixels) with the pose and intrinsics of that image.
/// - `vslam/imu` (event channel): raw accelerometer and gyroscope samples from
///   CoreMotion at 100 Hz.
/// - `vslam/ar_view` (platform view): camera preview that owns the ARSession.
///
/// Timestamps: `ARFrame.timestamp` and `CMLogItem.timestamp` both count
/// seconds since boot on the same host clock. Every timestamp sent to Dart is
/// converted with the one `epochMicros` below (one `bootEpoch` offset), so
/// camera, pose and IMU times compare directly. Any remaining camera–IMU delay
/// (exposure, sensor pipeline) is estimated from data in WP2.
final class ArPlugin: NSObject, FlutterPlugin, FlutterStreamHandler {
  static let viewType = "vslam/ar_view"

  private var poseSink: FlutterEventSink?
  let frames = FrameStream()
  private(set) lazy var imu = ImuStream(toEpochMicros: { [unowned self] in self.epochMicros($0) })

  /// Minimum spacing of streamed frames in seconds; 0 disables them.
  var frameInterval: TimeInterval = 0.1

  /// Epoch seconds at system boot (ARFrame timestamps count from boot).
  let bootEpoch = Date().timeIntervalSince1970 - ProcessInfo.processInfo.systemUptime

  static func register(with registrar: FlutterPluginRegistrar) {
    let instance = ArPlugin()
    let channel = FlutterMethodChannel(name: "vslam/ar", binaryMessenger: registrar.messenger())
    registrar.addMethodCallDelegate(instance, channel: channel)
    let poses = FlutterEventChannel(name: "vslam/ar_pose", binaryMessenger: registrar.messenger())
    poses.setStreamHandler(instance)
    let frameChannel = FlutterEventChannel(name: "vslam/ar_frames", binaryMessenger: registrar.messenger())
    frameChannel.setStreamHandler(instance.frames)
    let imuChannel = FlutterEventChannel(name: "vslam/imu", binaryMessenger: registrar.messenger())
    imuChannel.setStreamHandler(instance.imu)
    registrar.register(ArViewFactory(plugin: instance), withId: viewType)
  }

  func epochMicros(_ timestamp: TimeInterval) -> Double { (bootEpoch + timestamp) * 1_000_000 }

  func handle(_ call: FlutterMethodCall, result: @escaping FlutterResult) {
    switch call.method {
    case "checkAvailability":
      result(ARWorldTrackingConfiguration.isSupported ? "supported" : "unsupported")
    case "requestInstall":
      result("installed")
    case "setFrameRate":
      let hz = ((call.arguments as? [String: Any])?["hz"] as? NSNumber)?.doubleValue ?? 0
      frameInterval = hz > 0 ? 1.0 / hz : 0
      result(nil)
    case "deviceInfo":
      result(Self.deviceInfo())
    case "hasCameraPermission":
      result(AVCaptureDevice.authorizationStatus(for: .video) == .authorized)
    case "requestCameraPermission":
      switch AVCaptureDevice.authorizationStatus(for: .video) {
      case .authorized:
        result(true)
      case .notDetermined:
        AVCaptureDevice.requestAccess(for: .video) { granted in
          DispatchQueue.main.async { result(granted) }
        }
      default:
        // Denied or restricted: only the Settings app can change it.
        result(false)
      }
    default:
      result(FlutterMethodNotImplemented)
    }
  }

  /// Hardware model identifier (e.g. "iPhone15,2"), OS and app version, for
  /// the session metadata.
  static func deviceInfo() -> [String: String] {
    var sys = utsname()
    uname(&sys)
    let machine = withUnsafePointer(to: &sys.machine) {
      $0.withMemoryRebound(to: CChar.self, capacity: 1) { String(cString: $0) }
    }
    let info = Bundle.main.infoDictionary
    return [
      "model": machine,
      "os": "\(UIDevice.current.systemName) \(UIDevice.current.systemVersion)",
      "appVersion": info?["CFBundleShortVersionString"] as? String ?? "unknown",
      "appBuild": info?["CFBundleVersion"] as? String ?? "unknown",
    ]
  }

  func onListen(withArguments arguments: Any?, eventSink events: @escaping FlutterEventSink) -> FlutterError? {
    poseSink = events
    return nil
  }

  func onCancel(withArguments arguments: Any?) -> FlutterError? {
    poseSink = nil
    return nil
  }

  /// Sends an event to Dart. Safe to call from any thread.
  func send(_ event: [String: Any]) {
    if Thread.isMainThread {
      poseSink?(event)
    } else {
      DispatchQueue.main.async { self.poseSink?(event) }
    }
  }
}

/// Event sink for camera frames (a separate stream handler per channel).
final class FrameStream: NSObject, FlutterStreamHandler {
  private(set) var sink: FlutterEventSink?

  func onListen(withArguments arguments: Any?, eventSink events: @escaping FlutterEventSink) -> FlutterError? {
    sink = events
    return nil
  }

  func onCancel(withArguments arguments: Any?) -> FlutterError? {
    sink = nil
    return nil
  }

  func send(_ event: [String: Any]) {
    DispatchQueue.main.async { self.sink?(event) }
  }
}

/// Raw accelerometer and gyroscope from CoreMotion.
///
/// Events are `Float64List` batches of `[sensor, t_us, x, y, z] × n`, with
/// sensor 0 = accelerometer, 1 = gyroscope, sent about every 50 ms.
///
/// Units and signs (the convention the Dart side already used with
/// sensors_plus, so existing code and tests keep their meaning):
/// - accelerometer: m/s², `−acceleration × 9.80665`. CoreMotion reports
///   acceleration in g with the sign of gravity (−1 g on the axis that points
///   up at rest); negating it gives the specific force, +9.81 m/s² along the
///   axis that points up when the phone is at rest.
/// - gyroscope: rad/s, right-handed about the device axes, unchanged.
/// Device axes: x right, y up (toward the top of the screen in portrait),
/// z out of the screen.
final class ImuStream: NSObject, FlutterStreamHandler {
  static let rateHz = 100.0
  static let standardGravity = 9.80665

  private let motion = CMMotionManager()
  private let queue: OperationQueue = {
    let q = OperationQueue()
    q.name = "vslam.imu"
    q.maxConcurrentOperationCount = 1 // serial: the buffer is only touched here
    return q
  }()
  private let toEpochMicros: (TimeInterval) -> Double
  private var sink: FlutterEventSink?
  private var buffer: [Double] = []
  private var lastFlush: TimeInterval = 0

  init(toEpochMicros: @escaping (TimeInterval) -> Double) {
    self.toEpochMicros = toEpochMicros
    super.init()
  }

  func onListen(withArguments arguments: Any?, eventSink events: @escaping FlutterEventSink) -> FlutterError? {
    guard motion.isAccelerometerAvailable, motion.isGyroAvailable else {
      return FlutterError(code: "UNAVAILABLE", message: "Accelerometer or gyroscope not available", details: nil)
    }
    sink = events
    motion.accelerometerUpdateInterval = 1.0 / Self.rateHz
    motion.gyroUpdateInterval = 1.0 / Self.rateHz
    motion.startAccelerometerUpdates(to: queue) { [weak self] data, error in
      guard let self else { return }
      if let error { self.fail(error); return }
      guard let data else { return }
      let g = Self.standardGravity
      self.append(0, data.timestamp, -data.acceleration.x * g, -data.acceleration.y * g, -data.acceleration.z * g)
    }
    motion.startGyroUpdates(to: queue) { [weak self] data, error in
      guard let self else { return }
      if let error { self.fail(error); return }
      guard let data else { return }
      self.append(1, data.timestamp, data.rotationRate.x, data.rotationRate.y, data.rotationRate.z)
    }
    return nil
  }

  func onCancel(withArguments arguments: Any?) -> FlutterError? {
    motion.stopAccelerometerUpdates()
    motion.stopGyroUpdates()
    sink = nil
    queue.addOperation { [weak self] in self?.buffer.removeAll() }
    return nil
  }

  /// Runs on `queue`.
  private func append(_ sensor: Double, _ timestamp: TimeInterval, _ x: Double, _ y: Double, _ z: Double) {
    buffer.append(contentsOf: [sensor, toEpochMicros(timestamp), x, y, z])
    guard timestamp - lastFlush >= 0.05 else { return }
    lastFlush = timestamp
    let batch = buffer.withUnsafeBufferPointer { Data(buffer: $0) }
    buffer.removeAll(keepingCapacity: true)
    DispatchQueue.main.async { [weak self] in
      self?.sink?(FlutterStandardTypedData(float64: batch))
    }
  }

  private func fail(_ error: Error) {
    DispatchQueue.main.async { [weak self] in
      self?.sink?(FlutterError(code: "UNAVAILABLE", message: error.localizedDescription, details: nil))
    }
  }
}

final class ArViewFactory: NSObject, FlutterPlatformViewFactory {
  private let plugin: ArPlugin

  init(plugin: ArPlugin) {
    self.plugin = plugin
    super.init()
  }

  func create(withFrame frame: CGRect, viewIdentifier viewId: Int64, arguments args: Any?) -> FlutterPlatformView {
    ArPlatformView(frame: frame, plugin: plugin)
  }

  func createArgsCodec() -> FlutterMessageCodec & NSObjectProtocol {
    FlutterStandardMessageCodec.sharedInstance()
  }
}

/// ARSCNView showing the camera feed; streams the camera pose at ~30 Hz and
/// downsampled luma images at the requested frame rate.
///
/// Streamed frame width: `capturedImage` is box-filtered by the integer
/// factor `max(1, width / targetWidth)`; for the usual 1920×1440 capture that
/// is 2, giving 960×720.
final class ArPlatformView: NSObject, FlutterPlatformView, ARSessionDelegate {
  private let sceneView: ARSCNView
  private weak var plugin: ArPlugin?
  private var lastSent: TimeInterval = 0
  private var lastFrameSent: TimeInterval = 0
  private var lastState: String?
  private var converting = false
  private let imageQueue = DispatchQueue(label: "vslam.frames", qos: .userInitiated)
  static let targetWidth = 960
  static let jpegQuality = 0.95

  init(frame: CGRect, plugin: ArPlugin) {
    sceneView = ARSCNView(frame: frame)
    self.plugin = plugin
    super.init()
    sceneView.automaticallyUpdatesLighting = false
    sceneView.session.delegate = self
    run()
  }

  deinit {
    sceneView.session.pause()
  }

  func view() -> UIView { sceneView }

  private func run() {
    guard ARWorldTrackingConfiguration.isSupported else {
      send(state: "stopped", reason: "ARKit world tracking not supported")
      return
    }
    let config = ARWorldTrackingConfiguration()
    config.worldAlignment = .gravity
    config.planeDetection = []
    // Fixed focus keeps the focal length (and so the intrinsics) constant for
    // the whole run. Calibration (WP2) is done with the same setting.
    config.isAutoFocusEnabled = false
    sceneView.session.run(config, options: [.resetTracking, .removeExistingAnchors])
    send(state: "initializing", reason: nil)
  }

  private func send(state: String, reason: String?, pose: simd_float4x4? = nil, timestamp: TimeInterval? = nil) {
    lastState = state
    var event: [String: Any] = ["state": state]
    if let timestamp, let plugin { event["ts"] = plugin.epochMicros(timestamp) }
    if let reason { event["reason"] = reason }
    if let m = pose {
      let t = m.columns.3
      let q = simd_quatf(m)
      event["t"] = [Double(t.x), Double(t.y), Double(t.z)]
      event["q"] = [Double(q.imag.x), Double(q.imag.y), Double(q.imag.z), Double(q.real)]
    }
    plugin?.send(event)
  }

  // MARK: ARSessionDelegate (main queue)

  func session(_ session: ARSession, didUpdate frame: ARFrame) {
    let camera = frame.camera
    let state: String
    var reason: String?
    switch camera.trackingState {
    case .normal:
      state = "tracking"
    case .notAvailable:
      state = "stopped"
      reason = "Tracking not available"
    case .limited(let r):
      switch r {
      case .initializing:
        state = "initializing"
      case .excessiveMotion:
        state = "limited"
        reason = "Moving too fast"
      case .insufficientFeatures:
        state = "limited"
        reason = "Not enough texture"
      case .relocalizing:
        state = "limited"
        reason = "Relocalizing"
      @unknown default:
        state = "limited"
        reason = "Tracking limited"
      }
    }
    // ARKit delivers frames at 60 Hz. A 0.1 s interval is exactly six
    // frame periods, and timestamp jitter would make a strict `>=` test skip
    // to the seventh frame about half the time (≈ 8.6 Hz instead of 10 Hz), so
    // allow half a frame period of slack.
    if let plugin, plugin.frames.sink != nil, plugin.frameInterval > 0,
       frame.timestamp - lastFrameSent >= plugin.frameInterval - 0.008, !converting {
      lastFrameSent = frame.timestamp
      sendImage(frame, state: state, plugin: plugin)
    }
    if state == lastState && frame.timestamp - lastSent < 1.0 / 30.0 { return }
    lastSent = frame.timestamp
    // camera.transform is the physical camera pose in the gravity-aligned
    // world frame (+y up, camera looks down its −z axis).
    send(state: state, reason: reason, pose: state == "tracking" ? camera.transform : nil, timestamp: frame.timestamp)
  }

  /// Box-filters the luma plane of `capturedImage` (sensor orientation,
  /// e.g. 1920×1440) by an integer factor down to about `targetWidth` on a
  /// background queue, rescales the intrinsics to match, and JPEG-encodes the
  /// result so the dataset recorder can write it without re-encoding.
  ///
  /// Intrinsics: `camera.intrinsics` refers to `camera.imageResolution`,
  /// the size of `capturedImage`. An s×s box average maps the centre of
  /// output pixel u to input coordinate s·u + (s−1)/2, so a principal point
  /// c becomes (c + 0.5)/s − 0.5 and focal lengths divide by s
  /// (`CameraIntrinsics.scaledTo` in Dart is the same formula).
  private func sendImage(_ frame: ARFrame, state: String, plugin: ArPlugin) {
    converting = true
    let buffer = frame.capturedImage
    let k = frame.camera.intrinsics
    let transform = frame.camera.transform
    let ts = plugin.epochMicros(frame.timestamp)
    imageQueue.async { [weak self] in
      defer { DispatchQueue.main.async { self?.converting = false } }
      CVPixelBufferLockBaseAddress(buffer, .readOnly)
      defer { CVPixelBufferUnlockBaseAddress(buffer, .readOnly) }
      guard CVPixelBufferGetPlaneCount(buffer) >= 1,
            let base = CVPixelBufferGetBaseAddressOfPlane(buffer, 0) else { return }
      let width = CVPixelBufferGetWidthOfPlane(buffer, 0)
      let height = CVPixelBufferGetHeightOfPlane(buffer, 0)
      let stride = CVPixelBufferGetBytesPerRowOfPlane(buffer, 0)
      let s = max(1, width / ArPlatformView.targetWidth)
      let w = width / s, h = height / s
      let src = base.assumingMemoryBound(to: UInt8.self)
      var out = Data(count: w * h)
      out.withUnsafeMutableBytes { (dst: UnsafeMutableRawBufferPointer) in
        let d = dst.bindMemory(to: UInt8.self)
        for y in 0..<h {
          for x in 0..<w {
            var sum = 0
            for dy in 0..<s {
              let row = src + (y * s + dy) * stride + x * s
              for dx in 0..<s { sum += Int(row[dx]) }
            }
            d[y * w + x] = UInt8(sum / (s * s))
          }
        }
      }
      let scale = Double(s)
      let jpeg = encodeGrayJpeg(out, width: w, height: h, quality: ArPlatformView.jpegQuality)
      // simd_float3x3 is column-major: fx = [0][0], fy = [1][1], cx = [2][0], cy = [2][1].
      var event: [String: Any] = [
        "ts": ts,
        "w": w,
        "h": h,
        "y": FlutterStandardTypedData(bytes: out),
        "srcW": width,
        "srcH": height,
        "k": [
          Double(k[0][0]) / scale, Double(k[1][1]) / scale,
          (Double(k[2][0]) + 0.5) / scale - 0.5, (Double(k[2][1]) + 0.5) / scale - 0.5,
        ],
        "state": state,
      ]
      if let jpeg {
        event["jpg"] = FlutterStandardTypedData(bytes: jpeg)
        event["jpgQ"] = ArPlatformView.jpegQuality
      }
      if state == "tracking" {
        let t = transform.columns.3
        let q = simd_quatf(transform)
        event["t"] = [Double(t.x), Double(t.y), Double(t.z)]
        event["q"] = [Double(q.imag.x), Double(q.imag.y), Double(q.imag.z), Double(q.real)]
      }
      plugin.frames.send(event)
    }
  }

  func session(_ session: ARSession, didFailWithError error: Error) {
    send(state: "stopped", reason: error.localizedDescription)
  }

  func sessionWasInterrupted(_ session: ARSession) {
    send(state: "limited", reason: "Session interrupted")
  }

  /// Relocalize after an interruption instead of resetting, so the world
  /// origin (and the recorded trajectory) stays consistent.
  func sessionShouldAttemptRelocalization(_ session: ARSession) -> Bool { true }
}

/// Encodes an 8-bit grayscale image as JPEG with ImageIO.
private func encodeGrayJpeg(_ pixels: Data, width: Int, height: Int, quality: Double) -> Data? {
  guard let provider = CGDataProvider(data: pixels as CFData),
        let image = CGImage(
          width: width, height: height, bitsPerComponent: 8, bitsPerPixel: 8, bytesPerRow: width,
          space: CGColorSpaceCreateDeviceGray(), bitmapInfo: CGBitmapInfo(rawValue: CGImageAlphaInfo.none.rawValue),
          provider: provider, decode: nil, shouldInterpolate: false, intent: .defaultIntent)
  else { return nil }
  let out = NSMutableData()
  guard let destination = CGImageDestinationCreateWithData(out as CFMutableData, UTType.jpeg.identifier as CFString, 1, nil)
  else { return nil }
  CGImageDestinationAddImage(destination, image, [kCGImageDestinationLossyCompressionQuality: quality] as CFDictionary)
  guard CGImageDestinationFinalize(destination) else { return nil }
  return out as Data
}
