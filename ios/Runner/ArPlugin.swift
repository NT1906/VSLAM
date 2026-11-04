import ARKit
import AVFoundation
import Flutter
import UIKit

/// Bridges ARKit to Dart.
///
/// - `vslam/ar` (method channel): availability, camera permission, frame rate.
/// - `vslam/ar_pose` (event channel): camera poses from the active AR view.
/// - `vslam/ar_frames` (event channel): grayscale camera images with the pose
///   and intrinsics of that image, for the marker detector.
/// - `vslam/ar_view` (platform view): camera preview that owns the ARSession.
///
/// Timestamps are microseconds since the Unix epoch on the boot-based clock
/// that sensors_plus uses for CoreMotion, so camera and IMU samples compare
/// directly.
final class ArPlugin: NSObject, FlutterPlugin, FlutterStreamHandler {
  static let viewType = "vslam/ar_view"

  private var poseSink: FlutterEventSink?
  let frames = FrameStream()

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
final class ArPlatformView: NSObject, FlutterPlatformView, ARSessionDelegate {
  private let sceneView: ARSCNView
  private weak var plugin: ArPlugin?
  private var lastSent: TimeInterval = 0
  private var lastFrameSent: TimeInterval = 0
  private var lastState: String?
  private var converting = false
  private let imageQueue = DispatchQueue(label: "vslam.frames", qos: .userInitiated)

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
    if let plugin, plugin.frames.sink != nil, plugin.frameInterval > 0,
       frame.timestamp - lastFrameSent >= plugin.frameInterval, !converting {
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
  /// e.g. 1920×1440) down to ≤ 640 px wide on a background queue, and
  /// rescales the intrinsics to match.
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
      let s = max(1, width / 640)
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
      // simd_float3x3 is column-major: fx = [0][0], fy = [1][1], cx = [2][0], cy = [2][1].
      var event: [String: Any] = [
        "ts": ts,
        "w": w,
        "h": h,
        "y": FlutterStandardTypedData(bytes: out),
        "k": [
          Double(k[0][0]) / scale, Double(k[1][1]) / scale,
          (Double(k[2][0]) + 0.5) / scale - 0.5, (Double(k[2][1]) + 0.5) / scale - 0.5,
        ],
        "state": state,
      ]
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
