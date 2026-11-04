package com.vslam.app

import android.Manifest
import android.app.Activity
import android.content.Context
import android.content.pm.PackageManager
import android.os.Handler
import android.os.Looper
import com.google.ar.core.ArCoreApk
import com.google.ar.core.exceptions.UnavailableUserDeclinedInstallationException
import io.flutter.embedding.engine.plugins.FlutterPlugin
import io.flutter.embedding.engine.plugins.activity.ActivityAware
import io.flutter.embedding.engine.plugins.activity.ActivityPluginBinding
import io.flutter.plugin.common.EventChannel
import io.flutter.plugin.common.MethodCall
import io.flutter.plugin.common.MethodChannel
import io.flutter.plugin.common.PluginRegistry
import io.flutter.plugin.common.StandardMessageCodec
import io.flutter.plugin.platform.PlatformView
import io.flutter.plugin.platform.PlatformViewFactory

/**
 * Bridges ARCore to Dart.
 *
 * - `vslam/ar` (method channel): availability, install, camera permission,
 *   frame rate.
 * - `vslam/ar_pose` (event channel): camera poses from the active AR view.
 * - `vslam/ar_frames` (event channel): grayscale camera images with the pose
 *   and intrinsics of that image, for the marker detector.
 * - `vslam/ar_view` (platform view): camera preview that owns the ARCore session.
 */
class ArPlugin :
    FlutterPlugin,
    ActivityAware,
    MethodChannel.MethodCallHandler,
    EventChannel.StreamHandler,
    PluginRegistry.RequestPermissionsResultListener {

    companion object {
        const val VIEW_TYPE = "vslam/ar_view"
        private const val CAMERA_REQUEST = 4201
    }

    private val mainHandler = Handler(Looper.getMainLooper())
    private var methodChannel: MethodChannel? = null
    private var poseChannel: EventChannel? = null
    private var poseSink: EventChannel.EventSink? = null
    private var frameChannel: EventChannel? = null
    @Volatile private var frameSink: EventChannel.EventSink? = null

    /** Minimum spacing of streamed camera frames; 0 disables them. */
    @Volatile var frameIntervalMs = 100L
        private set

    val wantsFrames: Boolean get() = frameSink != null && frameIntervalMs > 0
    private var activityBinding: ActivityPluginBinding? = null
    private var appContext: Context? = null
    private var pendingPermission: MethodChannel.Result? = null
    private var installRequested = false

    val activity: Activity? get() = activityBinding?.activity
    var activeView: ArPlatformView? = null

    override fun onAttachedToEngine(binding: FlutterPlugin.FlutterPluginBinding) {
        appContext = binding.applicationContext
        methodChannel = MethodChannel(binding.binaryMessenger, "vslam/ar").also {
            it.setMethodCallHandler(this)
        }
        poseChannel = EventChannel(binding.binaryMessenger, "vslam/ar_pose").also {
            it.setStreamHandler(this)
        }
        frameChannel = EventChannel(binding.binaryMessenger, "vslam/ar_frames").also {
            it.setStreamHandler(object : EventChannel.StreamHandler {
                override fun onListen(arguments: Any?, events: EventChannel.EventSink?) {
                    frameSink = events
                }

                override fun onCancel(arguments: Any?) {
                    frameSink = null
                }
            })
        }
        binding.platformViewRegistry.registerViewFactory(VIEW_TYPE, ArViewFactory(this))
    }

    override fun onDetachedFromEngine(binding: FlutterPlugin.FlutterPluginBinding) {
        methodChannel?.setMethodCallHandler(null)
        poseChannel?.setStreamHandler(null)
        frameChannel?.setStreamHandler(null)
        methodChannel = null
        poseChannel = null
        frameChannel = null
        appContext = null
    }

    override fun onAttachedToActivity(binding: ActivityPluginBinding) {
        activityBinding = binding
        binding.addRequestPermissionsResultListener(this)
    }

    override fun onDetachedFromActivityForConfigChanges() = onDetachedFromActivity()

    override fun onReattachedToActivityForConfigChanges(binding: ActivityPluginBinding) =
        onAttachedToActivity(binding)

    override fun onDetachedFromActivity() {
        activityBinding?.removeRequestPermissionsResultListener(this)
        activityBinding = null
    }

    fun onHostResume() = activeView?.resume()

    fun onHostPause() = activeView?.pause()

    fun hasCameraPermission(): Boolean {
        val ctx = appContext ?: return false
        return ctx.checkSelfPermission(Manifest.permission.CAMERA) == PackageManager.PERMISSION_GRANTED
    }

    override fun onMethodCall(call: MethodCall, result: MethodChannel.Result) {
        when (call.method) {
            "checkAvailability" -> result.success(checkAvailability())
            "requestInstall" -> requestInstall(result)
            "hasCameraPermission" -> result.success(hasCameraPermission())
            "requestCameraPermission" -> requestCameraPermission(result)
            "setFrameRate" -> {
                val hz = (call.argument<Number>("hz") ?: 0).toDouble()
                frameIntervalMs = if (hz > 0) (1000.0 / hz).toLong() else 0L
                result.success(null)
            }
            else -> result.notImplemented()
        }
    }

    private fun checkAvailability(): String {
        val ctx = appContext ?: return "unsupported"
        val availability = ArCoreApk.getInstance().checkAvailability(ctx)
        return when {
            availability.isTransient -> "unknown"
            availability == ArCoreApk.Availability.SUPPORTED_INSTALLED -> "supported"
            availability.isSupported -> "needsInstall"
            else -> "unsupported"
        }
    }

    private fun requestInstall(result: MethodChannel.Result) {
        val act = activity ?: return result.error("no_activity", "No activity", null)
        try {
            when (ArCoreApk.getInstance().requestInstall(act, !installRequested)) {
                ArCoreApk.InstallStatus.INSTALLED -> result.success("installed")
                ArCoreApk.InstallStatus.INSTALL_REQUESTED -> {
                    installRequested = true
                    result.success("requested")
                }
                else -> result.success("requested")
            }
        } catch (e: UnavailableUserDeclinedInstallationException) {
            result.success("declined")
        } catch (e: Exception) {
            result.error("install_failed", e.message, null)
        }
    }

    private fun requestCameraPermission(result: MethodChannel.Result) {
        if (hasCameraPermission()) return result.success(true)
        val act = activity ?: return result.success(false)
        pendingPermission?.success(false)
        pendingPermission = result
        act.requestPermissions(arrayOf(Manifest.permission.CAMERA), CAMERA_REQUEST)
    }

    override fun onRequestPermissionsResult(
        requestCode: Int,
        permissions: Array<out String>,
        grantResults: IntArray,
    ): Boolean {
        if (requestCode != CAMERA_REQUEST) return false
        val granted = grantResults.isNotEmpty() && grantResults[0] == PackageManager.PERMISSION_GRANTED
        pendingPermission?.success(granted)
        pendingPermission = null
        return true
    }

    override fun onListen(arguments: Any?, events: EventChannel.EventSink?) {
        poseSink = events
    }

    override fun onCancel(arguments: Any?) {
        poseSink = null
    }

    /** Sends an event to Dart. Safe to call from any thread. */
    fun sendEvent(event: Map<String, Any?>) {
        mainHandler.post { poseSink?.success(event) }
    }

    /** Sends a camera frame to Dart. Safe to call from any thread. */
    fun sendFrame(frame: Map<String, Any?>) {
        mainHandler.post { frameSink?.success(frame) }
    }
}

class ArViewFactory(private val plugin: ArPlugin) : PlatformViewFactory(StandardMessageCodec.INSTANCE) {
    override fun create(context: Context, viewId: Int, args: Any?): PlatformView =
        ArPlatformView(context, plugin)
}
