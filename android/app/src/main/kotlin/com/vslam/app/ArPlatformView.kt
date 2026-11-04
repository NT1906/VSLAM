package com.vslam.app

import android.content.Context
import android.opengl.GLES20
import android.opengl.GLSurfaceView
import android.os.Build
import android.os.SystemClock
import android.util.Log
import android.view.Surface
import android.view.View
import android.view.WindowManager
import com.google.ar.core.ArCoreApk
import com.google.ar.core.Config
import com.google.ar.core.Session
import com.google.ar.core.TrackingFailureReason
import com.google.ar.core.TrackingState
import com.google.ar.core.Frame
import com.google.ar.core.exceptions.CameraNotAvailableException
import com.google.ar.core.exceptions.NotYetAvailableException
import com.google.ar.core.exceptions.SessionPausedException
import com.google.ar.core.exceptions.UnavailableException
import io.flutter.plugin.platform.PlatformView
import javax.microedition.khronos.egl.EGLConfig
import javax.microedition.khronos.opengles.GL10

/**
 * Camera preview that owns the ARCore session. Each GL frame updates the
 * session, draws the camera image and streams the camera pose to Dart; at
 * the requested rate it also streams the CPU camera image (luma plane) with
 * that image's pose and intrinsics.
 *
 * Timestamps are sent as microseconds since the Unix epoch on the same
 * boot-based clock that sensors_plus uses for the IMU, so camera and IMU
 * samples can be compared directly.
 */
class ArPlatformView(
    private val context: Context,
    private val plugin: ArPlugin,
) : PlatformView, GLSurfaceView.Renderer {

    companion object {
        private const val TAG = "VslamAr"
        private const val POSE_INTERVAL_MS = 33L // ~30 Hz
    }

    private val surfaceView = GLSurfaceView(context).apply {
        preserveEGLContextOnPause = true
        setEGLContextClientVersion(2)
        setEGLConfigChooser(8, 8, 8, 8, 16, 0)
        setRenderer(this@ArPlatformView)
        renderMode = GLSurfaceView.RENDERMODE_CONTINUOUSLY
    }
    private val background = BackgroundRenderer()

    // Accessed from both the UI and GL threads.
    @Volatile private var session: Session? = null
    @Volatile private var viewportChanged = false
    private var textureBoundTo: Session? = null
    private var viewportWidth = 0
    private var viewportHeight = 0
    private var lastPoseSent = 0L
    private var lastFrameSent = 0L
    private var lastState: String? = null
    private var resumed = false

    /** Epoch µs minus frame-clock µs, fixed on the first frame. */
    private var clockOffsetUs: Long? = null

    init {
        plugin.activeView = this
        resume()
    }

    override fun getView(): View = surfaceView

    override fun dispose() {
        if (plugin.activeView === this) plugin.activeView = null
        pause()
        session?.close()
        session = null
    }

    fun resume() {
        if (resumed) return
        if (!plugin.hasCameraPermission()) {
            sendStatus("stopped", "Camera permission not granted")
            return
        }
        val s = session ?: createSession() ?: return
        try {
            s.resume()
        } catch (e: CameraNotAvailableException) {
            sendStatus("stopped", "Camera not available")
            return
        }
        surfaceView.onResume()
        resumed = true
        sendStatus("initializing", null)
    }

    fun pause() {
        if (!resumed) return
        resumed = false
        // Blocks until the GL thread has left onDrawFrame, so the session is
        // never paused mid-update.
        surfaceView.onPause()
        session?.pause()
    }

    private fun createSession(): Session? {
        if (ArCoreApk.getInstance().checkAvailability(context) != ArCoreApk.Availability.SUPPORTED_INSTALLED) {
            sendStatus("stopped", "ARCore is not installed")
            return null
        }
        return try {
            Session(context).also { s ->
                val config = Config(s).apply {
                    focusMode = Config.FocusMode.AUTO
                    updateMode = Config.UpdateMode.LATEST_CAMERA_IMAGE
                    planeFindingMode = Config.PlaneFindingMode.DISABLED
                    lightEstimationMode = Config.LightEstimationMode.DISABLED
                }
                s.configure(config)
                session = s
            }
        } catch (e: UnavailableException) {
            Log.e(TAG, "Failed to create AR session", e)
            sendStatus("stopped", "AR unavailable: ${e.javaClass.simpleName}")
            null
        }
    }

    private fun displayRotation(): Int {
        val display = if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.R) {
            plugin.activity?.display
        } else {
            @Suppress("DEPRECATION")
            (context.getSystemService(Context.WINDOW_SERVICE) as WindowManager).defaultDisplay
        }
        return display?.rotation ?: Surface.ROTATION_0
    }

    private fun sendStatus(state: String, reason: String?) {
        lastState = state
        plugin.sendEvent(mapOf("state" to state, "reason" to reason))
    }

    // ---- GLSurfaceView.Renderer (GL thread) ----

    override fun onSurfaceCreated(gl: GL10?, config: EGLConfig?) {
        GLES20.glClearColor(0f, 0f, 0f, 1f)
        background.createOnGlThread()
        textureBoundTo = null
    }

    override fun onSurfaceChanged(gl: GL10?, width: Int, height: Int) {
        GLES20.glViewport(0, 0, width, height)
        viewportWidth = width
        viewportHeight = height
        viewportChanged = true
    }

    override fun onDrawFrame(gl: GL10?) {
        GLES20.glClear(GLES20.GL_COLOR_BUFFER_BIT or GLES20.GL_DEPTH_BUFFER_BIT)
        val s = session ?: return
        if (textureBoundTo !== s) {
            s.setCameraTextureName(background.textureId)
            textureBoundTo = s
        }
        if (viewportChanged) {
            s.setDisplayGeometry(displayRotation(), viewportWidth, viewportHeight)
            viewportChanged = false
        }

        val frame = try {
            s.update()
        } catch (e: SessionPausedException) {
            return
        } catch (e: CameraNotAvailableException) {
            sendStatus("stopped", "Camera not available")
            return
        }
        background.draw(frame)

        val now = SystemClock.uptimeMillis()
        val camera = frame.camera
        val (state, reason) = when (camera.trackingState) {
            TrackingState.TRACKING -> "tracking" to null
            TrackingState.PAUSED -> when (camera.trackingFailureReason) {
                TrackingFailureReason.NONE -> "initializing" to null
                TrackingFailureReason.INSUFFICIENT_LIGHT -> "limited" to "Too dark"
                TrackingFailureReason.EXCESSIVE_MOTION -> "limited" to "Moving too fast"
                TrackingFailureReason.INSUFFICIENT_FEATURES -> "limited" to "Not enough texture"
                TrackingFailureReason.CAMERA_UNAVAILABLE -> "limited" to "Camera unavailable"
                else -> "limited" to "Tracking lost"
            }
            else -> "stopped" to "Tracking stopped"
        }
        val tsUs = epochMicros(frame.timestamp)
        if (plugin.wantsFrames && now - lastFrameSent >= plugin.frameIntervalMs) {
            if (sendCameraImage(frame, state, tsUs)) lastFrameSent = now
        }

        if (state == lastState && now - lastPoseSent < POSE_INTERVAL_MS) return
        lastPoseSent = now
        lastState = state

        val event = mutableMapOf<String, Any?>("state" to state, "reason" to reason, "ts" to tsUs.toDouble())
        if (state == "tracking") {
            // Physical camera pose in the gravity-aligned ARCore world frame.
            val pose = camera.pose
            event["t"] = listOf(pose.tx().toDouble(), pose.ty().toDouble(), pose.tz().toDouble())
            event["q"] = listOf(
                pose.qx().toDouble(), pose.qy().toDouble(), pose.qz().toDouble(), pose.qw().toDouble(),
            )
        }
        plugin.sendEvent(event)
    }

    /**
     * ARCore frame timestamps share a clock with the camera, which is either
     * `elapsedRealtimeNanos` (boot time, the IMU clock) or `nanoTime`
     * (uptime). Pick whichever is closer and map it to the epoch like
     * sensors_plus does.
     */
    private fun epochMicros(frameNs: Long): Long {
        val offset = clockOffsetUs ?: run {
            val realtime = SystemClock.elapsedRealtimeNanos()
            val mono = System.nanoTime()
            val base = if (Math.abs(frameNs - realtime) <= Math.abs(frameNs - mono)) realtime else mono
            (System.currentTimeMillis() * 1000 - base / 1000).also { clockOffsetUs = it }
        }
        return offset + frameNs / 1000
    }

    /** Copies the luma plane of the CPU image and sends it with its pose. */
    private fun sendCameraImage(frame: Frame, state: String, tsUs: Long): Boolean {
        val image = try {
            frame.acquireCameraImage()
        } catch (e: NotYetAvailableException) {
            return false
        } catch (e: IllegalStateException) {
            return false
        }
        image.use {
            val w = it.width
            val h = it.height
            val plane = it.planes[0]
            val buffer = plane.buffer
            val rowStride = plane.rowStride
            val bytes = ByteArray(w * h)
            if (rowStride == w && plane.pixelStride == 1) {
                buffer.get(bytes, 0, w * h)
            } else {
                for (row in 0 until h) {
                    buffer.position(row * rowStride)
                    if (plane.pixelStride == 1) {
                        buffer.get(bytes, row * w, w)
                    } else {
                        for (col in 0 until w) bytes[row * w + col] = buffer.get(row * rowStride + col * plane.pixelStride)
                    }
                }
            }
            // Intrinsics of the CPU image; rescale if ARCore reports them
            // for a different resolution.
            val intrinsics = frame.camera.imageIntrinsics
            val dims = intrinsics.imageDimensions
            val sx = if (dims[0] > 0) w.toDouble() / dims[0] else 1.0
            val sy = if (dims[1] > 0) h.toDouble() / dims[1] else 1.0
            val f = intrinsics.focalLength
            val c = intrinsics.principalPoint
            val event = mutableMapOf<String, Any?>(
                "ts" to tsUs.toDouble(),
                "w" to w,
                "h" to h,
                "y" to bytes,
                "k" to listOf(f[0] * sx, f[1] * sy, (c[0] + 0.5) * sx - 0.5, (c[1] + 0.5) * sy - 0.5),
                "state" to state,
            )
            if (state == "tracking") {
                val pose = frame.camera.pose
                event["t"] = listOf(pose.tx().toDouble(), pose.ty().toDouble(), pose.tz().toDouble())
                event["q"] = listOf(
                    pose.qx().toDouble(), pose.qy().toDouble(), pose.qz().toDouble(), pose.qw().toDouble(),
                )
            }
            plugin.sendFrame(event)
        }
        return true
    }
}
