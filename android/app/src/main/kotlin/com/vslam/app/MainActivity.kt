package com.vslam.app

import io.flutter.embedding.android.FlutterActivity
import io.flutter.embedding.engine.FlutterEngine

class MainActivity : FlutterActivity() {
    private val arPlugin = ArPlugin()

    override fun configureFlutterEngine(flutterEngine: FlutterEngine) {
        super.configureFlutterEngine(flutterEngine)
        flutterEngine.plugins.add(arPlugin)
    }

    override fun onResume() {
        super.onResume()
        arPlugin.onHostResume()
    }

    override fun onPause() {
        // Pause the AR session before the activity gives up the camera.
        arPlugin.onHostPause()
        super.onPause()
    }
}
