package org.flightcoach.app

import android.os.Bundle
import android.webkit.WebView
import androidx.appcompat.app.AppCompatActivity

class MainActivity : AppCompatActivity() {
    companion object {
        init {
            System.loadLibrary("flightcoach_app")
        }
    }

    external fun nativeCreate(): Long
    external fun nativeDestroy(handle: Long)
    external fun nativeSelectManoeuvre(handle: Long, index: Int)
    external fun nativeRunFcscore(handle: Long)
    external fun nativeExportPlotlyHtml(handle: Long): String
    external fun nativeGetSummaryJson(handle: Long): String

    private var engineHandle: Long = 0
    private lateinit var webView: WebView

    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)
        engineHandle = nativeCreate()
        webView = WebView(this).apply {
            settings.javaScriptEnabled = true
            settings.domStorageEnabled = true
        }
        setContentView(webView)
        refreshPlot()
    }

    private fun refreshPlot() {
        if (engineHandle != 0L) {
            val html = nativeExportPlotlyHtml(engineHandle)
            webView.loadDataWithBaseURL("https://cdn.plot.ly", html, "text/html", "UTF-8", null)
        }
    }

    override fun onDestroy() {
        if (engineHandle != 0L) {
            nativeDestroy(engineHandle)
            engineHandle = 0
        }
        super.onDestroy()
    }
}
