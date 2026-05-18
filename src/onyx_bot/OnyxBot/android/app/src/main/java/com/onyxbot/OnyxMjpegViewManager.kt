package com.onyxbot

import android.webkit.WebSettings
import android.webkit.WebView
import com.facebook.react.uimanager.SimpleViewManager
import com.facebook.react.uimanager.ThemedReactContext
import com.facebook.react.uimanager.annotations.ReactProp

class OnyxMjpegViewManager : SimpleViewManager<WebView>() {
  override fun getName(): String = "OnyxMjpegView"

  override fun createViewInstance(reactContext: ThemedReactContext): WebView {
    return WebView(reactContext).apply {
      setBackgroundColor(android.graphics.Color.BLACK)
      settings.cacheMode = WebSettings.LOAD_NO_CACHE
      settings.loadWithOverviewMode = true
      settings.useWideViewPort = true
      settings.builtInZoomControls = false
      settings.displayZoomControls = false
    }
  }

  @ReactProp(name = "sourceUrl")
  fun setSourceUrl(view: WebView, sourceUrl: String?) {
    if (sourceUrl.isNullOrBlank()) {
      view.loadUrl("about:blank")
      return
    }

    val html = """
      <!doctype html>
      <html>
        <head>
          <meta name="viewport" content="width=device-width,initial-scale=1,maximum-scale=1,user-scalable=no">
          <style>
            html, body {
              margin: 0;
              width: 100%;
              height: 100%;
              background: #111827;
              overflow: hidden;
            }
            img {
              width: 100%;
              height: 100%;
              object-fit: contain;
              display: block;
            }
          </style>
        </head>
        <body>
          <img src="$sourceUrl">
        </body>
      </html>
    """.trimIndent()

    view.loadDataWithBaseURL(sourceUrl, html, "text/html", "UTF-8", null)
  }
}
