package com.onyxbot

import android.content.Context
import android.graphics.Bitmap
import android.graphics.BitmapFactory
import android.graphics.Canvas
import android.graphics.Color
import android.graphics.Paint
import android.graphics.RectF
import android.view.View
import com.facebook.react.uimanager.SimpleViewManager
import com.facebook.react.uimanager.ThemedReactContext
import com.facebook.react.uimanager.annotations.ReactProp
import java.io.BufferedInputStream
import java.io.ByteArrayOutputStream
import java.net.HttpURLConnection
import java.net.URL
import kotlin.math.min

private val JPEG_START = byteArrayOf(0xff.toByte(), 0xd8.toByte())
private val JPEG_END = byteArrayOf(0xff.toByte(), 0xd9.toByte())

class OnyxMjpegView(context: Context) : View(context) {
  @Volatile private var running = false
  @Volatile private var currentBitmap: Bitmap? = null
  private var worker: Thread? = null
  private val paint = Paint(Paint.ANTI_ALIAS_FLAG or Paint.FILTER_BITMAP_FLAG)

  fun setSourceUrl(sourceUrl: String?) {
    stopStream()

    if (sourceUrl.isNullOrBlank()) {
      currentBitmap = null
      invalidate()
      return
    }

    running = true
    worker = Thread { readStream(sourceUrl) }.apply {
      name = "OnyxMjpegStream"
      isDaemon = true
      start()
    }
  }

  private fun readStream(sourceUrl: String) {
    while (running) {
      var connection: HttpURLConnection? = null

      try {
        connection = URL(sourceUrl).openConnection() as HttpURLConnection
        connection.connectTimeout = 3000
        connection.readTimeout = 3000
        connection.useCaches = false
        connection.connect()

        val input = BufferedInputStream(connection.inputStream, 64 * 1024)
        val pending = ByteArrayOutputStream()
        val chunk = ByteArray(16 * 1024)

        while (running) {
          val read = input.read(chunk)
          if (read < 0) break

          pending.write(chunk, 0, read)
          extractFrames(pending)
        }
      } catch (_: Exception) {
        Thread.sleep(500)
      } finally {
        connection?.disconnect()
      }
    }
  }

  private fun extractFrames(pending: ByteArrayOutputStream) {
    var bytes = pending.toByteArray()

    while (true) {
      val start = indexOf(bytes, JPEG_START, 0)
      if (start < 0) {
        pending.reset()
        if (bytes.isNotEmpty()) {
          pending.write(bytes, bytes.size - min(bytes.size, 1), min(bytes.size, 1))
        }
        return
      }

      val end = indexOf(bytes, JPEG_END, start + JPEG_START.size)
      if (end < 0) {
        pending.reset()
        pending.write(bytes, start, bytes.size - start)
        return
      }

      val frameEnd = end + JPEG_END.size
      val bitmap = BitmapFactory.decodeByteArray(bytes, start, frameEnd - start)
      if (bitmap != null) {
        currentBitmap = bitmap
        postInvalidateOnAnimation()
      }

      bytes = bytes.copyOfRange(frameEnd, bytes.size)
      pending.reset()
      pending.write(bytes)
    }
  }

  override fun onDraw(canvas: Canvas) {
    super.onDraw(canvas)
    canvas.drawColor(Color.rgb(17, 24, 39))

    val bitmap = currentBitmap ?: return
    val viewRatio = width.toFloat() / height.toFloat()
    val bitmapRatio = bitmap.width.toFloat() / bitmap.height.toFloat()

    val dest = if (bitmapRatio > viewRatio) {
      val drawHeight = width / bitmapRatio
      val top = (height - drawHeight) / 2f
      RectF(0f, top, width.toFloat(), top + drawHeight)
    } else {
      val drawWidth = height * bitmapRatio
      val left = (width - drawWidth) / 2f
      RectF(left, 0f, left + drawWidth, height.toFloat())
    }

    canvas.drawBitmap(bitmap, null, dest, paint)
  }

  override fun onDetachedFromWindow() {
    stopStream()
    super.onDetachedFromWindow()
  }

  private fun stopStream() {
    running = false
    worker?.interrupt()
    worker = null
  }

  private fun indexOf(bytes: ByteArray, pattern: ByteArray, fromIndex: Int): Int {
    if (pattern.isEmpty() || bytes.size < pattern.size) return -1

    for (i in fromIndex..bytes.size - pattern.size) {
      var matched = true
      for (j in pattern.indices) {
        if (bytes[i + j] != pattern[j]) {
          matched = false
          break
        }
      }
      if (matched) return i
    }

    return -1
  }
}

class OnyxMjpegViewManager : SimpleViewManager<OnyxMjpegView>() {
  override fun getName(): String = "OnyxMjpegView"

  override fun createViewInstance(reactContext: ThemedReactContext): OnyxMjpegView {
    return OnyxMjpegView(reactContext)
  }

  @ReactProp(name = "sourceUrl")
  fun setSourceUrl(view: OnyxMjpegView, sourceUrl: String?) {
    view.setSourceUrl(sourceUrl)
  }
}
