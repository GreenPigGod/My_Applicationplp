package com.example.myapplicationplp

import android.util.Log
import androidx.annotation.OptIn
import androidx.camera.core.ExperimentalGetImage
import androidx.camera.core.ImageProxy
import java.nio.ByteBuffer

class AnalyzerImpl : NativeAnalyzer() {
    @OptIn(ExperimentalGetImage::class)
    override fun analyze(image: ImageProxy) {
        val img = image.image ?: run { image.close(); return }
        try {
            val yPlane = img.planes[0]
            val yArr = yPlane.buffer.toByteArray()
            val avg = processYuv420(
                yArr,
                yPlane.rowStride,
                image.width,
                image.height
            )
            Log.d("NDK", "Sobel avg = $avg")
        } finally {
            image.close()
        }
    }
    private fun ByteBuffer.toByteArray(): ByteArray {
        val b = ByteArray(remaining()); get(b); rewind(); return b
    }
}
