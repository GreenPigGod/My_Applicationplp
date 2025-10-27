package com.example.myapplicationplp

import androidx.camera.core.ImageAnalysis

abstract class NativeAnalyzer : ImageAnalysis.Analyzer {
    companion object { init { System.loadLibrary("imgproc") } }
    protected external fun processYuv420(
        y: ByteArray, yRowStride: Int, width: Int, height: Int
    ): Int
    //JNIを呼ぶを?
}