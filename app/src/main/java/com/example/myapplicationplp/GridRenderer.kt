package com.example.myapplicationplp

import android.graphics.Canvas
import android.graphics.Color
import android.graphics.Paint
import kotlin.math.abs
data class Intrinsics(val fx: Float, val fy: Float, val cx: Float, val cy: Float)

class GridRenderer(
    var gridStepMeters: Float = 0.5f,   // 世界平面上の格子ピッチ[m]
    var extentMeters: Float = 12f,      // 描画範囲（±）
    var lineAlpha: Int = 38             // 0..255 （~15%）
) {
    private val H = FloatArray(9)       // ホモグラフィ K [r1 r2 t]
    private val p = Paint().apply {
        color = Color.WHITE
        strokeWidth = 2f
        isAntiAlias = true
    }
    private val pAxisX = Paint().apply { color = Color.RED; strokeWidth = 3f; isAntiAlias = true }
    private val pAxisY = Paint().apply { color = Color.GREEN; strokeWidth = 3f; isAntiAlias = true }

    /** ARCore更新後に毎フレーム呼ぶ：World->Camera 姿勢と内部パラメータをセット */
    fun updatePoseAndIntrinsics(K: Intrinsics, Rcw: FloatArray, tcw: FloatArray) {
        buildHomography(K, Rcw, tcw, H)
    }

    /** 真っ黒画面でも OK：Canvas に北基準の直交格子を投影描画 */
    fun draw(canvas: Canvas) {
        val w = canvas.width.toFloat()
        val h = canvas.height.toFloat()

        canvas.drawColor(Color.BLACK)

        p.alpha = lineAlpha
        // X = const（北南方向）
        drawFamily(canvas, fixIsX = true, step = gridStepMeters)
        // Y = const（東西方向）
        drawFamily(canvas, fixIsX = false, step = gridStepMeters)

        // 原点十字（任意。1m）
        projectXY(H, 0f, 0f)?.let { (x0,y0) ->
            projectXY(H, 1f, 0f)?.let { (x1,y1) -> canvas.drawLine(x0,y0,x1,y1,pAxisX) }
            projectXY(H, 0f, 1f)?.let { (x2,y2) -> canvas.drawLine(x0,y0,x2,y2,pAxisY) }
        }
    }

    // --- 内部処理 ---
    private fun drawFamily(canvas: Canvas, fixIsX: Boolean, step: Float) {
        val e = extentMeters
        var c = (-(e / step).toInt()) * step
        while (c <= e + 1e-6f) {
            val p0 = if (fixIsX) projectXY(H, c, -e) else projectXY(H, -e, c)
            val p1 = if (fixIsX) projectXY(H, c, +e) else projectXY(H, +e, c)
            if (p0 != null && p1 != null) canvas.drawLine(p0.first, p0.second, p1.first, p1.second, p)
            c += step
        }
    }

    private fun buildHomography(K: Intrinsics, Rcw: FloatArray, tcw: FloatArray, outH: FloatArray) {
        // Rcw: row-major。r1=col0, r2=col1
        val r11=Rcw[0]; val r21=Rcw[3]; val r31=Rcw[6]
        val r12=Rcw[1]; val r22=Rcw[4]; val r32=Rcw[7]
        val t1=tcw[0];  val t2=tcw[1];  val t3=tcw[2]
        // B = [r1 r2 t]
        val b00=r11; val b01=r12; val b02=t1
        val b10=r21; val b11=r22; val b12=t2
        val b20=r31; val b21=r32; val b22=t3
        // H = K * B
        val fx=K.fx; val fy=K.fy; val cx=K.cx; val cy=K.cy
        outH[0] = fx*b00 + cx*b20; outH[1] = fx*b01 + cx*b21; outH[2] = fx*b02 + cx*b22
        outH[3] = fy*b10 + cy*b20; outH[4] = fy*b11 + cy*b21; outH[5] = fy*b12 + cy*b22
        outH[6] =      b20;        outH[7] =      b21;        outH[8] =      b22
    }

    private fun projectXY(H: FloatArray, X: Float, Y: Float): Pair<Float,Float>? {
        val u0 = H[0]*X + H[1]*Y + H[2]
        val u1 = H[3]*X + H[4]*Y + H[5]
        val u2 = H[6]*X + H[7]*Y + H[8]
        if (abs(u2) < 1e-6f) return null
        return Pair(u0/u2, u1/u2)
    }
}
