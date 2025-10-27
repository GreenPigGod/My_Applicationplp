package com.example.myapplicationplp
import kotlin.math.*

import android.os.Bundle
import android.view.WindowManager
import androidx.activity.ComponentActivity
import androidx.activity.compose.setContent

import androidx.core.view.WindowCompat
import androidx.core.view.WindowInsetsCompat
import androidx.core.view.WindowInsetsControllerCompat
// Compose 側
import androidx.compose.foundation.Canvas
import androidx.compose.foundation.background

import androidx.compose.foundation.layout.fillMaxSize

import androidx.compose.runtime.Composable
import androidx.compose.runtime.mutableStateOf
import androidx.compose.runtime.remember

import androidx.compose.ui.Modifier

import androidx.compose.ui.geometry.Offset

import androidx.compose.ui.graphics.Color

import androidx.compose.runtime.getValue
import androidx.compose.runtime.setValue

import androidx.compose.ui.graphics.drawscope.DrawScope

import android.Manifest
import android.content.pm.PackageManager
import androidx.activity.result.contract.ActivityResultContracts
import androidx.compose.runtime.*
import androidx.core.content.ContextCompat


class SpotlightActivity : ComponentActivity() {
    private val askCamera = registerForActivityResult(
        ActivityResultContracts.RequestPermission()
    ) { granted ->
        if (granted) startArSession() else finish()
    }
    private var session: com.google.ar.core.Session? = null
    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)

        // 画面を常時点灯＆最大輝度に
        window.addFlags(WindowManager.LayoutParams.FLAG_KEEP_SCREEN_ON)
        window.attributes = window.attributes.apply {
            screenBrightness = 1.0f
        }

        // システムバーを隠す（戻すときはonStop等で解除）
        WindowCompat.setDecorFitsSystemWindows(window, false)
        WindowInsetsControllerCompat(window, window.decorView).apply {
            hide(WindowInsetsCompat.Type.systemBars())
            systemBarsBehavior =
                WindowInsetsControllerCompat.BEHAVIOR_SHOW_TRANSIENT_BARS_BY_SWIPE
        }
        setContent {
            var intr by remember { mutableStateOf(Intrinsics(1f,1f,0f,0f)) }
            var Rcw by remember { mutableStateOf(FloatArray(9) { if (it%4==0) 1f else 0f }) }
            var tcw by remember { mutableStateOf(floatArrayOf(0f,0f,0f)) }

            val worker = PoseEstimatorWorker()
            val yawEstRad = worker.snapshotYawRad()

            // フレームごとに更新（擬似コード：あなたの描画タイミングで呼ぶ）
            LaunchedEffect(Unit) {
                while (true) {
                    val s = this@SpotlightActivity.session
                    if(s != null){
                        val triple = fetchPoseAndIntrinsicsCorrected(s, yawEstRad)
                        if (triple != null) {
                            val (RcwCorr, tcwCorr, K) = triple
                            Rcw = RcwCorr; tcw = tcwCorr; intr = K
                        }
                    }
                    kotlinx.coroutines.delay(16)
                }
            }

            ProjectedGrid(
                intr = intr,
                Rcw = Rcw,
                tcw = tcw,
                gridStepMeters = 0.5f,
                extentMeters = 12f,
                strokePx = 2f
            )
        }

        if (ContextCompat.checkSelfPermission(this, Manifest.permission.CAMERA)
            == PackageManager.PERMISSION_GRANTED) {
            startArSession()
        } else {
            askCamera.launch(Manifest.permission.CAMERA)
        }
    }
    private fun startArSession() {
        try {
            val status = com.google.ar.core.ArCoreApk.getInstance()
                .requestInstall(this, /*userRequestedInstall=*/true)
            if (status == com.google.ar.core.ArCoreApk.InstallStatus.INSTALLED) {
                if (session == null) session = com.google.ar.core.Session(this)
            }
        } catch (e: Exception) {
            // 非対応端末などの例外ハンドリング
            e.printStackTrace()
        }
    }

    private fun decomposeRwc_ZYX(Rwc: FloatArray): Triple<Float,Float,Float> {
        val r00 = Rwc[0]; val r01 = Rwc[1]; val r02 = Rwc[2]
        val r10 = Rwc[3]; val r11 = Rwc[4]; val r12 = Rwc[5]
        val r20 = Rwc[6]; val r21 = Rwc[7]; val r22 = Rwc[8]
        val pitch = asin(-r20)
        val roll  = atan2(r21, r22)
        val yaw   = atan2(r10, r00)
        return Triple(yaw, pitch, roll)
    }

    private fun composeRwc_ZYX(yaw: Float, pitch: Float, roll: Float): FloatArray {
        val cy = cos(yaw);   val sy = sin(yaw)
        val cp = cos(pitch); val sp = sin(pitch)
        val cr = cos(roll);  val sr = sin(roll)
        // Rz * Ry * Rx
        return floatArrayOf(
            cy*cp,              cy*sp*sr - sy*cr,   cy*sp*cr + sy*sr,
            sy*cp,              sy*sp*sr + cy*cr,   sy*sp*cr - cy*sr,
            -sp,                cp*sr,              cp*cr
        )
    }

    // --- メイン：yaw を MAP で置換して Rcw を作る ---
    fun fetchPoseAndIntrinsicsCorrected(
        session: com.google.ar.core.Session,
        yawEstRad: Float,                         // MAP 推定の「北に対する yaw」
    ): Triple<FloatArray, FloatArray, Intrinsics>? {
        val frame = session.update()
        val cam = frame.camera
        if (cam.trackingState != com.google.ar.core.TrackingState.TRACKING) return null

        val intr = cam.imageIntrinsics
        val K = Intrinsics(
            fx = intr.focalLength[0],
            fy = intr.focalLength[1],
            cx = intr.principalPoint[0],
            cy = intr.principalPoint[1]
        )

        val poseCW = cam.pose.inverse()
        val Rcw_ar = FloatArray(9)
        val tcw = FloatArray(3)
        run {
            val m = FloatArray(16)
            poseCW.toMatrix(m, 0)
            // 4x4  → 3x3
            Rcw_ar[0]=m[0]; Rcw_ar[1]=m[4]; Rcw_ar[2]=m[8]
            Rcw_ar[3]=m[1]; Rcw_ar[4]=m[5]; Rcw_ar[5]=m[9]
            Rcw_ar[6]=m[2]; Rcw_ar[7]=m[6]; Rcw_ar[8]=m[10]
            tcw[0]=m[12]; tcw[1]=m[13]; tcw[2]=m[14]
        }

        // roll/pitch は ARCore、yaw は 推定器 へ差し替え
        // 1) Rwc_ar = (Rcw_ar)^T を ZYX 分解
        val Rwc_ar = floatArrayOf(
            Rcw_ar[0], Rcw_ar[3], Rcw_ar[6],
            Rcw_ar[1], Rcw_ar[4], Rcw_ar[7],
            Rcw_ar[2], Rcw_ar[5], Rcw_ar[8]
        )
        val (_, pitch_ar, roll_ar) = decomposeRwc_ZYX(Rwc_ar)

        // 2) Rwc_corr = Z(yawEst) Y(pitch_ar) X(roll_ar)
        val Rwc_corr = composeRwc_ZYX(yawEstRad, pitch_ar, roll_ar)

        // 3) Rcw_corr = (Rwc_corr)^T
        val Rcw_corr = floatArrayOf(
            Rwc_corr[0], Rwc_corr[3], Rwc_corr[6],
            Rwc_corr[1], Rwc_corr[4], Rwc_corr[7],
            Rwc_corr[2], Rwc_corr[5], Rwc_corr[8]
        )
        return Triple(Rcw_corr, tcw, K)
    }
@Composable
fun ProjectedGrid(
    intr: Intrinsics,
    Rcw: FloatArray,     // 3x3 world->camera (row-major)
    tcw: FloatArray,     // 3x1
    gridStepMeters: Float = 0.5f, // 世界平面上の格子間隔 [m]
    extentMeters: Float = 10f,    // 表示半径（±）
    strokePx: Float = 2f
) {
    Canvas(
        modifier = Modifier
            .fillMaxSize()
            .background(Color.Black)
    ) {
        val H = buildHomography(intr, Rcw, tcw)

        // X=const ライン群（北南方向の直線）
        drawGridFamily(H, fixIsX = true, step = gridStepMeters, extent = extentMeters, strokePx = strokePx, color = Color(1f,1f,1f,0.15f))

        // Y=const ライン群（東西方向の直線）
        drawGridFamily(H, fixIsX = false, step = gridStepMeters, extent = extentMeters, strokePx = strokePx, color = Color(1f,1f,1f,0.15f))

        // 原点十字（世界の原点：任意）
        drawGridAxis(H, axisLen = 1.0f)
    }
}

private fun DrawScope.drawGridFamily(
    H: FloatArray,
    fixIsX: Boolean,
    step: Float,
    extent: Float,
    strokePx: Float,
    color: Color
) {
    // 直線を画面に描くには、十分広い区間の2点を投影して結ぶ
    val tMin = -extent
    val tMax = +extent
    var c = (-extent / step).toInt() * step
    while (c <= extent + 1e-6f) {
        val p0 = if (fixIsX) projectXY(H, c, tMin) else projectXY(H, tMin, c)
        val p1 = if (fixIsX) projectXY(H, c, tMax) else projectXY(H, tMax, c)
        if (p0 != null && p1 != null) {
            drawLine(
                color = color,
                start = Offset(p0.first, p0.second),
                end   = Offset(p1.first, p1.second),
                strokeWidth = strokePx
            )
        }
        c += step
    }
}
private fun buildHomography(
    K: Intrinsics,
    Rcw: FloatArray,  // 3x3 row-major (world->camera)
    tcw: FloatArray   // 3x1
): FloatArray {       // 3x3 row-major
    // r1 = Rcw col0, r2 = col1
    val r11 = Rcw[0]; val r21 = Rcw[3]; val r31 = Rcw[6]
    val r12 = Rcw[1]; val r22 = Rcw[4]; val r32 = Rcw[7]
    val t1  = tcw[0]; val t2  = tcw[1]; val t3  = tcw[2]

    // B = [r1 r2 t]
    val b00=r11; val b01=r12; val b02=t1
    val b10=r21; val b11=r22; val b12=t2
    val b20=r31; val b21=r32; val b22=t3

    // H = K * B
    val fx=K.fx; val fy=K.fy; val cx=K.cx; val cy=K.cy
    val H = FloatArray(9)
    H[0] = fx*b00 + cx*b20; H[1] = fx*b01 + cx*b21; H[2] = fx*b02 + cx*b22
    H[3] = fy*b10 + cy*b20; H[4] = fy*b11 + cy*b21; H[5] = fy*b12 + cy*b22
    H[6] =      b20;        H[7] =      b21;        H[8] =      b22
    return H
}

// u = H [X,Y,1]^T  →  画面座標 (px, py)
private fun projectXY(H: FloatArray, X: Float, Y: Float): Pair<Float,Float>? {
    val u0 = H[0]*X + H[1]*Y + H[2]
    val u1 = H[3]*X + H[4]*Y + H[5]
    val u2 = H[6]*X + H[7]*Y + H[8]
    if (abs(u2) < 1e-6f) return null
    return Pair(u0/u2, u1/u2)
}
private fun DrawScope.drawGridAxis(H: FloatArray, axisLen: Float) {
    // X軸（北：赤）
    val x0 = projectXY(H, 0f, 0f)
    val x1 = projectXY(H, axisLen, 0f)
    if (x0 != null && x1 != null) {
        drawLine(Color.Red, Offset(x0.first, x0.second), Offset(x1.first, x1.second), strokeWidth = 3f)
    }
    // Y軸（東：緑）
    val y1 = projectXY(H, 0f, axisLen)
    if (x0 != null && y1 != null) {
        drawLine(Color.Green, Offset(x0.first, x0.second), Offset(y1.first, y1.second), strokeWidth = 3f)
    }
}
}
