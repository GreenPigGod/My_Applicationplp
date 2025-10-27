package com.example.myapplicationplp

import java.util.concurrent.atomic.AtomicReference

import kotlinx.coroutines.CoroutineScope
import kotlinx.coroutines.SupervisorJob
import kotlinx.coroutines.cancel
import kotlinx.coroutines.launch
import kotlinx.coroutines.newSingleThreadContext
import kotlinx.coroutines.channels.Channel

class PoseEstimatorWorker(
    private val estimator: PoseEstimatorMAP = PoseEstimatorMAP(maxIter = 6)
) {
    sealed interface Msg {
        data class ArCore(val Rm: FloatArray, val tm: FloatArray, val wPoseInv: FloatArray): Msg
        data class Grav(val gWorld: FloatArray, val aBody: FloatArray, val wInv: FloatArray): Msg
        data class Mag(val mWorld: FloatArray, val mBody: FloatArray, val wInv: FloatArray): Msg
        object Tick : Msg // 推定を回すトリガ
    }
    private val northW = AtomicReference(floatArrayOf(1f, 0f, 0f)) // 世界の“北”
    private val upW    = AtomicReference(floatArrayOf(0f, 0f, 1f)) // 世界の“上”（重力反対）

    private val scope = CoroutineScope(SupervisorJob() + newSingleThreadContext("pose-worker"))
    private val inbox = Channel<Msg>(capacity = Channel.CONFLATED)

    // 状態（ワーカー専有）
    private val R = FloatArray(9).also { m3SetIdentity(it) }
    private val t = FloatArray(3)

    // 最新測定（ワーカー専有）
    private var poseMeas: PoseMeas? = null
    private var gravMeas: UnitVecMeas? = null
    private var magMeas : UnitVecMeas? = null

    // 出力はロック無しで読む
    private val outR = AtomicReference(FloatArray(9).also { m3SetIdentity(it) })
    private val outT = AtomicReference(FloatArray(3))

    fun start() {
        scope.launch {
            for (m in inbox) when (m) {
                is Msg.ArCore -> {
                    poseMeas = PoseMeas(m.Rm, m.tm, m.wPoseInv)
                }
                is Msg.Grav -> {
                    gravMeas = UnitVecMeas(m.gWorld, m.aBody, m.wInv)
                    upW.set(m.gWorld.copyOf())
                }
                is Msg.Mag -> {
                    magMeas = UnitVecMeas(m.mWorld, m.mBody, m.wInv)
                    northW.set(m.mWorld.copyOf())
                }
                Msg.Tick -> {
                    val p = poseMeas ?: return@launch
                    val g = gravMeas ?: return@launch
                    val mg = magMeas  // 磁気は無くても回すのは可
                    val res = estimator.solve(
                        R0 = R, t0 = t,
                        poseMeas = listOf(p),
                        gravMeas = listOf(g),
                        magMeas  = mg?.let { listOf(it) } ?: emptyList(),
                        returnSigma = false
                    )
                    // 次回初期値として上書き（ワーカー内のみでアクセス）
                    m3Copy(R, res.R); v3Copy(t, res.t)
                    // 描画スレッド用に公開（コピー渡し）
                    outR.set(res.R.copyOf())
                    outT.set(res.t.copyOf())
                }
            }
        }
    }
    // ワーカーの public 関数として追加
    fun snapshotYawRad(): Float {
        val R = outR.get()                 // 推定済み R（world->body）
        val mW = northW.get()
        val gW = upW.get()
        return headingRad(R, mW, gW)       // 下のヘルパを呼ぶ
    }
    private fun headingRad(
        R_world2body: FloatArray,          // 3x3 row-major（推定器のR）
        mWorld: FloatArray,                 // 地磁気の世界方向
        gWorld: FloatArray                  // 重力の世界方向（上向きでOK）
    ): Float {
        // 小ユーティリティ
        fun dot(a: FloatArray, b: FloatArray) = a[0]*b[0] + a[1]*b[1] + a[2]*b[2]
        fun norm(a: FloatArray) = kotlin.math.sqrt((a[0]*a[0] + a[1]*a[1] + a[2]*a[2]).toDouble()).toFloat()
        fun normalize(v: FloatArray) { val n = norm(v); if (n > 1e-12f) { v[0]/=n; v[1]/=n; v[2]/=n } }

        // 水平面へ地磁気を射影 → 世界の“北”単位ベクトル
        val nW = floatArrayOf(
            mWorld[0] - dot(mWorld, gWorld)*gWorld[0],
            mWorld[1] - dot(mWorld, gWorld)*gWorld[1],
            mWorld[2] - dot(mWorld, gWorld)*gWorld[2]
        )
        normalize(nW)

        // 本体座標へ変換：nB = R^T * nW
        val nB_x = R_world2body[0]*nW[0] + R_world2body[3]*nW[1] + R_world2body[6]*nW[2]
        val nB_y = R_world2body[1]*nW[0] + R_world2body[4]*nW[1] + R_world2body[7]*nW[2]

        return kotlin.math.atan2(nB_y, nB_x) // ラジアン
    }


    fun stop() {
        scope.cancel()
    }

    suspend fun send(msg: Msg) { inbox.send(msg) }


    fun snapshotRotation(): FloatArray = outR.get()
    fun snapshotTranslation(): FloatArray = outT.get()
}
