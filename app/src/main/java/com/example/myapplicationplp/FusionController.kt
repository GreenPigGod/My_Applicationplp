package com.example.myapplicationplp

import kotlinx.coroutines.CoroutineScope
import kotlinx.coroutines.Dispatchers
import kotlinx.coroutines.launch
import android.hardware.Sensor
import android.hardware.SensorEvent
import android.hardware.SensorEventListener
import android.hardware.SensorManager



class FusionController(
    private val sensorManager: SensorManager,
    private val sessionProvider: () -> com.google.ar.core.Session
) : SensorEventListener {

    private var session: com.google.ar.core.Session? = null
    private var worker: PoseEstimatorWorker? = null

    // センサバッファ（UI/GLと分離、ここでは簡易LPF）
    private val accBody = FloatArray(3)
    private val magBody = FloatArray(3)
    private val gWorld = floatArrayOf(0f,0f,1f)
    private var mWorld = floatArrayOf(1f,0f,0f)

    // 逆共分散（適宜調整）
    private fun eye6(s: Float) = FloatArray(36).also { for (i in 0 until 6) it[i*6+i]=s }
    private fun eye3(s: Float) = FloatArray(9).also { it[0]=s; it[4]=s; it[8]=s }
    private val wPoseInv = eye6(1f)
    private val wGravInv = eye3(50f)
    private val wMagInv  = eye3(10f)

    fun onResume() {
        // 1) セッション確保
        session = sessionProvider().also { it.resume() }

        // 2) ワーカー開始
        worker = PoseEstimatorWorker().also { it.start() }

        // 3) センサ登録（Game≒~50–100Hz）
        sensorManager.getDefaultSensor(Sensor.TYPE_ACCELEROMETER)?.also {
            sensorManager.registerListener(this, it, SensorManager.SENSOR_DELAY_GAME)
        }
        sensorManager.getDefaultSensor(Sensor.TYPE_MAGNETIC_FIELD)?.also {
            sensorManager.registerListener(this, it, SensorManager.SENSOR_DELAY_GAME)
        }
    }

    fun onPause() {
        // 1) センサ解除 → 2) ワーカー停止 → 3) セッション停止 の順が安全
        sensorManager.unregisterListener(this)
        worker?.stop(); worker = null
        session?.pause(); session = null
    }

    override fun onSensorChanged(e: SensorEvent) {
        when (e.sensor.type) {
            Sensor.TYPE_ACCELEROMETER -> {
                for (i in 0..2) accBody[i] = 0.95f*accBody[i] + 0.05f*e.values[i]
                worker?.let { w ->
                    CoroutineScope(Dispatchers.Main.immediate).launch {
                        w.send(PoseEstimatorWorker.Msg.Grav(gWorld, accBody.copyOf(), wGravInv))
                    }
                }
            }
            Sensor.TYPE_MAGNETIC_FIELD -> {
                for (i in 0..2) magBody[i] = 0.9f*magBody[i] + 0.1f*e.values[i]
                worker?.let { w ->
                    CoroutineScope(Dispatchers.Main.immediate).launch {
                        w.send(PoseEstimatorWorker.Msg.Mag(mWorld, magBody.copyOf(), wMagInv))
                    }
                }
            }
        }
    }
    override fun onAccuracyChanged(sensor: Sensor?, accuracy: Int) {}

    // GridRenderer（GLスレッド）から呼ぶ
    fun onDrawFrame(renderer: (FloatArray, FloatArray) -> Unit) {
        val sess = session ?: return
        val frame = sess.update() // GLスレッド推奨
        val pose = frame.camera.displayOrientedPose
        val rm = FloatArray(9)
        run {
            val m = FloatArray(16) // column-major 4x4
            pose.toMatrix(m, 0)    // ARCore Pose → 4x4 (column-major)
            // 3x3回転だけを row-major に抽出
            rm[0] = m[0];  rm[1] = m[4];  rm[2] = m[8]
            rm[3] = m[1];  rm[4] = m[5];  rm[5] = m[9]
            rm[6] = m[2];  rm[7] = m[6];  rm[8] = m[10]
        }

        val tm = floatArrayOf(pose.tx(), pose.ty(), pose.tz())

        // 入力をワーカーへ（最新だけで良い→Conflated）
        worker?.let { w ->
            CoroutineScope(Dispatchers.Main.immediate).launch {
                w.send(PoseEstimatorWorker.Msg.ArCore(rm, tm, wPoseInv))
                w.send(PoseEstimatorWorker.Msg.Tick) // この時点の最新バンドルで一回推定
            }
        }

        // 非ブロッキングで最新推定を取得して描画
        val R = worker?.snapshotRotation() ?: rm
        val t = worker?.snapshotTranslation() ?: tm
        renderer(R, t)
    }
}
