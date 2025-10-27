package com.example.myapplicationplp

data class PoseMeas(
    val R: FloatArray,          // 3x3
    val t: FloatArray,          // 3
    val sigmaInv6: FloatArray   // 6x6 inverse covariance
)
data class UnitVecMeas(
    val vWorldUnit: FloatArray, // 世界側の単位ベクトル（重力 or 地磁気）
    val vBody: FloatArray,      // 端末で観測したベクトル（内部で正規化）
    val sigmaInv3: FloatArray   // 3x3 inverse covariance
)
data class MapResult(
    val R: FloatArray,  // 3x3
    val t: FloatArray,  // 3
    val Sigma6: FloatArray // 6x6 近似共分散（H^{-1}）
)

private fun residualPoseSE3(r6: FloatArray, R_est: FloatArray, t_est: FloatArray, Rm: FloatArray, tm: FloatArray) {
    val Rt = f9(); m3Transpose(Rt, Rm)
    val Rrel = f9(); m3Mul(Rrel, Rt, R_est)
    val w = f3(); logSO3(w, Rrel)
    r6[0]=w[0]; r6[1]=w[1]; r6[2]=w[2]
    r6[3]=t_est[0]-tm[0]; r6[4]=t_est[1]-tm[1]; r6[5]=t_est[2]-tm[2]
}

private fun residualUnitVecAndJac(
    r3: FloatArray, Jw3x3: FloatArray,
    R_est: FloatArray, vWorld: FloatArray, vBody: FloatArray
) {
    val Rt = f9(); m3Transpose(Rt, R_est)
    val aPred = f3(); m3Vec(aPred, Rt, vWorld); v3NormalizeInplace(aPred)
    val aMeas = f3(); v3Copy(aMeas, vBody); v3NormalizeInplace(aMeas)
    v3Cross(r3, aPred, aMeas)
    val s = v3Dot(aPred, aMeas)
    Jw3x3[0]=s; Jw3x3[1]=0f; Jw3x3[2]=0f
    Jw3x3[3]=0f; Jw3x3[4]=s; Jw3x3[5]=0f
    Jw3x3[6]=0f; Jw3x3[7]=0f; Jw3x3[8]=s
    Jw3x3[0]-=aMeas[0]*aPred[0]; Jw3x3[1]-=aMeas[0]*aPred[1]; Jw3x3[2]-=aMeas[0]*aPred[2]
    Jw3x3[3]-=aMeas[1]*aPred[0]; Jw3x3[4]-=aMeas[1]*aPred[1]; Jw3x3[5]-=aMeas[1]*aPred[2]
    Jw3x3[6]-=aMeas[2]*aPred[0]; Jw3x3[7]-=aMeas[2]*aPred[1]; Jw3x3[8]-=aMeas[2]*aPred[2]
}

class PoseEstimatorMAP(
    private val maxIter: Int = 10,
    private val epsRot: Float = 1e-6f,
    private val epsTrans: Float = 1e-6f
) {
    private val H = f36()
    private val b = f6()
    private val L = f36()
    private val r6 = f6()
    private val r3 = f3()
    private val Jw = f9()

    fun solve(
        R0: FloatArray,
        t0: FloatArray,
        poseMeas: List<PoseMeas>,
        gravMeas: List<UnitVecMeas>,
        magMeas: List<UnitVecMeas>,
        returnSigma: Boolean = true
    ): MapResult {
        val R = f9(); m3Copy(R, R0)
        val t = f3(); v3Copy(t, t0)

        repeat(maxIter) {
            zero36(H); zero6(b)

            for (z in poseMeas) {
                residualPoseSE3(r6, R, t, z.R, z.t)
                accumulatePoseIdentity(H, b, r6, z.sigmaInv6)
            }
            for (z in gravMeas) {
                residualUnitVecAndJac(r3, Jw, R, z.vWorldUnit, z.vBody)
                accumulateRotBlock(H, b, Jw, r3, z.sigmaInv3)
            }
            for (z in magMeas) {
                residualUnitVecAndJac(r3, Jw, R, z.vWorldUnit, z.vBody)
                accumulateRotBlock(H, b, Jw, r3, z.sigmaInv3)
            }

            val ok = cholesky6(H, L)
            if (!ok) {
                for (i in 0 until 6) H[i*6 + i] += 1e-6f
                cholesky6(H, L)
            }
            val dx = f6()
            cholSolve6(L, b, dx)

            rightUpdateSE3(R, t, dx)

            val nr = kotlin.math.sqrt(dx[0]*dx[0]+dx[1]*dx[1]+dx[2]*dx[2])
            val nt = kotlin.math.sqrt(dx[3]*dx[3]+dx[4]*dx[4]+dx[5]*dx[5])
            if (nr < epsRot && nt < epsTrans) {
                val Sigma = if (returnSigma) {
                    val S = f36(); cholInvert6(L, S); S
                } else FloatArray(0)
                return MapResult(R, t, Sigma)
            }
        }

        val Sigma = if (returnSigma && cholesky6(H, L)) {
            val S = f36(); cholInvert6(L, S); S
        } else FloatArray(0)

        return MapResult(R, t, Sigma)
    }
}
