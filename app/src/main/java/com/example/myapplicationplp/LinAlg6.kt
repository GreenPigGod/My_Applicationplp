package com.example.myapplicationplp

class LinAlg6 {

}

internal fun zero36(A: FloatArray) { for (i in 0 until 36) A[i]=0f }
internal fun zero6(v: FloatArray) { for (i in 0 until 6) v[i]=0f }

// H += S,  b += S*(-r)  （J=I の SE3ポーズ因子）
internal fun accumulatePoseIdentity(
    H: FloatArray, b: FloatArray,
    r6: FloatArray, S6: FloatArray
) {
    for (i in 0 until 36) H[i] += S6[i]
    for (i in 0 until 6) {
        var s = 0f
        for (j in 0 until 6) s += S6[i*6 + j] * (-r6[j])
        b[i] += s
    }
}

// 回転のみ（3×6 J=[Jw|0]）: H(0..2,0..2)+=Jw^T S Jw,  b(0..2)+=Jw^T S (-r)
internal fun accumulateRotBlock(
    H: FloatArray, b: FloatArray,
    Jw: FloatArray,           // 3x3
    r3: FloatArray,           // 3
    S3: FloatArray            // 3x3 inverse covariance
) {
    val JT_S = FloatArray(9)
    for (i in 0 until 3) for (k in 0 until 3) {
        var s = 0f
        for (j in 0 until 3) s += Jw[j*3+i] * S3[j*3+k]
        JT_S[i*3 + k] = s
    }
    val JT_S_J = FloatArray(9)
    for (i in 0 until 3) for (k in 0 until 3) {
        var s = 0f
        for (j in 0 until 3) s += JT_S[i*3 + j] * Jw[j*3 + k]
        JT_S_J[i*3 + k] = s
    }
    for (i in 0 until 3) for (j in 0 until 3) {
        H[i*6 + j] += JT_S_J[i*3 + j]
    }
    val q = FloatArray(3)
    for (i in 0 until 3) {
        var s = 0f
        for (j in 0 until 3) s += S3[i*3 + j] * (-r3[j])
        q[i] = s
    }
    for (i in 0 until 3) {
        var s = 0f
        for (j in 0 until 3) s += Jw[j*3 + i] * q[j]
        b[i] += s
    }
}

// Cholesky(6x6 SPD)
internal fun cholesky6(A: FloatArray, L: FloatArray): Boolean {
    for (i in 0 until 36) L[i]=0f
    for (i in 0 until 6) {
        for (j in 0..i) {
            var sum = A[i*6 + j]
            for (k in 0 until j) sum -= L[i*6 + k] * L[j*6 + k]
            if (i == j) {
                if (sum <= 1e-12f) return false
                L[i*6 + j] = kotlin.math.sqrt(sum)
            } else {
                L[i*6 + j] = sum / L[j*6 + j]
            }
        }
    }
    return true
}

internal fun cholSolve6(L: FloatArray, b: FloatArray, x: FloatArray) {
    val y = FloatArray(6)
    for (i in 0 until 6) {
        var sum = b[i]
        for (k in 0 until i) sum -= L[i*6 + k]*y[k]
        y[i] = sum / L[i*6 + i]
    }
    for (i in 5 downTo 0) {
        var sum = y[i]
        for (k in i+1 until 6) sum -= L[k*6 + i]*x[k]
        x[i] = sum / L[i*6 + i]
    }
}

internal fun cholInvert6(L: FloatArray, invA: FloatArray) {
    val ei = FloatArray(6)
    val col = FloatArray(6)
    for (i in 0 until 36) invA[i]=0f
    for (i in 0 until 6) {
        for (k in 0 until 6) ei[k]=0f
        ei[i]=1f
        cholSolve6(L, ei, col)
        for (r in 0 until 6) invA[r*6 + i] = col[r]
    }
}

// 右更新：R <- R * Exp(δθ), t += δρ
internal fun rightUpdateSE3(R: FloatArray, t: FloatArray, dx: FloatArray) {
    val w = f3(); w[0]=dx[0]; w[1]=dx[1]; w[2]=dx[2]
    val dR = f9(); expSO3(dR, w)
    val Rnew = f9(); m3Mul(Rnew, R, dR)
    m3Copy(R, Rnew)
    t[0]+=dx[3]; t[1]+=dx[4]; t[2]+=dx[5]
}
