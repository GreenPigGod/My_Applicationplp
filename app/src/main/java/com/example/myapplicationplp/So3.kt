package com.example.myapplicationplp

class So3 {

}
// --- ベクトル/行列ユーティリティ ---
internal fun f3() = FloatArray(3)
internal fun f6() = FloatArray(6)
internal fun f9() = FloatArray(9)
internal fun f36() = FloatArray(36)

internal fun v3Set(dst: FloatArray, x: Float, y: Float, z: Float) {
    dst[0]=x; dst[1]=y; dst[2]=z
}
internal fun v3Copy(dst: FloatArray, src: FloatArray) {
    dst[0]=src[0]; dst[1]=src[1]; dst[2]=src[2]
}
internal fun v3AddInplace(a: FloatArray, b: FloatArray) {
    a[0]+=b[0]; a[1]+=b[1]; a[2]+=b[2]
}
internal fun v3Sub(dst: FloatArray, a: FloatArray, b: FloatArray) {
    dst[0]=a[0]-b[0]; dst[1]=a[1]-b[1]; dst[2]=a[2]-b[2]
}
internal fun v3ScaleInplace(a: FloatArray, s: Float) {
    a[0]*=s; a[1]*=s; a[2]*=s
}
internal fun v3Dot(a: FloatArray, b: FloatArray): Float = a[0]*b[0] + a[1]*b[1] + a[2]*b[2]
internal fun v3Cross(dst: FloatArray, a: FloatArray, b: FloatArray) {
    val x = a[1]*b[2] - a[2]*b[1]
    val y = a[2]*b[0] - a[0]*b[2]
    val z = a[0]*b[1] - a[1]*b[0]
    dst[0]=x; dst[1]=y; dst[2]=z
}
internal fun v3Norm(a: FloatArray): Float {
    val x=a[0]; val y=a[1]; val z=a[2]
    return kotlin.math.sqrt(x*x + y*y + z*z)
}
internal fun v3NormalizeInplace(a: FloatArray) {
    val n = v3Norm(a)
    if (n > 1e-12f) v3ScaleInplace(a, 1.0f/n)
}

// --- 3x3 行列 (row-major) ---
internal fun m3SetIdentity(R: FloatArray) {
    for (i in 0 until 9) R[i]=0f
    R[0]=1f; R[4]=1f; R[8]=1f
}
internal fun m3Copy(dst: FloatArray, src: FloatArray) { for (i in 0 until 9) dst[i]=src[i] }
internal fun m3Mul(C: FloatArray, A: FloatArray, B: FloatArray) { // C = A*B
    val a00=A[0]; val a01=A[1]; val a02=A[2]
    val a10=A[3]; val a11=A[4]; val a12=A[5]
    val a20=A[6]; val a21=A[7]; val a22=A[8]
    val b00=B[0]; val b01=B[1]; val b02=B[2]
    val b10=B[3]; val b11=B[4]; val b12=B[5]
    val b20=B[6]; val b21=B[7]; val b22=B[8]
    C[0]=a00*b00+a01*b10+a02*b20
    C[1]=a00*b01+a01*b11+a02*b21
    C[2]=a00*b02+a01*b12+a02*b22
    C[3]=a10*b00+a11*b10+a12*b20
    C[4]=a10*b01+a11*b11+a12*b21
    C[5]=a10*b02+a11*b12+a12*b22
    C[6]=a20*b00+a21*b10+a22*b20
    C[7]=a20*b01+a21*b11+a22*b21
    C[8]=a20*b02+a21*b12+a22*b22
}
internal fun m3Transpose(C: FloatArray, A: FloatArray) {
    C[0]=A[0]; C[1]=A[3]; C[2]=A[6]
    C[3]=A[1]; C[4]=A[4]; C[5]=A[7]
    C[6]=A[2]; C[7]=A[5]; C[8]=A[8]
}
internal fun m3Vec(dst: FloatArray, M: FloatArray, v: FloatArray) {
    val x=v[0]; val y=v[1]; val z=v[2]
    dst[0]=M[0]*x+M[1]*y+M[2]*z
    dst[1]=M[3]*x+M[4]*y+M[5]*z
    dst[2]=M[6]*x+M[7]*y+M[8]*z
}

// --- SO(3) の Exp/Log ---
internal fun expSO3(R: FloatArray, w: FloatArray) {
    val th = v3Norm(w)
    if (th < 1e-12f) { m3SetIdentity(R); return }
    val a0 = w[0]/th; val a1 = w[1]/th; val a2 = w[2]/th
    val s = kotlin.math.sin(th)
    val c = kotlin.math.cos(th)
    val A = s/th
    val B = (1f - c)/(th*th)
    val K00=0f;   val K01=-a2; val K02= a1
    val K10= a2;  val K11=0f;  val K12=-a0
    val K20=-a1;  val K21= a0; val K22=0f
    val KK00= K01*K10 + K02*K20
    val KK01= K01*K11 + K02*K21
    val KK02= K01*K12 + K02*K22
    val KK10= K10*K00 + K12*K20
    val KK11= K10*K01 + K12*K21
    val KK12= K10*K02 + K12*K22
    val KK20= K20*K00 + K21*K10
    val KK21= K20*K01 + K21*K11
    val KK22= K20*K02 + K21*K12
    R[0] = 1f + A*K00 + B*KK00
    R[1] =      A*K01 + B*KK01
    R[2] =      A*K02 + B*KK02
    R[3] =      A*K10 + B*KK10
    R[4] = 1f + A*K11 + B*KK11
    R[5] =      A*K12 + B*KK12
    R[6] =      A*K20 + B*KK20
    R[7] =      A*K21 + B*KK21
    R[8] = 1f + A*K22 + B*KK22
}

internal fun logSO3(w: FloatArray, R: FloatArray) {
    val tr = ((R[0] + R[4] + R[8]) - 1f) * 0.5f
    val x = tr.coerceIn(-1f, 1f)
    val th = kotlin.math.acos(x.toDouble()).toFloat()
    if (th < 1e-12f) { v3Set(w, 0f,0f,0f); return }
    val denom = 2f * kotlin.math.sin(th)
    val wx = (R[7]-R[5]) * (th/denom)
    val wy = (R[2]-R[6]) * (th/denom)
    val wz = (R[3]-R[1]) * (th/denom)
    v3Set(w, wx, wy, wz)
}
