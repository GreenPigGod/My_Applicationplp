//
// Created by 田中伸明 on 2025/10/24.
//
// jni/native_analyzer_jni.cpp
#include <jni.h>
#include <vector>
#include <android/log.h>
#include "core/portable_image.h"
#include "core/filter_min.h"

using core::Gray;
using core::Filter;

extern "C" JNIEXPORT jint JNICALL
Java_com_example_myapplicationplp_NativeAnalyzer_processYuv420(
        JNIEnv* env, jobject /*thiz*/,
        jbyteArray yArr, jint yRowStride, jint w, jint h) {

    if(!yArr || w<=0 || h<=0) return -1;
    jbyte* yptr = env->GetByteArrayElements(yArr, nullptr);

    Gray src(w,h), amp, dir, zc;
    // Y(8bit) → float
    for(int y=0; y<h; ++y){
        const uint8_t* row = reinterpret_cast<uint8_t*>(yptr) + y*yRowStride;
        for(int x=0; x<w; ++x) *src.at(x,y) = float(row[x]);
    }

    // カメラからグレースケール画像を送る
    // dx, dy を作って Filter::edge_amp_dir → zero_crossing
    Filter f;
    f.edge_amp_dir(dx, dy, amp, dir);
    f.zero_crossing(amp, dir, zc);

    env->ReleaseByteArrayElements(yArr, yptr, JNI_ABORT);
    return 0;
}
