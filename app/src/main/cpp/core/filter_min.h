//
// Created by 田中伸明 on 2025/10/26.
//

#ifndef MY_APPLICATIONPLP_FILTER_MIN_H
#define MY_APPLICATIONPLP_FILTER_MIN_H

#endif //MY_APPLICATIONPLP_FILTER_MIN_H
// core/filter_min.h
#pragma once
#include "portable_image.h"
#include <vector>
#include <algorithm>
#include <cmath>

namespace core {

    class Filter {
    public:
        using Gray = core::Gray;

        void mean3(const Gray& in, Gray& out);
        void mask_filter(const Gray& in, Gray& out, float* mask, int n);
        void edge_amp_dir(const Gray& dx, const Gray& dy, Gray& amp, Gray& dir);
        void zero_crossing(const Gray& in, const Gray& dir, Gray& out); // 名前は小文字化

        // 必要に応じて増やす（reduce_depth / threshold 等）
    };

} // namespace core
namespace core {
    struct Point { float x, y; };
    using Points    = std::vector<Point>;
    using Contours  = std::vector<Points>;

// 追加：RGBAに描く版
    void draw_contours_rgba(
            uint8_t* rgba, int stride, int w, int h,
            int dx, int dy, const Contours& conts, uint32_t colorABGR);
}
