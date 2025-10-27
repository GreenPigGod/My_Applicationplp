//
// Created by 田中伸明 on 2025/10/26.
//

#ifndef MY_APPLICATIONPLP_PORTABLE_IMAGE_H
#define MY_APPLICATIONPLP_PORTABLE_IMAGE_H


// core/portable_image.h
#pragma once
#include <vector>
#include <cstdint>
#include <cstring>
#include <cmath>

namespace core {

// ピクセル（必要なら）
    template<class T>
    struct pixel_t {
        T r{}, g{}, b{};
        pixel_t() = default;
        pixel_t(T _r, T _g, T _b): r(_r), g(_g), b(_b) {}
    };

// 1chのfloat画像が主用途なら gray だけでもOK
    template <class T>
    class image {
    public:
        image(): w_(0), h_(0), pitch_(0) {}
        explicit image(int w, int h){ resize(w,h); }
        bool resize(int w, int h){
            if(w<=0||h<=0) return false;
            w_=w; h_=h; pitch_=w;
            buf_.assign(size_t(w)*h, T{});
            return true;
        }
        int width() const { return w_; }
        int height() const { return h_; }
        T*       at(int x,int y)       { return &buf_[size_t(y)*pitch_ + x]; }
        const T* at(int x,int y) const { return &buf_[size_t(y)*pitch_ + x]; }

        // DIB.hの cubic_at を移植（境界は折返し）
        T cubic_at(float x, float y) const {
            int cx[16], cy[16];
            cx[0]=int(x); cy[0]=int(y);
            cx[1]=cx[0]+1; cy[1]=cy[0];
            cx[2]=cx[0];   cy[2]=cy[0]+1;
            cx[3]=cx[0]+1; cy[3]=cy[0]+1;
            cx[4]=cx[0]-1; cy[4]=cy[0]-1; cx[5]=cx[0]-1; cy[5]=cy[0]+0;
            cx[6]=cx[0]-1; cy[6]=cy[0]+1; cx[7]=cx[0]-1; cy[7]=cy[0]+2;
            cx[8]=cx[0]+0; cy[8]=cy[0]-1; cx[9]=cx[0]+0; cy[9]=cy[0]+2;
            cx[10]=cx[0]+1; cy[10]=cy[0]-1; cx[11]=cx[0]+1; cy[11]=cy[0]+2;
            cx[12]=cx[0]+2; cy[12]=cy[0]-1; cx[13]=cx[0]+2; cy[13]=cy[0]+0;
            cx[14]=cx[0]+2; cy[14]=cy[0]+1; cx[15]=cx[0]+2; cy[15]=cy[0]+2;

            auto wfun = [](float d){
                d = std::fabs(d);
                if (d<=1) return 1 - 2*d*d + d*d*d;
                if (d<=2) return 4 - 8*d + 5*d*d - d*d*d;
                return 0.f;
            };
            T ret{}; // 0初期化
            for(int i=0;i<16;i++){
                float fx = wfun(cx[i]-x);
                float fy = wfun(cy[i]-y);
                int ix=cx[i], iy=cy[i];
                if(ix<0) ix=-ix; if(iy<0) iy=-iy;
                if(ix>=w_) ix = w_+w_-ix-1;
                if(iy>=h_) iy = h_+h_-iy-1;
                ret += (*at(ix,iy)) * (fx*fy);
            }
            return ret;
        }

    private:
        int w_, h_, pitch_;
        std::vector<T> buf_;
    };

    using Gray = image<float>;

} // namespace core
#endif //MY_APPLICATIONPLP_PORTABLE_IMAGE_H