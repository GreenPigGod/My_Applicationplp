//
// Created by 田中伸明 on 2025/10/26.
//
// core/filter_min.cpp
#include "filter_min.h"

namespace core {

    void Filter::mean3(const Gray& in, Gray& out){
        int w=in.width(), h=in.height();
        out.resize(w,h);
        for(int y=1;y<h-1;y++){
            for(int x=1;x<w-1;x++){
                int sum=0;
                for(int n=-1;n<=1;n++)
                    for(int m=-1;m<=1;m++)
                        sum += *in.at(x+m,y+n);
                *out.at(x,y) = sum/9.0f;
            }
        }
    }

    void Filter::mask_filter(const Gray& in, Gray& out, float* mask, int siz_n){
        int w=in.width(), h=in.height(); out.resize(w,h);
        int siz = int(std::floor((siz_n-1)/2.0));
        for(int y=0;y<h;y++){
            for(int x=0;x<w;x++){
                float sum=0; int n=0;
                for(int ny=-siz;ny<=siz;ny++){
                    for(int nx=-siz;nx<=siz;nx++,n++){
                        int sx=x+nx, sy=y+ny;
                        if(sx<0) sx=-sx; if(sx>=w) sx=w-1-(sx-w);
                        if(sy<0) sy=-sy; if(sy>=h) sy=h-1-(sy-h);
                        sum += (*in.at(sx,sy)) * mask[n];
                    }
                }
                *out.at(x,y)=sum;
            }
        }
    }

    void Filter::edge_amp_dir(const Gray& dx, const Gray& dy, Gray& amp, Gray& dir){
        int w=dx.width(), h=dx.height(); amp.resize(w,h); dir.resize(w,h);
        for(int y=0;y<h;y++){
            for(int x=0;x<w;x++){
                float px=*dx.at(x,y), py=*dy.at(x,y);
                *amp.at(x,y)=std::sqrt(px*px+py*py);
                *dir.at(x,y)=std::atan2(py,px);
            }
        }
    }

    void Filter::zero_crossing(const Gray& in, const Gray& dir, Gray& out){
        int w=in.width(), h=in.height(); out.resize(w,h);
        static const int nx[8]={1,1,0,-1,-1,-1,0,1};
        static const int ny[8]={0,1,1,1,0,-1,-1,-1};
        for(int y=0;y<h;y++){
            for(int x=0;x<w;x++){
                float p1=*in.at(x,y);
                float theta=*dir.at(x,y);
                if(theta<0) theta += float(2*M_PI);
                int n;
                if(theta <  M_PI/8 || theta > 2*M_PI - M_PI/8) n=0;
                else if(theta < 3*M_PI/8) n=1;
                else if(theta < 5*M_PI/8) n=2;
                else if(theta < 7*M_PI/8) n=3;
                else if(theta < 9*M_PI/8) n=4;
                else if(theta <11*M_PI/8) n=5;
                else if(theta <13*M_PI/8) n=6;
                else n=7;
                float p2=*in.at(x+nx[n], y+ny[n]);
                *out.at(x,y) = (p1*p2<0) ? 255.f : 0.f;
            }
        }

    }

} // namespace core


namespace core {

    static inline void pset_rgba(uint8_t* base, int stride, int w, int h, int x, int y, uint32_t abgr){
        if ((unsigned)x >= (unsigned)w || (unsigned)y >= (unsigned)h) return;
        uint8_t* p = base + y*stride + x*4;
        // ABGR = [A B G R]
        p[0] = (abgr >> 16) & 0xFF; // R   ←必要ならここをARGBなどに合わせて入替
        p[1] = (abgr >> 8)  & 0xFF; // G
        p[2] = (abgr      ) & 0xFF; // B
        p[3] = (abgr >> 24) & 0xFF; // A
    }

    static inline void draw_rect3x3(uint8_t* img,int stride,int w,int h,int x,int y,uint32_t c){
        for(int yy=-1; yy<=1; ++yy)
            for(int xx=-1; xx<=1; ++xx)
                pset_rgba(img,stride,w,h,x+xx,y+yy,c);
    }

    static inline void line_rgba(uint8_t* img,int stride,int w,int h,
                                 int x0,int y0,int x1,int y1,uint32_t c){
        // Bresenham
        int dx = std::abs(x1-x0), sx = x0<x1 ? 1 : -1;
        int dy = -std::abs(y1-y0), sy = y0<y1 ? 1 : -1;
        int err = dx + dy;
        for(;;){
            pset_rgba(img,stride,w,h,x0,y0,c);
            if (x0==x1 && y0==y1) break;
            int e2 = 2*err;
            if (e2 >= dy){ err += dy; x0 += sx; }
            if (e2 <= dx){ err += dx; y0 += sy; }
        }
    }

    void draw_contours_rgba(uint8_t* rgba, int stride, int w, int h,
                            int dx, int dy, const Contours& conts, uint32_t colorABGR)
    {
        for (const auto& poly : conts){
            if (poly.size()<2) continue;
            // ポリライン描画
            for(size_t i=0;i+1<poly.size();++i){
                int x0 = dx + (int)std::lround(poly[i].x);
                int y0 = dy + (int)std::lround(poly[i].y);
                int x1 = dx + (int)std::lround(poly[i+1].x);
                int y1 = dy + (int)std::lround(poly[i+1].y);
                line_rgba(rgba, stride, w, h, x0, y0, x1, y1, colorABGR);
            }
            // 始点/終点のマーカー（元実装の FillSolidRect 相当）
            int bx = dx + (int)std::lround(poly.front().x);
            int by = dy + (int)std::lround(poly.front().y);
            int ex = dx + (int)std::lround(poly.back().x);
            int ey = dy + (int)std::lround(poly.back().y);
            // 反転色にしたい場合は colorABGR を 0xFFFFFFFF ^ colorABGR などで
            draw_rect3x3(rgba, stride, w, h, bx, by, colorABGR);
            draw_rect3x3(rgba, stride, w, h, ex, ey, colorABGR);
        }
    }

} // namespace core
