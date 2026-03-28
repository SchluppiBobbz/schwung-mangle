/*
  Copyright (C) 2011 Nasca Octavian Paul
  Author: Nasca Octavian Paul

  This program is free software; you can redistribute it and/or modify
  it under the terms of version 2 of the GNU General Public License
  as published by the Free Software Foundation.

  Stripped version for Mangle integration.
*/
#pragma once
#define HAVE_PS_FREE_EDIT
#ifdef HAVE_PS_FREE_EDIT
#define FREE_EDIT_MAX_POINTS 50
#include <math.h>
#include <stdio.h>

#include "globals.h"

#define LOG_2 0.693147181
#define LOG_10 2.302585093

#define dB2rap(dB) ((exp((dB)*LOG_10/20.0)))
#define rap2dB(rap) ((20*log(rap)/LOG_10))

struct FreeEditPos{
    REALTYPE x,y;
    bool enabled;
};
enum FREE_EDIT_EXTREME_SCALE{
    FE_LINEAR=0,
    FE_LOG=1,
    FE_DB=2
};
class FreeEditExtremes{
    public:
        FreeEditExtremes(){
            init();
        };
        void init(REALTYPE min_=0.0,REALTYPE max_=1.0,FREE_EDIT_EXTREME_SCALE scale_=FE_LINEAR,bool lock_min_max_=false,bool lock_scale_=false){
            min=min_;
            max=max_;
            scale=scale_;
            lock_min_max=lock_min_max_;
            lock_scale=lock_scale_;
            correct_values();
        };

        inline REALTYPE coord_to_real_value(REALTYPE coord){
            REALTYPE result;
            switch(scale){
                case FE_LOG:
                    result=exp(log(min)+coord*(log(max)-log(min)));
                    return result;
                default:
                    result=min+coord*(max-min);
                    return result;
            };
        };

        inline REALTYPE real_value_to_coord(REALTYPE val){
            switch(scale){
                case FE_LOG:
                    {
                        REALTYPE coord=log(val/min)/log(max/min);
                        clamp1(coord);
                        return coord;
                    };
                default:
                    {
                        REALTYPE diff=max-min;
                        REALTYPE coord;
                        if (fabs(diff)>0.0001) coord=(val-min)/diff;
                        else coord=min;
                        clamp1(coord);
                        return coord;
                    };
            };
        };

        void set_min(REALTYPE val){
            if (lock_min_max) return;
            min=val;
            correct_values();
        };
        REALTYPE get_min(){
            return min;
        };
        void set_max(REALTYPE val){
            if (lock_min_max) return;
            max=val;
            correct_values();
        };
        REALTYPE get_max(){
            return max;
        };
        FREE_EDIT_EXTREME_SCALE get_scale(){
            return scale;
        };
        void set_scale(FREE_EDIT_EXTREME_SCALE val){
            if (lock_scale) return;
            scale=val;
        };
    private:
        inline REALTYPE clamp1(REALTYPE m){
            if (m<0.0) return 0.0;
            if (m>1.0) return 1.0;
            return m;
        };
        void correct_values(){
            if (scale!=FE_LOG) return;
            if (min<1e-8) min=1e-8f;
            if (max<1e-8) max=1e-8f;
        };
        bool lock_min_max,lock_scale;
        REALTYPE min,max;
        FREE_EDIT_EXTREME_SCALE scale;
};
class FreeEdit{
    public:
        enum INTERP_MODE{
            LINEAR=0,
            COSINE=1
        };
        FreeEdit();
        FreeEdit (const FreeEdit &other);
        const FreeEdit &operator=(const FreeEdit &other);
        void deep_copy_from(const FreeEdit &other);

        bool get_enabled() const{
            return enabled;
        };
        void set_enabled(bool val){
            enabled=val;
        };

        inline int get_npoints(){
            return npos;
        };

        inline bool is_enabled(int n) const{
            if ((n<0)||(n>=npos)) return false;
            return pos[n].enabled;
        };
        inline void set_enabled(int n,bool enabled_){
            if ((n<0)||(n>=npos)) return;
            pos[n].enabled=enabled_;
        };

        inline REALTYPE get_posx(int n) const{
            if ((n<0)||(n>=npos)) return 0.0;
            return pos[n].x;
        };
        inline REALTYPE get_posy(int n) const{
            if ((n<0)||(n>=npos)) return 0.0;
            return pos[n].y;
        };
        inline void set_posx(int n,REALTYPE x){
            if ((n<2)||(n>=npos)) return;
            pos[n].x=clamp1(x);
        };
        inline void set_posy(int n,REALTYPE y){
            if ((n<0)||(n>=npos)) return;
            pos[n].y=clamp1(y);
        };

        void set_all_values(REALTYPE val){
            for (int i=0;i<npos;i++){
                if (pos[i].enabled) pos[i].y=extreme_y.real_value_to_coord(val);
            }
        };

        INTERP_MODE get_interp_mode() const{
            return interp_mode;
        };
        void set_interp_mode(INTERP_MODE interp_mode_){
            interp_mode=interp_mode_;
        };

        REALTYPE get_smooth() const{
            return smooth;
        };
        void set_smooth(REALTYPE smooth_){
            smooth=clamp1(smooth_);;
        };

        void get_curve(int datasize,REALTYPE *data,bool real_values);

        ~FreeEdit(){
            delete []pos;
        };

        void update_curve(int size=16384);
        REALTYPE get_value(REALTYPE x);

        FreeEditExtremes extreme_x,extreme_y;

        struct{
            REALTYPE *data;
            int size;
            int allocsize;
        }curve;
    private:
        inline REALTYPE clamp1(REALTYPE m){
            if (m<0.0) return 0.0;
            if (m>1.0) return 1.0;
            return m;
        };
        inline void swap(REALTYPE &m1,REALTYPE &m2){
            REALTYPE tmp=m1;
            m1=m2;
            m2=tmp;
        };
        FreeEditPos *pos;
        int npos;
        REALTYPE smooth;
        INTERP_MODE interp_mode;
        bool enabled;
};

#endif
