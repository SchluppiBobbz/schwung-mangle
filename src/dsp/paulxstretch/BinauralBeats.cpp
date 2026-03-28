/*
  Copyright (C) 2008-2011 Nasca Octavian Paul
  Author: Nasca Octavian Paul

  This program is free software; you can redistribute it and/or modify
  it under the terms of version 2 of the GNU General Public License
  as published by the Free Software Foundation.

  Stripped version for Mangle integration.
*/

#include <math.h>
#include <stdio.h>
#include "BinauralBeats.h"

const REALTYPE Hilbert::coefl[]={0.6923877778065f, 0.9360654322959f, 0.9882295226860f, 0.9987488452737f};
const REALTYPE Hilbert::coefr[]={0.4021921162426f, 0.8561710882420f, 0.9722909545651f, 0.9952884791278f};

BinauralBeats::BinauralBeats(int samplerate_){
    samplerate=samplerate_;
    hilbert_t=0.0f;
};


void BinauralBeats::process(REALTYPE *smpsl,REALTYPE *smpsr,int nsmps,REALTYPE pos_percents){
    if (pars.free_edit.get_enabled()){
        float mono=pars.mono*0.5f;
        for (int i=0;i<nsmps;i++){
            REALTYPE inl=smpsl[i];
            REALTYPE inr=smpsr[i];
            REALTYPE outl=inl*(1.0f-mono)+inr*mono;
            REALTYPE outr=inr*(1.0f-mono)+inl*mono;
            smpsl[i]=outl;
            smpsr[i]=outr;
        };

        REALTYPE freq=pars.free_edit.get_value(pos_percents);

        freq*=0.5;
        for (int i=0;i<nsmps;i++){
            hilbert_t=fmod(hilbert_t+freq/samplerate,1.0f);
            REALTYPE h1=0.0f,h2=0.0f;
            hl.process(smpsl[i],h1,h2);

            REALTYPE x=hilbert_t*2.0f*(float)M_PI;
            REALTYPE m1=h1*cos(x);
            REALTYPE m2=h2*sin(x);
            REALTYPE outl1=m1+m2;
            REALTYPE outl2=m1-m2;

            h1=0;
            h2=0;
            hr.process(smpsr[i],h1,h2);

            m1=h1*cos(x);
            m2=h2*sin(x);
            REALTYPE outr1=m1-m2;
            REALTYPE outr2=m1+m2;

            switch(pars.stereo_mode){
                case SM_LEFT_RIGHT:
                    smpsl[i]=outl2;
                    smpsr[i]=outr2;
                    break;
                case SM_RIGHT_LEFT:
                    smpsl[i]=outl1;
                    smpsr[i]=outr1;
                    break;
                case SM_SYMMETRIC:
                    smpsl[i]=(outl1+outr1)*0.5f;
                    smpsr[i]=(outl2+outr2)*0.5f;
                    break;
            };
        };
    };
};
