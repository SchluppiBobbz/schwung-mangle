/*
  Copyright (C) 2009 Nasca Octavian Paul
  Author: Nasca Octavian Paul

  This program is free software; you can redistribute it and/or modify
  it under the terms of version 2 of the GNU General Public License
  as published by the Free Software Foundation.

  Stripped version for Mangle integration — FreeFilter removed.
*/

#include <math.h>
#include <stdlib.h>
#include <stdio.h>
#include "ProcessedStretch.h"

ProcessedStretch::ProcessedStretch(REALTYPE rap_,int in_bufsize_,FFTWindow w,bool bypass_,REALTYPE samplerate_,int stereo_mode_)
    : Stretch(rap_,in_bufsize_,w,bypass_,samplerate_,stereo_mode_)
{
};

ProcessedStretch::~ProcessedStretch()
{
};

void ProcessedStretch::set_parameters(ProcessParameters *ppar)
{
    pars=*ppar;
}

void ProcessedStretch::setBufferSize(int sz)
{
    jassert(sz > 0);
    Stretch::setBufferSize(sz);
    {
        nfreq = bufsize;
        m_infreq = floatvector(nfreq);
        m_sumfreq = floatvector(nfreq);
        m_tmpfreq1 = floatvector(nfreq);
        m_tmpfreq2 = floatvector(nfreq);
    }
}

void ProcessedStretch::copy(REALTYPE* freq1, REALTYPE* freq2)
{
    for (int i = 0; i<nfreq; i++) freq2[i] = freq1[i];
};

void ProcessedStretch::add(REALTYPE *freq2,REALTYPE *freq1,REALTYPE a){
    for (int i=0;i<nfreq;i++) freq2[i]+=freq1[i]*a;
};

void ProcessedStretch::mul(REALTYPE *freq1,REALTYPE a){
    for (int i=0;i<nfreq;i++) freq1[i]*=a;
};

void ProcessedStretch::zero(REALTYPE *freq1){
    for (int i=0;i<nfreq;i++) freq1[i]=0.0;
};

REALTYPE ProcessedStretch::get_stretch_multiplier(REALTYPE /*pos_percents*/){
    REALTYPE result=1.0;
    return result;
};

void ProcessedStretch::process_spectrum(REALTYPE *freq)
{
    for (auto& e : m_spectrum_processes)
    {
        spectrum_copy(nfreq, freq, m_infreq.data());
        if (e.m_index == SPT_Harmonics && e.m_enabled)
            spectrum_do_harmonics(pars, m_tmpfreq1, nfreq, samplerate, m_infreq.data(), freq);
        if (e.m_index == SPT_TonalVsNoise && e.m_enabled)
            spectrum_do_tonal_vs_noise(pars,nfreq,samplerate,m_tmpfreq1, m_infreq.data(), freq);
        if (e.m_index == SPT_FreqShift && e.m_enabled)
            spectrum_do_freq_shift(pars,nfreq,samplerate,m_infreq.data(), freq);
        if (e.m_index == SPT_PitchShift && e.m_enabled)
            spectrum_do_pitch_shift(pars,nfreq,m_infreq.data(), freq, pow(2.0f, pars.pitch_shift.cents / 1200.0f));
        if (e.m_index == SPT_RatioMix && e.m_enabled)
            spectrum_do_ratiomix(pars,nfreq,samplerate, m_sumfreq, m_tmpfreq1, m_infreq.data(), freq);
        if (e.m_index == SPT_Spread && e.m_enabled)
            spectrum_spread(nfreq,samplerate,m_tmpfreq1,m_infreq.data(), freq, pars.spread.bandwidth);
        if (e.m_index == SPT_Filter && e.m_enabled)
            spectrum_do_filter(pars,nfreq,samplerate,m_infreq.data(), freq);
        if (e.m_index == SPT_Compressor && e.m_enabled)
            spectrum_do_compressor(pars,nfreq, m_infreq.data(), freq);
    }
};
