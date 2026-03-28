/*
  Copyright (C) 2006-2011 Nasca Octavian Paul
  Author: Nasca Octavian Paul
  Author/Copyright (C) 2017 Xenakios

  This program is free software; you can redistribute it and/or modify
  it under the terms of version 2 of the GNU General Public License
  as published by the Free Software Foundation.

  Stripped version for Mangle integration — JUCE dependencies removed, pffft only.
*/

#pragma once

#include "globals.h"

#ifndef PS_USE_VDSP_FFT
#define PS_USE_VDSP_FFT 0
#endif

#ifndef PS_USE_PFFFT
#define PS_USE_PFFFT 0
#endif

#if PS_USE_VDSP_FFT
#elif PS_USE_PFFFT
#include "pffft/pffft.h"
#else
#include "fftw3.h"
#endif

#include <random>
#include <type_traits>


template<typename T>
class FFTWBuffer
{
public:
    FFTWBuffer()
    {
        static_assert(std::is_floating_point<T>::value,"FFTWBuffer only works with floating point types");
    }
    ~FFTWBuffer()
    {
        freeimpl(m_buf);
    }
    void resize(int size, bool clear)
    {
        jassert(size>0);
        if (size==m_size && clear==false)
            return;
        if (m_buf)
            freeimpl(m_buf);
        mallocimpl(m_buf,size);

        if (clear)
            for (int i=0;i<size;++i)
                m_buf[i]=T();
        m_size = size;
    }

    T& operator[](int index)
    {
        jassert(index >= 0 && index < m_size);
        return m_buf[index];
    }
    const T& operator[](int index) const
    {
        jassert(index >= 0 && index < m_size);
        return m_buf[index];
    }

    T* data()
    {
        jassert(m_buf!=nullptr);
        return m_buf;
    }
    int getSize() { return m_size; }
    FFTWBuffer(FFTWBuffer&& other) : m_buf(other.m_buf), m_size(other.m_size)
    {
        other.m_buf = nullptr;
        other.m_size = 0;
    }
    FFTWBuffer& operator = (FFTWBuffer&& other)
    {
        std::swap(other.m_buf, m_buf);
        std::swap(other.m_size, m_size);
        return *this;
    }
    FFTWBuffer(const FFTWBuffer&) = delete;
    FFTWBuffer& operator = (const FFTWBuffer&) = delete;
private:
    T* m_buf = nullptr;
    int m_size = 0;
    void mallocimpl(T*& buf,int size)
    {
#if PS_USE_VDSP_FFT
        buf = (T*)malloc(size*sizeof(T));
#elif PS_USE_PFFFT
        buf = (T*)pffft_aligned_malloc(size*sizeof(T));
#else
        if constexpr (std::is_same<T,float>::value)
            buf = (float*)fftwf_malloc(size*sizeof(float));
        else
            buf = (double*)fftw_malloc(size * sizeof(double));
#endif
    }
    void freeimpl(T*& buf)
    {
        if (buf!=nullptr)
        {
#if PS_USE_VDSP_FFT
            free(buf);
#elif PS_USE_PFFFT
            pffft_aligned_free(buf);
#else
            if constexpr (std::is_same<T,float>::value)
                fftwf_free(buf);
            else
                fftw_free(buf);
#endif
            buf = nullptr;
        }
    }
};

enum FFTWindow{W_RECTANGULAR,W_HAMMING,W_HANN,W_BLACKMAN,W_BLACKMAN_HARRIS};

class FFT
{//FFT class that considers phases as random
    public:
        FFT(int nsamples_, bool no_inverse=false);
        ~FFT();
        void smp2freq();
        void freq2smp();
        void applywindow(FFTWindow type);
        std::vector<REALTYPE> smp;
        std::vector<REALTYPE> freq;

        int nsamples=0;

    private:

#if PS_USE_VDSP_FFT
        void * planfft;
        int log2N;
        FFTWBuffer<REALTYPE> m_workReal;
        FFTWBuffer<REALTYPE> m_workImag;
#elif PS_USE_PFFFT
        PFFFT_Setup *planpffft = nullptr;
        FFTWBuffer<REALTYPE> m_work;
#else
        fftwf_plan planfftw,planifftw;
#endif
        FFTWBuffer<REALTYPE> data;

        struct{
            std::vector<REALTYPE> data;
            FFTWindow type;
        }window;

        std::mt19937 m_randgen;
        std::uniform_int_distribution<unsigned int> m_randdist{0,32767};
};

class Stretch
{
    public:
        Stretch(REALTYPE rap_,int in_bufsize_,FFTWindow w=W_HAMMING,bool bypass_=false,REALTYPE samplerate_=44100,int stereo_mode_=0);
        virtual ~Stretch();

        int get_max_bufsize(){
            return bufsize*3;
        };
        int get_bufsize(){
            return bufsize;
        };
        virtual void setBufferSize(int sz);
        REALTYPE get_onset_detection_sensitivity(){
            return onset_detection_sensitivity;
        };

        REALTYPE process(REALTYPE *smps,int nsmps);
        void set_freezing(bool new_freezing){
            freezing=new_freezing;
        };
        bool isFreezing() { return freezing; }

        std::vector<REALTYPE> out_buf;

        int get_nsamples(REALTYPE current_pos_percents);
        int get_nsamples_for_fill();
        int get_skip_nsamples();

        void set_rap(REALTYPE newrap);

        void set_onset_detection_sensitivity(REALTYPE detection_sensitivity);;
        void here_is_onset(REALTYPE onset);
        virtual void setSampleRate(REALTYPE sr) { samplerate = jlimit(1000.0f, 384000.0f, sr); }
        REALTYPE getSampleRate() { return samplerate; }
        FFTWindow window_type;
    protected:
        int bufsize=0;

        virtual void process_spectrum(REALTYPE *){};
        virtual REALTYPE get_stretch_multiplier(REALTYPE pos_percents);
        REALTYPE samplerate=0.0f;

    private:

        void do_analyse_inbuf(REALTYPE *smps);
        void do_next_inbuf_smps(REALTYPE *smps);
        REALTYPE do_detect_onset();

        REALTYPE rap,onset_detection_sensitivity;
        std::vector<REALTYPE> old_out_smps;
        std::vector<REALTYPE> old_freq;
        std::vector<REALTYPE> new_smps,old_smps,very_old_smps;

        std::unique_ptr<FFT> infft,outfft;
        std::unique_ptr<FFT> fft;
        long double remained_samples;
        long double extra_onset_time_credit;
        REALTYPE c_pos_percents;
        int skip_samples;
        bool require_new_buffer;
        bool bypass,freezing;
    JUCE_DECLARE_NON_COPYABLE_WITH_LEAK_DETECTOR(Stretch)
};
