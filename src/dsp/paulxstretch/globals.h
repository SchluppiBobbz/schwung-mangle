/*
  Copyright (C) 2006-2011 Nasca Octavian Paul
  Author: Nasca Octavian Paul

  This program is free software; you can redistribute it and/or modify
  it under the terms of version 2 of the GNU General Public License
  as published by the Free Software Foundation.

  Stripped version for Mangle integration — JUCE dependencies removed.
*/
#pragma once

#include <vector>
#include <memory>
#include <algorithm>
#include <cassert>
#include "ps_compat.h"

using REALTYPE = float;

using floatvector = std::vector<REALTYPE>;

#ifndef NULL
#define NULL 0
#endif

const double c_PI = 3.14159265359;

template<typename Cont, typename T>
inline void fill_container(Cont& c, const T& x)
{
    std::fill(std::begin(c), std::end(c), x);
}

template<typename T>
class CircularBuffer final
{
public:
    CircularBuffer(int size)
    {
        m_buf.resize(size);
    }
    void clear()
    {
        m_avail = 0;
        m_readpos = 0;
        m_writepos = 0;
        fill_container(m_buf, T());
    }
    void push(T x)
    {
        m_buf[m_writepos] = x;
        ++m_writepos;
        ++m_avail;
        if (m_writepos >= (int)m_buf.size())
            m_writepos = 0;
    }
    T get()
    {
        jassert(m_avail > 0);
        T x = m_buf[m_readpos];
        ++m_readpos;
        --m_avail;
        if (m_readpos >= (int)m_buf.size())
            m_readpos = 0;
        return x;
    }
    int available() { return m_avail; }
    int getToBuf(T* buf, int len)
    {
        if (len > m_avail)
            len = m_avail;
        for (int i = 0; i < len; ++i)
            buf[i] = get();
        return len;
    }
    int getFromBuf(T* buf, int len)
    {
        for (int i = 0; i < len; ++i)
            push(buf[i]);
        return len;
    }
    int getSize() { return (int)m_buf.size(); }
    void resize(int size)
    {
        m_avail = 0;
        m_readpos = 0;
        m_writepos = 0;
        m_buf.resize(size);
    }
private:
    int m_writepos = 0;
    int m_readpos = 0;
    int m_avail = 0;
    std::vector<T> m_buf;
};
