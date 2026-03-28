/*
  PaulXStretch JUCE compatibility shim for Mangle integration.
  Replaces JUCE dependencies with standard C++ equivalents.
*/
#pragma once

#include <cassert>
#include <cstring>
#include <cmath>
#include <algorithm>
#include <vector>
#include <memory>

// Replace jassert with standard assert (no-op in NDEBUG builds)
#define jassert(x) assert(x)

// Replace jlimit with std::clamp
template<typename T>
inline T jlimit(T minval, T maxval, T val) {
    return std::clamp(val, minval, maxval);
}

// Replace JUCE FloatVectorOperations
struct FloatVectorOperations {
    static void copy(float* dst, const float* src, int n) {
        memcpy(dst, src, n * sizeof(float));
    }
    static void multiply(float* dst, float val, int n) {
        for (int i = 0; i < n; ++i) dst[i] *= val;
    }
    static void multiply(float* dst, const float* src, int n) {
        for (int i = 0; i < n; ++i) dst[i] *= src[i];
    }
    static void multiply(float* dst, const float* src, float val, int n) {
        for (int i = 0; i < n; ++i) dst[i] = src[i] * val;
    }
};

// Replace JUCE_DECLARE_NON_COPYABLE_WITH_LEAK_DETECTOR
#define JUCE_DECLARE_NON_COPYABLE_WITH_LEAK_DETECTOR(ClassName) \
    ClassName(const ClassName&) = delete; \
    ClassName& operator=(const ClassName&) = delete;
