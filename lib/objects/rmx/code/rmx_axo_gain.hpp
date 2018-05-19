//
// DSP Utilities for Axoloti patches
//
// Copyright (c) Stefan Arisona aka robot_mixeur
//
// https://robotized.arisona.ch
//
// License: GPL
//

#pragma once

#include "rmx_axo_base.hpp"

namespace rmx {

template<typename Filter = PassThrough>
class Gain final {
public:
    Gain(int lpCutoff = MINUS_ONE) : filter(lpCutoff) {}

    inline void process(const int32buffer in, int32buffer out, int gain) {
        for (int i = 0; i < BUFSIZE; ++i) {
            out[i] = __SSAT(___SMMUL(in[i] << 3, filter.process(gain) << 2), 28);
        }
    }

private:
    Filter filter;
};

template<typename Filter = PassThrough>
class StereoGain final {
public:
    StereoGain(int lpCutoff = MINUS_ONE) : filter(lpCutoff) {}

    inline void process(const int32buffer in0, const int32buffer in1, int32buffer out0, int32buffer out1, int gain) {
        for (int i = 0; i < BUFSIZE; ++i) {
            int g = filter.process(gain) << 2;
            out0[i] = __SSAT(___SMMUL(in0[i] << 3, g), 28);
            out1[i] = __SSAT(___SMMUL(in1[i] << 3, g), 28);
        }
    }

private:
    Filter filter;
};

template<typename Filter = PassThrough>
class ZeroCrossingGain final {
public:
    ZeroCrossingGain(int lpCutoff = MINUS_ONE) : filter(lpCutoff) {}

    inline void process(const int32buffer in, int32buffer out, int gain) {
        for (int i = 0; i < BUFSIZE; ++i) {
            if (in[i] > 0 != this->in > 0) 
                this->gain = gain;
            this->in = in[i];
            out[i] = __SSAT(___SMMUL(in[i] << 3, filter.process(gain) << 2), 28);
        }
    }

private:
    Filter filter;
    int in = 0;
    int gain = 0;
};

template<typename Filter = PassThrough>
class LinearGain final {
public:
    LinearGain(int lpCutoff = MINUS_ONE) : filter(lpCutoff) {}

    inline void process(const int32buffer in, int32buffer out, int gain) {
        gain = filter.process(gain);
        int step = (gain - prev) >> BUFSIZEPOW;
        int curr = gain;
        prev = gain;
        for (int i = 0; i < BUFSIZE; ++i) {
            out[i] = __SSAT(___SMMUL(in[i] << 3, curr << 2), 28);
            curr += step;
        }
    }

private:
    Filter filter;
    int curr = 0;
    int prev = 0;
    int step = 0;
};

} // namespace rmx
