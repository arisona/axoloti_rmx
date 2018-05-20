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
#include "rmx_axo_filter.hpp"

namespace rmx {

class AutoPan final {
public:
    AutoPan()   {
    }

    void process(const int32buffer inBufL, const int32buffer inBufR,
                 int32buffer outBufL, int32buffer outBufR,
                 int rate, int amount) {
        amount = lp.process(amount);

        int sineL;
        SINE2TINTERP(phase, sineL);
        sineL = ___SMMUL(sineL, sineL);
        int sineR;
        SINE2TINTERP(phase + (INT_MAX >> 1) - 1, sineR);	
        sineR = ___SMMUL(sineR, sineR);

        int freq;
        MTOFEXTENDED(rate, freq)
        phase += freq >> 10;

        int src = ONE - amount << 2;
        int panL = ___SMMUL(sineL, amount << 2);
        int panR = ___SMMUL(sineR, amount << 2);
        for (int s = 0; s < BUFSIZE; ++s) {
            int l = inBufL[s] << 3;
            int r = inBufR[s] << 3;
            outBufL[s] = ___SMMUL(l, src) + ___SMMUL(l, panL << 2);
            outBufR[s] = ___SMMUL(r, src) + ___SMMUL(r, panR << 2);
        }
    }

private:
    unsigned int phase = 0;

    LP lp { 0 };
};

} // namespace rmx