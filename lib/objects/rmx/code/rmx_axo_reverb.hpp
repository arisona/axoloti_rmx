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

class Reverb final {
public:
    Reverb()  {
    }

    void setup(uint16_t* buf) {
        reverb.Init(buf);
    }

    void process(const int32buffer inBufL, const int32buffer inBufR,
                 int32buffer outBufL, int32buffer outBufR,
                 const int amount, const int time, const int gain, const int diffusion, const int cutoff,
                 const int modEnv = 0, const int env = 0) {

        static float left[BUFSIZE];
        static float right[BUFSIZE];

        int a = lp.process((amount == 0) ? 0 : (amount + (___SMMUL(env << 3, modEnv << 2) << 2)));

        reverb.set_amount(rmx::clamp(q27_to_float(a), 0.0f, 1.0f) * 0.5f);
        reverb.set_input_gain(rmx::clamp(q27_to_float(gain), 0.0f, 1.0f));
        reverb.set_time(rmx::clamp(q27_to_float(time), 0.0f, 1.0f) * 0.8f + 0.1f);
        reverb.set_diffusion(rmx::clamp(q27_to_float(diffusion), 0.0f, 1.0f) * 0.5f + 0.5f);
        reverb.set_lp(rmx::clamp(q27_to_float(cutoff), 0.0f, 1.0f));

        for (int i = 0; i < BUFSIZE; ++i) {
            left[i] = q27_to_float(inBufL[i]);
            right[i] = q27_to_float(inBufR[i]);
        }

        reverb.Process(left, right, BUFSIZE);

        for (int i = 0; i < BUFSIZE; ++i) {
            outBufL[i] = float_to_q27(left[i]);
            outBufR[i] = float_to_q27(right[i]);
        }
    }

private:
    elements::Reverb reverb;

    LP lp { ONE >> 1 };
};

} // namespace rmx