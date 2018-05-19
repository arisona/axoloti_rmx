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

class EnvelopeFollower final {	
public:
    EnvelopeFollower(int attack = 0, int release = 0) {
        update(attack, release);
    }

    void update(int attack, int release) {
        if (attack != this->attack) {
            this->attack = attack;
            a = time2gain(attack);
        }

        if (release != this->release) {
            this->release = release;
            r = time2gain(release);
        }
    }

    int process(int attack, int release, const int32buffer in0, const int32buffer in1) {
        update(attack, release);
        return process(in0, in1);
    }

    int process(const int32buffer in0, const int32buffer in1) {
        int level = 0;
        for (int i = 0; i < BUFSIZE; ++i) {
            int s;
            s = in0[i] >> (BUFSIZEPOW + 1);
            level += s > 0 ? s : -s;
            s = in1[i] >> (BUFSIZEPOW + 1);
            level += s > 0 ? s : -s;
        }

        if (level > env) {
            env = ___SMMUL(a << 2, env - level << 3) + level;
        } else if (level < env) {
            env = ___SMMUL(r << 2, env - level << 3) + level;
        }
        return __SSAT(lp.process(env), 28);
    }

    inline int get() const {
        return envFiltered;
    }

private:
    static inline int time2gain(int t) {
        static const float EXP_T[64] = {
            0.001f, 0.001155f, 0.001334f, 0.00154f, 0.001778f, 0.002054f, 0.002371f, 0.002738f,
            0.003162f, 0.003652f, 0.004217f, 0.00487f, 0.005623f, 0.006494f, 0.007499f, 0.00866f, 
            0.01f, 0.011548f, 0.013335f, 0.015399f, 0.017783f, 0.020535f, 0.023714f, 0.027384f, 
            0.031623f, 0.036517f, 0.04217f, 0.048697f, 0.056234f, 0.064938f, 0.074989f, 0.086596f, 
            0.1f, 0.115478f, 0.133352f, 0.153993f, 0.177828f, 0.205353f, 0.237137f, 0.273842f,
            0.316228f, 0.365174f, 0.421697f, 0.486968f, 0.562341f, 0.649382f, 0.749894f, 0.865964f, 
            1.0f, 1.154782f, 1.333521f, 1.539927f, 1.778279f, 2.053525f, 2.371374f, 2.73842f,
            3.162278f, 3.651741f, 4.216965f, 4.869675f, 5.623413f, 6.493816f, 7.498942f, 8.659643f, 
        };

        t = (t >> (27 - 6)) & 0x3f;
        return float_to_q27((float)pow(0.1, 1.0 / (EXP_T[t] * CONTROLRATE)));
    }

    int attack = -1;
    int release = -1;

    int a = 0;
    int r = 0;

    LP lp { ONE - (ONE >> 2) };

    int env = 0;
    int envFiltered = 0;
};



class EnvelopeFollowerSimple final {
    // from Axoloti Follower object
public:
    EnvelopeFollowerSimple() {
    }

    int process(const int32buffer in0, const int32buffer in1) {
        int level = 0;
        for (int i = 0; i < BUFSIZE; ++i) {
            int s;
            s = in0[i] >> 1;
            level += s > 0 ? s : -s;
            s = in1[i] >> 1;
            level += s > 0 ? s : -s;
        }
        envelope -= envelope >> slope;
        envelope += level >> (slope + 4);
        return envelope;
    }

    inline int get() const {
        return envelope;
    }

private:
    // 2=1.3ms 3=2.7ms 4=5.3ms 5=10.6ms 6=21.2ms 7=42.6ms 8=85.3ms 9=170.7ms
    int slope = 4;
    int envelope = 0;
};

} // namespace rmx
