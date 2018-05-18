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

namespace rmx {

static const int BUFSIZEPOW = 4;

static const int ONE = (1 << 27) - 1;
static const int MIN = -(1 << 27);

static const int CONTROLRATE = SAMPLERATE >> BUFSIZEPOW;


inline int clamp(int value, int min, int max) {
    return std::max(min, std::min(max, value));
}

inline float clamp(float value, float min, float max) {
    return std::max(min ,std::min(max, value));
}

inline int mod(int a, int b) {
    int r = a % b;
    return r < 0 ? r + b : r;
}


template<typename T, int length>
class MovingAverage final {
public:
    MovingAverage() {
        for (int i = 0; i < length; ++i)
            buffer[i] = T{};
    }

    ~MovingAverage() {
    }

    T process(T value) {
        sum -= buffer[pos];
        sum += value;
        buffer[pos] = value;
        pos = pos < length - 1 ? pos + 1 : 0;
        return sum / length;
    }

private:
    T buffer[length];
    T sum = T{};
    int pos = 0;
};



namespace detail {

class BiquadBase {
    // from Axoloti axoloti_filters.h
public:
    BiquadBase() {}

    inline void process(const int32buffer in, int32buffer out) {
        for (int i = 0; i < BUFSIZE; ++i) {
            out[i] = process(in[i]);
        }
    }

    inline int process(int input) {
        int accu = ___SMMUL(cxn_0, input);
        accu = ___SMMLA(cxn_1, filter_x_n1, accu);
        accu = ___SMMLA(cxn_2, filter_x_n2, accu);
        accu = ___SMMLS(cyn_1, filter_y_n1, accu);
        accu = ___SMMLS(cyn_2, filter_y_n2, accu);
        int output = accu << 4;
        filter_x_n2 = filter_x_n1;
        filter_x_n1 = input;
        filter_y_n2 = filter_y_n1;
        filter_y_n1 = output;
        return __SSAT(output, 28);
    }

protected:
    bool doSetup(int cutoff, int reso, int& cosW0, int& alpha, int& a0_inv_q31, int& q_inv) {
        if (cutoff == this->cutoff && reso == this->reso)
            return false;
        this->cutoff = cutoff;
        this->reso = reso;

        int filter_W0;
        MTOF(cutoff, filter_W0);
        q_inv = INT_MAX - (__USAT(reso, 27) << 4);

        filter_W0 = filter_W0 >> 1;
        int sinW0 = arm_sin_q31(filter_W0);
        cosW0 = arm_cos_q31(filter_W0);
        alpha = ___SMMUL(sinW0, q_inv);
        float filter_a0 = (HALFQ31 + alpha);
        float filter_a0_inv = ((INT32_MAX >> 2) / filter_a0);
        a0_inv_q31 = (int)(INT32_MAX * filter_a0_inv);

        return true;
    }


    int cutoff = -1;
    int reso = -1;

    int cxn_0 = 0;
    int cxn_1 = 0;
    int cxn_2 = 0;
    int cyn_1 = 0;
    int cyn_2 = 0;

    int filter_x_n1 = 0;
    int filter_x_n2 = 0;
    int filter_y_n1 = 0;
    int filter_y_n2 = 0;
};

} // namespace detail 

class BiquadLP final : public detail::BiquadBase {
public:
    BiquadLP(int cutoff = 0, int reso = 0) {
        setup(cutoff, reso);
    }

    void setup(int cutoff, int reso) {
        int cosW0;
        int alpha;
        int a0_inv_q31;
        int q_inv;
        if (!doSetup(cutoff, reso, cosW0, alpha, a0_inv_q31, q_inv))
            return;

        cyn_1 = ___SMMUL((-cosW0), a0_inv_q31);
        cyn_2 = ___SMMUL((HALFQ31 - alpha), a0_inv_q31);
        cxn_0 = ___SMMUL(___SMMUL(HALFQ31 - (cosW0 >> 1), a0_inv_q31), q_inv);
        cxn_1 = cxn_0 << 1;
        cxn_2 = cxn_0;        
    }
};

class BiquadBP final : public detail::BiquadBase {
public:
    BiquadBP(int cutoff = 0, int reso = 0) {
        setup(cutoff, reso);
    }

    void setup(int cutoff, int reso) {
        int cosW0;
        int alpha;
        int a0_inv_q31;
        int q_inv;
        if (!doSetup(cutoff, reso, cosW0, alpha, a0_inv_q31, q_inv))
            return;

        cyn_1 = ___SMMUL((-cosW0), a0_inv_q31);
        cyn_2 = ___SMMUL((HALFQ31 - alpha), a0_inv_q31);
        cxn_0 = ___SMMUL(alpha, a0_inv_q31);
        cxn_1 = 0;
        cxn_2 = -cxn_0;
    }
};

class BiquadHP final : public detail::BiquadBase {
public:
    BiquadHP(int cutoff = 0, int reso = 0) {
        setup(cutoff, reso);
    }

    void setup(int cutoff, int reso) {
        int cosW0;
        int alpha;
        int a0_inv_q31;
        int q_inv;
        if (!doSetup(cutoff, reso, cosW0, alpha, a0_inv_q31, q_inv))
            return;

        cyn_1 = ___SMMUL((-cosW0), a0_inv_q31);
        cyn_2 = ___SMMUL((HALFQ31 - alpha), a0_inv_q31);
        cxn_0 = ___SMMUL(___SMMUL(HALFQ31 + (cosW0 >> 1), a0_inv_q31), q_inv);
        cxn_1 = -(cxn_0 << 1);
        cxn_2 = cxn_0;
    }
};



namespace detail {

class SVFBase {
    // from Axoloti SVF
    // in addition, see: http://www.musicdsp.org/showArchiveComment.php?ArchiveID=92
public:
    SVFBase(int cutoff, int reso) {
        setup(cutoff, reso);
    }

    void setup(int cutoff, int reso) {
        int alpha = 0;
        MTOFEXTENDED(cutoff, alpha);
        SINE2TINTERP(alpha, freq);		

        damp = INT_MAX - (reso << 3) - (reso << 2);
    }

protected:
    inline void processInternal(int sample) {
        int notch = sample - (___SMMUL(damp, band) << 1);
        low = low + (___SMMUL(freq, band) << 1);
        high = notch - low;
        band = (___SMMUL(freq, high) << 1) + band;
    }

    int freq = 0;
    int damp = 0;

    int low = 0;
    int high = 0;
    int band = 0;
};

} // namespace detail

class SVFLP final : public detail::SVFBase {
public:
    SVFLP(int cutoff = 0, int reso = 0) : SVFBase(cutoff, reso) {}

    inline int process(int sample) {
        processInternal(sample);
        return low;
    }

    inline void process(const int32buffer in, int32buffer out) {
        for (int i = 0; i < BUFSIZE; ++i) {
            out[i] = process(in[i]);
        }
    }
};

class SVFBP final : public detail::SVFBase {
public:
    SVFBP(int cutoff = 0, int reso = 0) : SVFBase(cutoff, reso) {}

    inline int process(int sample) {
        processInternal(sample);
        return band;
    }

    inline void process(const int32buffer in, int32buffer out) {
        for (int i = 0; i < BUFSIZE; ++i) {
            out[i] = process(in[i]);
        }
    }    
};

class SVFHP final : public detail::SVFBase {
public:
    SVFHP(int cutoff = 0, int reso = 0) : SVFBase(cutoff, reso) {}

    inline int process(int sample) {
        processInternal(sample);
        return high;
    }

    inline void process(const int32buffer in, int32buffer out) {
        for (int i = 0; i < BUFSIZE; ++i) {
            out[i] = process(in[i]);
        }
    }
};



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

    int process(int attack, int release, const int32buffer input0, const int32buffer input1) {
        update(attack, release);
        return process(input0, input1);
    }

    int process(const int32buffer input0, const int32buffer input1) {
        int level = 0;
        for (int i = 0; i < BUFSIZE; ++i) {
            int s;
            s = input0[i] >> (BUFSIZEPOW + 1);
            level += s > 0 ? s : -s;
            s = input1[i] >> (BUFSIZEPOW + 1);
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

    SVFLP lp { ONE >> 1, 0 };

    int env = 0;
    int envFiltered = 0;
};



class EnvelopeFollowerSimple final {
    // from Axoloti Follower object
public:
    EnvelopeFollowerSimple() {
    }

    int process(const int32buffer input0, const int32buffer input1) {
        int level = 0;
        for (int i = 0; i < BUFSIZE; ++i) {
            int s;
            s = input0[i] >> 1;
            level += s > 0 ? s : -s;
            s = input1[i] >> 1;
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
