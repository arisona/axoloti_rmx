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

class PassThrough final {
public:
    PassThrough(int cutoff = 0, int reso = 0) {}

    inline int process(int v) {
        return v;
    }
};



class LP final {
public:
    LP(int cutoff = 0) {
        update(cutoff);
    }

    inline void update(int cutoff) {
        MTOF(cutoff, f);
    }

    inline int process(int sample) {
        v = ___SMMLA((sample - v) << 1, f, v);
        return v;
    }
    
    inline void process(const int32buffer in, int32buffer out) {
        for (int i = 0; i < BUFSIZE; ++i) {
            out[i] = process(in[i]);
        }
    }

private:
    int f = 0;
    int v = 0;
};



class HP final {
public:
    HP(int cutoff = 0) {
        update(cutoff);
    }

    inline void update(int cutoff) {
        MTOF(cutoff, f);
    }
    
    inline int process(int sample) {
        v = ___SMMLA((sample - v) << 1, f, v);
        return sample - v;
    }

    inline void process(const int32buffer in, int32buffer out) {
        for (int i = 0; i < BUFSIZE; ++i) {
            out[i] = process(in[i]);
        }
    }

private:
    int f = 0;
    int v = 0;
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

    inline int process(int in) {
        int accu = ___SMMUL(cxn_0, in);
        accu = ___SMMLA(cxn_1, filter_x_n1, accu);
        accu = ___SMMLA(cxn_2, filter_x_n2, accu);
        accu = ___SMMLS(cyn_1, filter_y_n1, accu);
        accu = ___SMMLS(cyn_2, filter_y_n2, accu);
        int output = accu << 4;
        filter_x_n2 = filter_x_n1;
        filter_x_n1 = in;
        filter_y_n2 = filter_y_n1;
        filter_y_n1 = output;
        return __SSAT(output, 28);
    }

protected:
    bool doUpdate(int cutoff, int reso, int& cosW0, int& alpha, int& a0_inv_q31, int& q_inv) {
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
        update(cutoff, reso);
    }

    void update(int cutoff, int reso) {
        int cosW0;
        int alpha;
        int a0_inv_q31;
        int q_inv;
        if (!doUpdate(cutoff, reso, cosW0, alpha, a0_inv_q31, q_inv))
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
        update(cutoff, reso);
    }

    void update(int cutoff, int reso) {
        int cosW0;
        int alpha;
        int a0_inv_q31;
        int q_inv;
        if (!doUpdate(cutoff, reso, cosW0, alpha, a0_inv_q31, q_inv))
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
        update(cutoff, reso);
    }

    void update(int cutoff, int reso) {
        int cosW0;
        int alpha;
        int a0_inv_q31;
        int q_inv;
        if (!doUpdate(cutoff, reso, cosW0, alpha, a0_inv_q31, q_inv))
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
        update(cutoff, reso);
    }

    void update(int cutoff, int reso) {
        int alpha = 0;
        MTOFEXTENDED(cutoff, alpha);
        SINE2TINTERP(alpha, freq);		

        // slightly less resonance if reso = 0 compared to orignal implementation
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

} // namespace rmx
