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

// the filters all have the same api (cutoff, reso, drive), also when
// reso and drive are ignored for some types. this way, they can be
// used as templates in an uniform manner.

class PassThrough final {
public:
    PassThrough(int cutoff = 0, int reso = 0, int drive = 0) {}

    inline void update(int cutoff, int reso = 0, int drive = 0) {
    }

    inline int process(int v) {
        return v;
    }
};



class LP final {
public:
    LP(int cutoff = 0, int reso = 0, int drive = 0) {
        update(cutoff);
    }

    inline void update(int cutoff, int reso = 0, int drive = 0) {
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
    HP(int cutoff = 0, int reso = 0, int drive = 0) {
        update(cutoff);
    }

    inline void update(int cutoff, int reso = 0, int drive = 0) {
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

    inline int process(int sample) {
        int accu = ___SMMUL(cxn_0, sample);
        accu = ___SMMLA(cxn_1, filter_x_n1, accu);
        accu = ___SMMLA(cxn_2, filter_x_n2, accu);
        accu = ___SMMLS(cyn_1, filter_y_n1, accu);
        accu = ___SMMLS(cyn_2, filter_y_n2, accu);
        int output = accu << 4;
        filter_x_n2 = filter_x_n1;
        filter_x_n1 = sample;
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

    void update(int cutoff, int reso = 0, int drive = 0) {
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
    BiquadBP(int cutoff = 0, int reso = 0, int drive = 0) {
        update(cutoff, reso);
    }

    void update(int cutoff, int reso = 0, int drive = 0) {
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
    BiquadHP(int cutoff = 0, int reso = 0, int drive = 0) {
        update(cutoff, reso);
    }

    void update(int cutoff, int reso = 0, int drive = 0) {
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
    // from Axoloti SVF, but with 2x oversampling
    // in addition, see: http://www.musicdsp.org/showArchiveComment.php?ArchiveID=92
    // note that drive implementation a bit esotheric...
public:
    SVFBase(int cutoff, int reso) {
        update(cutoff, reso);
    }

    void update(int cutoff, int reso = 0, int drive = 0) {
        int alpha = 0;
        MTOFEXTENDED(cutoff, alpha);
        SINE2TINTERP(alpha, freq);		

        // slightly less resonance if reso = 0 compared to orignal implementation
        damp = INT_MAX - (reso << 3) - (reso << 2);

        // adjust drive (less drive for high frequencies)
        this->drive = (drive << 4) >> (freq >> 28);
    }

protected:
    inline void processInternal(int sample) {
        int notch = sample - (___SMMUL(damp, band) << 1);
        low = low + (___SMMUL(freq, band) << 1);
        high = notch - low;

        int db3 = ___SMMUL(band, band) << 5;
        db3 = ___SMMUL(db3, band) << 5;
        db3 = ___SMMUL(db3, drive) << 5;
        band = (___SMMUL(freq, high) << 1) + band - db3;
    }

    int freq = 0;
    int damp = 0;
    int drive = 0;

    int low = 0;
    int high = 0;
    int band = 0;
};

} // namespace detail

class SVFLP final : public detail::SVFBase {
public:
    SVFLP(int cutoff = 0, int reso = 0, int drive = 0) : SVFBase(cutoff, reso) {}

    inline int process(int sample) {
        processInternal(sample);
        int low0 = low >> 1;
        processInternal(sample);
        int low1 = low >> 1;
        return __SSAT(low0 + low1, 28);
    }

    inline void process(const int32buffer in, int32buffer out) {
        for (int i = 0; i < BUFSIZE; ++i) {
            out[i] = process(in[i]);
        }
    }
};

class SVFBP final : public detail::SVFBase {
public:
    SVFBP(int cutoff = 0, int reso = 0, int drive = 0) : SVFBase(cutoff, reso) {}

    inline int process(int sample) {
        processInternal(sample);
        int band0 = band >> 1;
        processInternal(sample);
        int band1 = band >> 1;
        return __SSAT(band0 + band1, 28);
    }

    inline void process(const int32buffer in, int32buffer out) {
        for (int i = 0; i < BUFSIZE; ++i) {
            out[i] = process(in[i]);
        }
    }    
};

class SVFHP final : public detail::SVFBase {
public:
    SVFHP(int cutoff = 0, int reso = 0, int drive = 0) : SVFBase(cutoff, reso) {}

    inline int process(int sample) {
        processInternal(sample);
        int high0 = high >> 1;
        processInternal(sample);
        int high1 = high >> 1;
        return __SSAT(high0 + high1, 28);
    }

    inline void process(const int32buffer in, int32buffer out) {
        for (int i = 0; i < BUFSIZE; ++i) {
            out[i] = process(in[i]);
        }
    }
};

} // namespace rmx
