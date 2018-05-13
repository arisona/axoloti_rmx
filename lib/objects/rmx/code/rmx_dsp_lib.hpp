#pragma once

namespace rmx {

static const int BUFSIZEPOW = 4;

static const int ONE = (1 << 27) - 1;
static const int MIN = -(1 << 27);


inline int clamp(int min, int max, int value) {
   return std::max(min, std::min(max, value));
}

inline float clamp(float min, float max, float value) {
   return std::max(min ,std::min(max, value));
}

inline int mod(int a, int b) {
    int r = a % b;
    return r < 0 ? r + b : r;
}


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
		return __SSAT(env, 28);
    }

	inline int get() const {
		return env;
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
		return float_to_q27((float)pow(0.1, 1.0 / (EXP_T[t] * SAMPLERATE / BUFSIZE)));
	}

	int attack = -1;
	int release = -1;

	int a = 0;
	int r = 0;
	int env = 0;
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


namespace detail {

class SVFBase {
	// from Axoloti SVF
	// in addition, see: http://www.musicdsp.org/showArchiveComment.php?ArchiveID=92
public:
	SVFBase() {}

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
    SVFLP() {}

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
    SVFBP() {}

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
    SVFHP() {}

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
    bool doSetup(int cutoff, int reso) {
        if (cutoff == this->cutoff && reso == this->reso)
            return false;
        this->cutoff = cutoff;
        this->reso = reso;
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
    BiquadLP() {}

    void setup(int cutoff, int reso) {
        if (!doSetup(cutoff, reso))
            return;

        int filter_W0;
        MTOF(cutoff, filter_W0);
        int q_inv = INT_MAX - (__USAT(reso, 27) << 4);

        filter_W0 = filter_W0 >> 1;
        int sinW0 = arm_sin_q31(filter_W0);
        int cosW0 = arm_cos_q31(filter_W0);
        int alpha = ___SMMUL(sinW0, q_inv);
        float filter_a0 = (HALFQ31 + alpha);
        float filter_a0_inv = ((INT32_MAX >> 2) / filter_a0);
        int a0_inv_q31 = (int)(INT32_MAX * filter_a0_inv);
        cyn_1 = ___SMMUL((-cosW0), a0_inv_q31);
        cyn_2 = ___SMMUL((HALFQ31 - alpha), a0_inv_q31);
        cxn_0 = ___SMMUL(___SMMUL(HALFQ31 - (cosW0 >> 1), a0_inv_q31), q_inv);
        cxn_1 = cxn_0 << 1;
        cxn_2 = cxn_0;        
    }
};

class BiquadBP final : public detail::BiquadBase {
public:
    BiquadBP() {}

    void setup(int cutoff, int reso) {
        if (!doSetup(cutoff, reso))
            return;

        int filter_W0;
        MTOF(cutoff, filter_W0);
        int q_inv = INT_MAX - (__USAT(reso, 27) << 4);

        filter_W0 = filter_W0 >> 1;
        int sinW0 = arm_sin_q31(filter_W0);
        int cosW0 = arm_cos_q31(filter_W0);
        int alpha = ___SMMUL(sinW0, q_inv);
        float filter_a0 = (HALFQ31 + alpha);
        float filter_a0_inv = ((INT32_MAX >> 2) / filter_a0);
        int a0_inv_q31 = (int)(INT32_MAX * filter_a0_inv);
        cyn_1 = ___SMMUL((-cosW0), a0_inv_q31);
        cyn_2 = ___SMMUL((HALFQ31 - alpha), a0_inv_q31);
        cxn_0 = ___SMMUL(alpha, a0_inv_q31);
        cxn_1 = 0;
        cxn_2 = -cxn_0;
    }
};

class BiquadHP final : public detail::BiquadBase {
public:
    BiquadHP() {}

    void setup(int cutoff, int reso) {
        if (!doSetup(cutoff, reso))
            return;

        int filter_W0;
        MTOF(cutoff, filter_W0);
        int q_inv = INT_MAX - (__USAT(reso, 27) << 4);

        filter_W0 = filter_W0 >> 1;
        int sinW0 = arm_sin_q31(filter_W0);
        int cosW0 = arm_cos_q31(filter_W0);
        int alpha = ___SMMUL(sinW0, q_inv);
        float filter_a0 = (HALFQ31 + alpha);
        float filter_a0_inv = ((INT32_MAX >> 2) / filter_a0);
        int a0_inv_q31 = (int)(INT32_MAX * filter_a0_inv);
        cyn_1 = ___SMMUL((-cosW0), a0_inv_q31);
        cyn_2 = ___SMMUL((HALFQ31 - alpha), a0_inv_q31);
        cxn_0 = ___SMMUL(___SMMUL(HALFQ31 + (cosW0 >> 1), a0_inv_q31), q_inv);
        cxn_1 = -(cxn_0 << 1);
        cxn_2 = cxn_0;
    }
};



class FadeDelay final {
public:
    FadeDelay(int length, int fadeTime) : length(length), step((1 << 27) >> fadeTime)  {
    }

    void setup(int32_t* bufferL, int32_t* bufferR) {
        this->bufferL = bufferL;
        this->bufferR = bufferR;

        for (int i = 0; i < length; ++i) {
            bufferL[i] = 0;
            bufferR[i] = 0;
        }
    }

    int debug = 0;
    void process(const int32buffer inBufL, const int32buffer inBufR,
                 const int inEnv,
                 int32buffer outBufL, int32buffer outBufR,
                 const int time, const int offset, const int timeMod,
                 const int feedback, const int pingpong,
                 const int hpCutoff, const int hpReso, const int hpMod,
                 const int lpCutoff, const int lpReso, const int lpMod,
                 const int modRate, const int modEnv,
                 const int bpm) {

        // update feedback / pingpong
        int sendRL = pingpong;
        int sendLR = ONE - sendRL;

        // update lfo modulation
        int modHpCutoff = 0;
        int modLpCutoff = 0;
        if (modRate > MIN) {
            // lfo
            int phase;
            MTOFEXTENDED(modRate, phase)
            modPhase += phase >> 6;

            int mod;
            SINE2TINTERP(modPhase, mod);	
            mod = ___SMMUL(mod, mod);

            // time
            modTime = ___SMMUL(mod, timeMod) >> 16;

            // filters
            modHpCutoff = ___SMMUL(mod, hpMod);
            modLpCutoff = ___SMMUL(mod, lpMod);
        } else {
            // keep quiet if rate is 0 (= oscillator off)
            modPhase = 0;
            modTime = std::max(0, modTime - 1);
        }

        // update env modulation (combine with lfo modulation)
        int modOffset = __SSAT(modTime + ___SMMUL(inEnv, ___SMMUL(timeMod, modEnv) >> 8), 28);
        modHpCutoff = __SSAT(modHpCutoff + ___SMMUL(inEnv << 3, ___SMMUL(hpMod << 3, modEnv << 2) << 3), 28);
        modLpCutoff = __SSAT(modLpCutoff + ___SMMUL(inEnv << 3, ___SMMUL(lpMod << 3, modEnv << 2) << 3), 28);

        // update filters
        int cutoff;
        cutoff = __SSAT(hpCutoff + modHpCutoff, 28);
        hpL.setup(cutoff, hpReso);
        hpR.setup(cutoff, hpReso);

        cutoff = __SSAT(lpCutoff - modLpCutoff, 28);
        lpL.setup(cutoff, lpReso);
        lpR.setup(cutoff, lpReso);

        // update time / read offsets
        if (!isFading && (time != this->time || offset != this->offset || bpm != this->bpm)) {
            this->time = time;
            this->offset = offset;
            this->bpm = bpm;

            fadeOutOffsetL = fadeInOffsetL;
            fadeInOffsetL = time2offset(time, length, bpm);

            fadeOutOffsetR = fadeInOffsetR;
            fadeInOffsetR = fadeInOffsetL + time2offset(offset, length);
            
            isFading = true;
            fadeInLevel = 0;
            fadeOutLevel = ONE;
        }

        // finally process buffers
        for (int s = 0; s < BUFSIZE; ++s) {
            // update feedback 
            int feedbackL = ___SMMUL(lastOutL << 3, sendLR << 2) + ___SMMUL(lastOutR << 3, sendRL << 2);	
            int feedbackR = ___SMMUL(lastOutR << 3, sendLR << 2) + ___SMMUL(lastOutL << 3, sendRL << 2);	
            feedbackL = __SSAT(___SMMUL(feedbackL << 3, feedback << 2), 28);
            feedbackR = __SSAT(___SMMUL(feedbackR << 3, feedback << 2), 28);
            
            // apply pingpong to input
            int inL = ___SMMUL(inBufL[s] << 3, sendLR << 2) + ___SMMUL(inBufL[s] << 3, sendRL << 2);	
            int inR = ___SMMUL(inBufR[s] << 3, sendLR << 2) + ___SMMUL(inBufR[s] << 3, sendRL << 2);	

            // update delay
            writepos = mod((writepos + 1), length);
            bufferL[writepos] = __SSAT(inL + feedbackL, 28);
            bufferR[writepos] = __SSAT(inR + feedbackR, 28);

            // read and mix fade in / out
            int indexInL = mod(writepos - fadeInOffsetL - modOffset, length);
            int indexInR = mod(writepos - fadeInOffsetR - modOffset, length);
            int indexOutL = mod(writepos - fadeOutOffsetL - modOffset, length);
            int indexOutR = mod(writepos - fadeOutOffsetR - modOffset, length);
            int outL = ___SMMUL(bufferL[indexInL] << 3, fadeInLevel << 2) + ___SMMUL(bufferL[indexOutL] << 3, fadeOutLevel << 2);
            int outR = ___SMMUL(bufferR[indexInR] << 3, fadeInLevel << 2) + ___SMMUL(bufferR[indexOutR] << 3, fadeOutLevel << 2);

            // filter output
            outL = __SSAT(lpL.process(__SSAT(hpL.process(outL), 28)), 28);
            outR = __SSAT(lpR.process(__SSAT(hpR.process(outR), 28)), 28);

            // write output
            outBufL[s] = lastOutL = __SSAT(outL, 28);
            outBufR[s] = lastOutR = __SSAT(outR, 28);

            // update fades
            if (fadeInLevel < ONE) {
                fadeInLevel += step;
                fadeOutLevel -= step;
            } else {
                fadeInLevel = ONE;
                fadeOutLevel = 0;
                isFading = false;
            }
        }
    }

private:
    // todo: synced delay time options
    // echo: 1/64 1/32 1/16 1/8 1/4 1/2 1
    // pingpong: 1 2 3 4 5 6 8 16 / 16
    // pingpong: 4 8 12 16 20 24 32 64 / 64
    // combined: 1/64 2/64 4/64 8/64 12/64 16/64 20/64 24/64 32/64 64/64

    inline int time2offset(int t, int length, int bpm = 0) {
        // time table: it increases exp (almost) first, afterwards linear up to 2 seconds.
        static const int TIMES[128] = {
            // start=0 (0ms) end=6000.0 (125.0ms) inc=exponential (including 0, strictly monotonic)
            0, 1, 2, 3, 4, 5, 6, 7, 9, 12, 16, 21, 29, 38, 50, 67, 
            89, 118, 156, 206, 273, 362, 480, 635, 841, 1114, 1474, 1952, 2585, 3422, 4531, 6000, 

            // start=6562.5 (136.71875ms) end=24000.0 (500.0ms) inc=562.5 (11.71875ms) 
            6562, 7125, 7687, 8250, 8812, 9375, 9937, 10500, 11062, 11625, 12187, 12750, 13312, 13875, 14437, 15000, 
            15562, 16125, 16687, 17250, 17812, 18375, 18937, 19500, 20062, 20625, 21187, 21750, 22312, 22875, 23437, 24000, 

            // start=24750.0 (515.625ms) end=48000.0 (1000.0ms) inc=750.0 (15.625ms) 
            24750, 25500, 26250, 27000, 27750, 28500, 29250, 30000, 30750, 31500, 32250, 33000, 33750, 34500, 35250, 36000, 
            36750, 37500, 38250, 39000, 39750, 40500, 41250, 42000, 42750, 43500, 44250, 45000, 45750, 46500, 47250, 48000, 

            // start=49500.0 (1031.25ms) end=96000.0 (2000.0ms) inc=1500.0 (31.25ms) 
            49500, 51000, 52500, 54000, 55500, 57000, 58500, 60000, 61500, 63000, 64500, 66000, 67500, 69000, 70500, 72000, 
            73500, 75000, 76500, 78000, 79500, 81000, 82500, 84000, 85500, 87000, 88500, 90000, 91500, 93000, 94500, 96000, 
        };

        // beat table: multiples of 1/64 notes
        static const float BEATS[] = {
            0, 1, 2, 4, 8, 12, 16, 20, 24, 32
        };

        if (bpm == 0) {
            t = (t >> (27 - 7)) & 0x7f;
            return std::min(TIMES[t], length - 1);
        } else {
            // make sure values are properly scaled on dial
            t = (int)(8.4f * q27_to_float(t) + 0.8f);
            if (t < 0)
                return 0;

            float duration64th = (60.0f / 16.0f) / bpm;
            float duration = duration64th * BEATS[t];
            int samples = (int)(duration * SAMPLERATE);
            return std::min(samples, length - 1);
        }
    }


    int32_t* bufferL = nullptr;
    int32_t* bufferR = nullptr;

    int length = 0;

    int step = 0;

    int writepos = 0;

    int fadeInOffsetL = 0;
    int fadeInOffsetR = 0;
    int fadeInLevel = 0;

    int fadeOutOffsetL = 0;
    int fadeOutOffsetR = 0;
    int fadeOutLevel = ONE;

    bool isFading = false;

    // time parameters (cached, so we can track when time changes to initiated fading)
    int time = 0;
    int offset = 0;
    int bpm = 0;

    // feedback
    int lastOutL = 0;
    int lastOutR = 0;

    // modulation (lfo & env)
    unsigned int modPhase = 0;
    int modTime = 0;

    // lp and hp filter types & states
    using HP = SVFHP;
    using LP = SVFLP;
    HP hpL;
    HP hpR;
    LP lpL;
    LP lpR;
};

} // namespace rmx
