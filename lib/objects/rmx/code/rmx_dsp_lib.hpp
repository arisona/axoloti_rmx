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


// EnvelopeFollower
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

	int process(int attack, int release, const int32_t* input0, const int32_t* input1) {
        update(attack, release);
		return process(input0, input1);
	}

	int process(const int32_t* input0, const int32_t* input1) {
		int level;
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
public:
	EnvelopeFollowerSimple() {
	}

	int process(const int32_t* input0, const int32_t* input1) {
		int32_t level = 0;
		for (int i = 0; i < BUFSIZE; ++i) {
			int32_t s;
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


class SVF final {
	// from Axoloti SVF
	// in addition, see: http://www.musicdsp.org/showArchiveComment.php?ArchiveID=92
public:
	SVF() {}

	void update(int cutoff, int reso) {
		int alpha = 0;
		MTOFEXTENDED(cutoff, alpha);
		SINE2TINTERP(alpha, freq);		

		damp = (0x80 << 24) - (reso << 3);
		damp = ___SMMUL(damp, damp);
	}

	inline int filterLow(int sample) {
		int notch = sample - (___SMMUL(damp, band) << 1);
		low = low + (___SMMUL(freq, band) << 1);
		int high = notch - low;
		band = (___SMMUL(freq, high) << 1) + band;
		return low;
	}

	inline int filterHigh(int sample) {
		int notch = sample - (___SMMUL(damp, band) << 1);
		low = low + (___SMMUL(freq, band) << 1);
		int high = notch - low;
		band = (___SMMUL(freq, high) << 1) + band;
		return high;
	}

private:
	int freq = 0;
	int damp = 0;

	int low = 0;
	int band = 0;
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

    void update(const int32_t* inBufL, const int32_t* inBufR,
                const int time, const int offset, const int timeMod,
                const int feedback, const int pingpong,
                const int hpCutoff, const int hpReso, const int hpMod,
                const int lpCutoff, const int lpReso, const int lpMod,
                const int modRate, const int modMix) {

        // update time / read offsets
        if (!isFading && (time != this->time || offset != this->offset)) {
            this->time = time;
            this->offset = offset;

            // update read offset
            fadeOutOffsetL = fadeInOffsetL;
            fadeInOffsetL = time2offset(time, length);

            fadeOutOffsetR = fadeInOffsetR;
            fadeInOffsetR = fadeInOffsetL + time2offset(offset, length);
            
            isFading = true;
            fadeInLevel = 0;
            fadeOutLevel = ONE;
        }

        // update feedback / pingpong
        this->feedback = feedback;
        this->sendRL = pingpong;
        this->sendLR = ONE - sendRL;

        // update lfo modulation
        if (modRate > MIN) {
            int mod;
            MTOFEXTENDED(modRate, mod)
            modPhase += mod >> 6;
        }

        if (modRate > MIN) {
            SINE2TINTERP(modPhase, this->timeMod);
            this->timeMod = ___SMMUL(this->timeMod, this->timeMod);
            this->timeMod = ___SMMUL(this->timeMod, timeMod) >> 16;
        } else {
            this->timeMod = 0;
        }

        if (modRate > MIN) {
            int mod;
            SINE2TINTERP(modPhase, mod);	
            mod = ___SMMUL(mod, mod);
            this->hpMod = ___SMMUL(mod, hpMod);
            this->lpMod = ___SMMUL(mod, lpMod);
        } else {
            this->hpMod = 0;
            this->lpMod = 0;
        }

        // update env modulation
        int env = envelope.process(inBufL, inBufR);
        this->timeMod = __SSAT(this->timeMod + ___SMMUL(env, ___SMMUL(timeMod, modMix) >> 8), 28);
        this->hpMod = __SSAT(this->hpMod + ___SMMUL(env << 3, ___SMMUL(hpMod << 3, modMix << 2) << 3), 28);
        this->lpMod = __SSAT(this->lpMod + ___SMMUL(env << 3, ___SMMUL(lpMod << 3, modMix << 2) << 3), 28);

        // update filters
        int cutoff;
        cutoff = __SSAT(hpCutoff + this->hpMod, 28);
        hpL.update(cutoff, hpReso);
        hpR.update(cutoff, hpReso);

        cutoff = lpCutoff <= this->lpMod ? 1 : (lpCutoff - this->lpMod);
        lpL.update(cutoff, lpReso);
        lpR.update(cutoff, lpReso);
    }

    int debug = 0;

    void process(const int32_t* inBufL, const int32_t* inBufR, int32_t* outBufL, int32_t* outBufR) {
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
            int indexInL = mod((writepos - fadeInOffsetL - timeMod), length);
            int indexInR = mod((writepos - fadeInOffsetR - timeMod), length);
            int indexOutL = mod((writepos - fadeOutOffsetL - timeMod), length);
            int indexOutR = mod((writepos - fadeOutOffsetR - timeMod), length);
            int outL = ___SMMUL(bufferL[indexInL] << 3, fadeInLevel << 2) + ___SMMUL(bufferL[indexOutL] << 3, fadeOutLevel << 2);
            int outR = ___SMMUL(bufferR[indexInR] << 3, fadeInLevel << 2) + ___SMMUL(bufferR[indexOutR] << 3, fadeOutLevel << 2);

            // filter output
            outL = __SSAT(lpL.filterLow(__SSAT(hpL.filterHigh(outL), 28)), 28);
            outR = __SSAT(lpR.filterLow(__SSAT(hpR.filterHigh(outR), 28)), 28);

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
    inline int time2offset(int t, int length) {
        // this table is pretty ad-hoc. it increases non-linear first, afterwards linear up to 2 seconds.
        static const int OFFSETS[128] = {
            10, 20, 30, 40, 50, 60, 70, 80, 90, 100, 120, 140, 180, 220, 240,
            1725, 3210, 4695, 6180, 7665, 9150, 10635, 12120, 13605, 15090, 16575, 18060, 19545, 21030, 22515, 24000,
            
            24750, 25500, 26250, 27000, 27750, 28500, 29250, 30000, 30750, 31500, 32250, 33000, 33750, 34500, 35250, 36000, 
            36750, 37500, 38250, 39000, 39750, 40500, 41250, 42000, 42750, 43500, 44250, 45000, 45750, 46500, 47250, 48000,

            48750, 49500, 50250, 51000, 51750, 52500, 53250, 54000, 54750, 55500, 56250, 57000, 57750, 58500, 59250, 60000,
            60750, 61500, 62250, 63000, 63750, 64500, 65250, 66000, 66750, 67500, 68250, 69000, 69750, 70500, 71250, 72000,

            72750, 73500, 74250, 75000, 75750, 76500, 77250, 78000, 78750, 79500, 80250, 81000, 81750, 82500, 83250, 84000,
            84750, 85500, 86250, 87000, 87750, 88500, 89250, 90000, 90750, 91500, 92250, 93000, 93750, 94500, 95250, 96000,
        };

		t = (t >> (27 - 7)) & 0x7f;
        return std::min(OFFSETS[t], length - 1);
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

    // delay time and offset (cached, so we can track when time changes to initiated fading)
    int time = 0;
    int offset = 0;

    // feedback & pingpong
    int feedback = 0;
    int sendLR = 0;
    int sendRL = 0;
    int lastOutL = 0;
    int lastOutR = 0;

    // modulation
    uint32_t modPhase = 0;
    int timeMod = 0;
    int hpMod = 0;
    int lpMod = 0;
    EnvelopeFollowerSimple envelope;

    // lp and hp filter states
    SVF hpL;
    SVF hpR;
    SVF lpL;
    SVF lpR;
};

} // namespace rmx
