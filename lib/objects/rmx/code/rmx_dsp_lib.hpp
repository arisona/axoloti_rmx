#pragma once

namespace rmx {

static const int32_t BUFSIZEPOW = 4;

static const int32_t ONE = (1 << 27) - 1;
static const int32_t MIN = -(1 << 27);



inline int32_t clamp(int32_t min, int32_t max, int32_t value) {
   return std::max<int32_t>(min, std::min<int32_t>(max, value));
}

inline float clamp(float min, float max, float value) {
   return std::max<float>(min ,std::min<float>(max, value));
}



// EnvelopeFollower
class EnvelopeFollower final {	
public:
    EnvelopeFollower(uint32_t attack = 0, uint32_t release = 0) {
        update(attack, release);
    }

    void update(int32_t attack, int32_t release) {
		if (attack != this->attack) {
			this->attack = attack;
			a = time2gain(attack);
		}

		if (release != this->release) {
			this->release = release;
			r = time2gain(release);
		}
    }

	int32_t process(int32_t attack, int32_t release, const int32_t* input0, const int32_t* input1) {
        update(attack, release);
		return process(input0, input1);
	}

	int32_t process(const int32_t* input0, const int32_t* input1) {
		int32_t level;
		for (int i = 0; i < BUFSIZE; ++i) {
			int32_t s;
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

	inline int32_t get() const {
		return env;
	}

private:
	inline int32_t time2gain(int32_t t) {
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

	int32_t attack = -1;
	int32_t release = -1;

	int32_t a = 0;
	int32_t r = 0;
	int32_t env = 0;
};


class EnvelopeFollowerSimple final {
public:
	EnvelopeFollowerSimple() {
	}

	int32_t process(const int32_t* input0, const int32_t* input1) {
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

	inline int32_t get() const {
		return envelope;
	}

private:
	// 2=1.3ms 3=2.7ms 4=5.3ms 5=10.6ms 6=21.2ms 7=42.6ms 8=85.3ms 9=170.7ms
	int32_t slope = 4;
	int32_t envelope = 0;
};


class SVF final {
	// from Axoloti SVF
	// in addition, see: http://www.musicdsp.org/showArchiveComment.php?ArchiveID=92
public:
	SVF() {}

	void update(int32_t cutoff, uint32_t reso) {
		int32_t alpha = 0;
		MTOFEXTENDED(cutoff, alpha);
		SINE2TINTERP(alpha, freq);		

		damp = (0x80 << 24) - (reso << 3);
		damp = ___SMMUL(damp, damp);
	}

	inline int32_t filterLow(int32_t sample) {
		int32_t notch = sample - (___SMMUL(damp, band) << 1);
		low = low + (___SMMUL(freq, band) << 1);
		int32_t high = notch - low;
		band = (___SMMUL(freq, high) << 1) + band;
		return low;
	}

	inline int32_t filterHigh(int32_t sample) {
		int32_t notch = sample - (___SMMUL(damp, band) << 1);
		low = low + (___SMMUL(freq, band) << 1);
		int32_t high = notch - low;
		band = (___SMMUL(freq, high) << 1) + band;
		return high;
	}

private:
	int32_t freq = 0;
	int32_t damp = 0;

	int32_t low = 0;
	int32_t band = 0;
};


class FadeDelay final {
public:
    FadeDelay(int32_t lengthPow, int32_t fadeTime)  {
        this->lengthPow = lengthPow;
        length = 1 << lengthPow;
        lengthMask = length - 1;

        step = (1 << 27) >> fadeTime;
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
                const int32_t time, const int32_t offset, const int32_t timeMod,
                const int32_t feedback, const int32_t pingpong,
                const int32_t hpCutoff, const int32_t hpReso, const int32_t hpMod,
                const int32_t lpCutoff, const int32_t lpReso, const int32_t lpMod,
                const int32_t modRate, const int32_t modMix) {

        // update time / read offsets
        if (!isFading && (time != this->time || offset != this->offset)) {
            this->time = time;
            this->offset = offset;

            // update read offset
            fadeOutOffsetL = fadeInOffsetL;
            fadeInOffsetL = (time >> (27 - lengthPow)) - length;
            fadeOutOffsetR = fadeInOffsetR;
            fadeInOffsetR = ((time + offset) >> (27 - lengthPow)) - length;
            
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
            int32_t mod;
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
            int32_t mod;
            SINE2TINTERP(modPhase, mod);	
            mod = ___SMMUL(mod, mod);
            this->hpMod = ___SMMUL(mod, hpMod);
            this->lpMod = ___SMMUL(mod, lpMod);
        } else {
            this->hpMod = 0;
            this->lpMod = 0;
        }

        // update env modulation
        int32_t env = envelope.process(inBufL, inBufR);
        this->timeMod = __SSAT(this->timeMod + ___SMMUL(env, ___SMMUL(timeMod, modMix) >> 8), 28);
        this->hpMod = __SSAT(this->hpMod + ___SMMUL(env << 3, ___SMMUL(hpMod << 3, modMix << 2) << 3), 28);
        this->lpMod = __SSAT(this->lpMod + ___SMMUL(env << 3, ___SMMUL(lpMod << 3, modMix << 2) << 3), 28);
        debug = modMix;

        // update filters
        int32_t cutoff;
        cutoff = __SSAT(hpCutoff + this->hpMod, 28);
        hpL.update(cutoff, hpReso);
        hpR.update(cutoff, hpReso);

        cutoff = lpCutoff <= this->lpMod ? 1 : (lpCutoff - this->lpMod);
        lpL.update(cutoff, lpReso);
        lpR.update(cutoff, lpReso);
    }

    int32_t debug = 0;

    void process(const int32_t* inBufL, const int32_t* inBufR, int32_t* outBufL, int32_t* outBufR) {
        for (int s = 0; s < BUFSIZE; ++s) {
            // update feedback 
            int32_t feedbackL = ___SMMUL(lastOutL << 3, sendLR << 2) + ___SMMUL(lastOutR << 3, sendRL << 2);	
            int32_t feedbackR = ___SMMUL(lastOutR << 3, sendLR << 2) + ___SMMUL(lastOutL << 3, sendRL << 2);	
            feedbackL = ___SMMUL(feedbackL << 3, feedback << 2);
            feedbackR = ___SMMUL(feedbackR << 3, feedback << 2);
            
            // apply pingpong to input
            int32_t inL = ___SMMUL(inBufL[s] << 3, sendLR << 2) + ___SMMUL(inBufL[s] << 3, sendRL << 2);	
            int32_t inR = ___SMMUL(inBufR[s] << 3, sendLR << 2) + ___SMMUL(inBufR[s] << 3, sendRL << 2);	

            // update delay
            writepos = (writepos + 1) & lengthMask;
            bufferL[writepos] = __SSAT(inL + feedbackL, 28);
            bufferR[writepos] = __SSAT(inR + feedbackR, 28);

            // read and mix fade in / out
            int indexInL = (writepos - fadeInOffsetL - timeMod) & lengthMask;
            int indexInR = (writepos - fadeInOffsetR - timeMod) & lengthMask;
            int indexOutL = (writepos - fadeOutOffsetL - timeMod) & lengthMask;
            int indexOutR = (writepos - fadeOutOffsetR - timeMod) & lengthMask;
            int32_t outL = ___SMMUL(bufferL[indexInL] << 3, fadeInLevel << 2) + ___SMMUL(bufferL[indexOutL] << 3, fadeOutLevel << 2);
            int32_t outR = ___SMMUL(bufferR[indexInR] << 3, fadeInLevel << 2) + ___SMMUL(bufferR[indexOutR] << 3, fadeOutLevel << 2);

            // filter output
            outL = lpL.filterLow(hpL.filterHigh(outL));
            outR = lpR.filterLow(hpR.filterHigh(outR));

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
    int32_t* bufferL = nullptr;
    int32_t* bufferR = nullptr;

    uint32_t lengthPow = 0;
    uint32_t length = 0;
    uint32_t lengthMask = 0;

    uint32_t step = 0;

    uint32_t writepos = 0;

    uint32_t fadeInOffsetL = 0;
    uint32_t fadeInOffsetR = 0;
    int32_t fadeInLevel = 0;

    uint32_t fadeOutOffsetL = 0;
    uint32_t fadeOutOffsetR = 0;
    int32_t fadeOutLevel = ONE;

    bool isFading = false;

    // delay time and offset (cached, so we can track when time changes to initiated fading)
    int32_t time = 0;
    int32_t offset = 0;

    // feedback & pingpong
    int32_t feedback = 0;
    int32_t sendLR = 0;
    int32_t sendRL = 0;
    int32_t lastOutL = 0;
    int32_t lastOutR = 0;

    // modulation
    uint32_t modPhase = 0;
    int32_t timeMod = 0;
    int32_t hpMod = 0;
    int32_t lpMod = 0;
    EnvelopeFollowerSimple envelope;

    // lp and hp filter states
    SVF hpL;
    SVF hpR;
    SVF lpL;
    SVF lpR;
};

} // namespace rmx
