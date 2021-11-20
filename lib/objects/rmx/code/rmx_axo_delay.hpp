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

class FadeDelay final {
public:
    FadeDelay(int length, int fadeTime) : length(length), step((1 << 27) >> fadeTime)  {
    }

    // temporary use for passing debug values to the gui
    int debug = 0;

    void setup(int* bufferL, int* bufferR) {
        this->bufferL = bufferL;
        this->bufferR = bufferR;

        for (int i = 0; i < length; ++i) {
            bufferL[i] = 0;
            bufferR[i] = 0;
        }
    }

    void process(const int32buffer inBufL, const int32buffer inBufR,
                 int32buffer outBufL, int32buffer outBufR,
                 const int time, const int offset, const int timeMod,
                 const int feedback, const int pingpong,
                 const int hpCutoff, const int hpReso, const int hpDrive, const int hpMod,
                 const int lpCutoff, const int lpReso, const int lpDrive, const int lpMod,
                 const int modRate, const int modEnv,
                 const int bdur, const int env) {

        // update feedback / pingpong
        int sendRL = pingpong;
        int sendLR = ONE - sendRL;

        // update lfo modulation
        int modHpCutoff = 0;
        int modLpCutoff = 0;
        if (modRate > MINUS_ONE) {
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
            // keep quiet if rate is -1 (= oscillator off)
            modPhase = 0;
            modTime = std::max(0, modTime - 1);
        }

        // update env modulation (combine with lfo modulation)
        int modOffset = __SSAT(modTime + (___SMMUL(env << 3, ___SMMUL(timeMod << 3, modEnv << 2) << 2) >> 18), 28);
        modHpCutoff = __SSAT(modHpCutoff + (___SMMUL(env << 3, ___SMMUL(hpMod << 3, modEnv << 2) << 2) << 2), 28);
        modLpCutoff = __SSAT(modLpCutoff + (___SMMUL(env << 3, ___SMMUL(lpMod << 3, modEnv << 2) << 2) << 2), 28);

        // update filters
        int cutoff;
        cutoff = __SSAT(hpCutoff + modHpCutoff, 28);
        hpL.update(cutoff, hpReso, hpDrive);
        hpR.update(cutoff, hpReso, hpDrive);

        cutoff = __SSAT(lpCutoff - modLpCutoff, 28);
        lpL.update(cutoff, lpReso, lpDrive);
        lpR.update(cutoff, lpReso, lpDrive);

        // update time / read offsets
        if (!isFading && (time != this->time || offset != this->offset || bdur != this->bdur)) {
            this->time = time;
            this->offset = offset;
            this->bdur = bdur;

            fadeOutOffsetL = fadeInOffsetL;
            fadeInOffsetL = time2offset(time, length, bdur);

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

    inline int time2offset(int t, int length, int bdur = 0) {
        // time table: it increases exp (almost) first, afterwards linear up to 2 seconds.
        static const int TIMES_OLD[128] = {
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

        static const int TIMES[128] = {
            // start=0.0 (0.0ms) end=600.0 (12.5ms) inc=19.35484 (0.4032258ms) 
            0, 19, 38, 58, 77, 96, 116, 135, 154, 174, 193, 212, 232, 251, 270, 290, 
            309, 329, 348, 367, 387, 406, 425, 445, 464, 483, 503, 522, 541, 561, 580, 600, 

            // start=656.25 (13.671875ms) end=2400.0 (50.0ms) inc=56.25 (1.171875ms) 
            656, 712, 768, 825, 881, 937, 993, 1050, 1106, 1162, 1218, 1275, 1331, 1387, 1443, 1500, 
            1556, 1612, 1668, 1725, 1781, 1837, 1893, 1950, 2006, 2062, 2118, 2175, 2231, 2287, 2343, 2400, 

            // start=2625.0 (54.6875ms) end=9600.0 (200.0ms) inc=225.0 (4.6875ms) 
            2625, 2850, 3075, 3300, 3525, 3750, 3975, 4200, 4425, 4650, 4875, 5100, 5325, 5550, 5775, 6000, 
            6225, 6450, 6675, 6900, 7125, 7350, 7575, 7800, 8025, 8250, 8475, 8700, 8925, 9150, 9375, 9600, 

            // start=12300.0 (256.25ms) end=96000.0 (2000.0ms) inc=2700.0 (56.25ms) 
            12300, 15000, 17700, 20400, 23100, 25800, 28500, 31200, 33900, 36600, 39300, 42000, 44700, 47400, 50100, 52800, 
            55500, 58200, 60900, 63600, 66300, 69000, 71700, 74400, 77100, 79800, 82500, 85200, 87900, 90600, 93300, 96000,
        };

        // beat table: multiples of 1/64 notes
        static const int BEATS[] = {
            0, 1, 2, 4, 4, 8, 8, 12, 12, 16, 16, 20, 20, 24, 24, 32
        };

        if (bdur == 0) {
            t = (t >> (27 - 7)) & 0x7f;
            return std::min(TIMES[t], length - 1);
        } else {
            t = (t >> (27 - 4)) & 0x0f;

            // float duration64th = bdur / 16.0f / 1000.0f;
            // float duration = duration64th * BEATS[t];
            // int samples = (int)(duration * SAMPLERATE);
            // simplified to ints:
            int samples = (SAMPLERATE / 1000 * BEATS[t] * bdur) >> 4;
            return std::min(samples, length - 1);
        }
    }


    int* bufferL = nullptr;
    int* bufferR = nullptr;

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
    int bdur = 0;

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