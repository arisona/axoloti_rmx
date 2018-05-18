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

class MidiClock final {
public:
    MidiClock() {}

    int process() {
        time++;
        // check for lost midi clock
        if (time - clockTime > CONTROLRATE)
            bdur = 0;
        return bdur;
    }

    void processMidi(midi_device_t device, uint8_t port, uint8_t status, uint8_t data1, uint8_t data2) {
        if (status != MIDI_TIMING_CLOCK)
            return;
        unsigned int delta = time - clockTime;
        clockTime = time;

        // bdur: duration of one beat in ms. calculate from 24ppq
        if (delta > 0) {
            bdur = 24 * 1000 / CONTROLRATE * delta;
        } else {
            bdur = 0;
        }
        bdur = average.process(bdur);
    }

private:
    void processClock() {

    }

    unsigned int time = 0;
    unsigned int clockTime = 0;

    MovingAverage<int, 16> average;

    int bdur = 0;
};

} // namespace rmx