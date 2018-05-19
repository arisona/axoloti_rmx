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
static const int MINUS_ONE = -(1 << 27);

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

} // namespace rmx
