#pragma once

#include <Arduino.h>

namespace jcy_audio {

enum class Cue : uint8_t {
    None = 0,
    Click,
    Adjust,
    Navigate,
    Confirm,
    Back,
    Toggle,
    Power,
    Wave,
};

// JC8048W550 / NS4168 fixed I2S wiring:
// GPIO0=BCLK, GPIO18=LRCLK, GPIO17=SDATA.
bool begin();
bool ready();
void play(Cue cue);

}  // namespace jcy_audio
