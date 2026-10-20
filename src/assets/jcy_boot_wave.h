#pragma once

#include <stddef.h>
#include <stdint.h>

namespace jcy_boot_wave {

constexpr uint32_t sample_rate = 22050;
constexpr uint32_t duration_ms = 1200;
extern const int16_t samples[];
extern const size_t sample_count;

}  // namespace jcy_boot_wave
