#include "jcy_audio.h"
#include "assets/jcy_boot_wave.h"

#include <ESP_I2S.h>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <freertos/task.h>

#include <algorithm>
#include <cmath>
#include <cstring>

namespace jcy_audio {
namespace {

constexpr int I2S_BCLK_PIN = 0;
constexpr int I2S_LRCLK_PIN = 18;
constexpr int I2S_DATA_PIN = 17;
constexpr uint32_t SAMPLE_RATE = 44100;
constexpr size_t FRAMES_PER_BLOCK = 192;
constexpr size_t CUE_QUEUE_DEPTH = 5;
constexpr float PI_F = 3.14159265358979323846f;

I2SClass speaker_i2s;
QueueHandle_t cue_queue = nullptr;
TaskHandle_t audio_task_handle = nullptr;
bool audio_ready = false;
float phase = 0.0f;
int16_t stereo_samples[FRAMES_PER_BLOCK * 2] = {};

void write_frames(const int16_t *samples, size_t frames) {
    const uint8_t *data = reinterpret_cast<const uint8_t *>(samples);
    size_t remaining = frames * 2 * sizeof(int16_t);
    while (remaining > 0) {
        const size_t written = speaker_i2s.write(data, remaining);
        if (written == 0) {
            taskYIELD();
            continue;
        }
        data += written;
        remaining -= written;
    }
}

void play_sweep(float start_hz, float end_hz, uint32_t duration_ms,
                int16_t amplitude) {
    const uint32_t total_frames = std::max<uint32_t>(
        1, static_cast<uint32_t>(
               (static_cast<uint64_t>(SAMPLE_RATE) * duration_ms) / 1000U));
    const uint32_t attack_frames = std::max<uint32_t>(1, SAMPLE_RATE / 500U);
    const uint32_t release_frames = std::max<uint32_t>(1, SAMPLE_RATE / 300U);
    uint32_t frames_done = 0;

    while (frames_done < total_frames) {
        const size_t frames = std::min<size_t>(
            FRAMES_PER_BLOCK, total_frames - frames_done);
        for (size_t i = 0; i < frames; ++i) {
            const uint32_t frame = frames_done + i;
            const float progress = total_frames > 1
                                       ? static_cast<float>(frame) /
                                             static_cast<float>(total_frames - 1)
                                       : 0.0f;
            const float frequency =
                start_hz + (end_hz - start_hz) * progress;
            float envelope = 1.0f;
            if (frame < attack_frames) {
                envelope = static_cast<float>(frame) /
                           static_cast<float>(attack_frames);
            }
            const uint32_t remaining = total_frames - frame;
            if (remaining < release_frames) {
                envelope = std::min(
                    envelope,
                    static_cast<float>(remaining) /
                        static_cast<float>(release_frames));
            }
            const int16_t sample = static_cast<int16_t>(
                sinf(phase) * static_cast<float>(amplitude) * envelope);
            stereo_samples[i * 2] = sample;
            stereo_samples[i * 2 + 1] = sample;
            phase += 2.0f * PI_F * frequency /
                     static_cast<float>(SAMPLE_RATE);
            if (phase >= 2.0f * PI_F) {
                phase -= 2.0f * PI_F;
            }
        }
        write_frames(stereo_samples, frames);
        frames_done += frames;
    }
}

void play_silence(uint32_t duration_ms) {
    memset(stereo_samples, 0, sizeof(stereo_samples));
    const uint32_t total_frames =
        static_cast<uint32_t>(
            (static_cast<uint64_t>(SAMPLE_RATE) * duration_ms) / 1000U);
    uint32_t frames_done = 0;
    while (frames_done < total_frames) {
        const size_t frames = std::min<size_t>(
            FRAMES_PER_BLOCK, total_frames - frames_done);
        write_frames(stereo_samples, frames);
        frames_done += frames;
    }
}

void play_recorded_wave() {
    static_assert(SAMPLE_RATE == jcy_boot_wave::sample_rate * 2U,
                  "Recorded wave sample-rate conversion expects exact 2x");
    constexpr size_t SOURCE_SAMPLES_PER_BLOCK = FRAMES_PER_BLOCK / 2U;
    size_t source_index = 0;

    while (source_index < jcy_boot_wave::sample_count) {
        const size_t source_samples = std::min<size_t>(
            SOURCE_SAMPLES_PER_BLOCK,
            jcy_boot_wave::sample_count - source_index);
        for (size_t i = 0; i < source_samples; ++i) {
            const int16_t sample =
                jcy_boot_wave::samples[source_index + i];
            const size_t output = i * 4U;
            // Duplicate mono 22050 Hz samples to 44100 Hz stereo without
            // changing pitch or allocating another large audio buffer.
            stereo_samples[output] = sample;
            stereo_samples[output + 1U] = sample;
            stereo_samples[output + 2U] = sample;
            stereo_samples[output + 3U] = sample;
        }
        write_frames(stereo_samples, source_samples * 2U);
        source_index += source_samples;
    }
}

void play_cue(Cue cue) {
    switch (cue) {
        case Cue::Click:
            play_sweep(1850.0f, 1450.0f, 24, 4300);
            break;
        case Cue::Adjust:
            play_sweep(2450.0f, 2050.0f, 18, 3900);
            break;
        case Cue::Navigate:
            play_sweep(900.0f, 1250.0f, 28, 4300);
            play_silence(8);
            play_sweep(1350.0f, 1650.0f, 30, 4300);
            break;
        case Cue::Confirm:
            play_sweep(1050.0f, 1700.0f, 55, 4700);
            break;
        case Cue::Back:
            play_sweep(1350.0f, 720.0f, 48, 4300);
            break;
        case Cue::Toggle:
            play_sweep(820.0f, 1280.0f, 38, 4500);
            break;
        case Cue::Power:
            play_sweep(620.0f, 980.0f, 34, 4800);
            play_silence(8);
            play_sweep(1180.0f, 1750.0f, 46, 4800);
            break;
        case Cue::Wave:
            play_recorded_wave();
            break;
        case Cue::None:
            return;
    }
    play_silence(12);
}

void audio_task(void *) {
    for (;;) {
        uint8_t raw_cue = 0;
        if (xQueueReceive(cue_queue, &raw_cue, portMAX_DELAY) == pdTRUE) {
            play_cue(static_cast<Cue>(raw_cue));
        }
    }
}

}  // namespace

bool begin() {
    if (audio_ready) {
        return true;
    }

    cue_queue = xQueueCreate(CUE_QUEUE_DEPTH, sizeof(uint8_t));
    if (cue_queue == nullptr) {
        Serial.println("AUDIO: cue queue allocation failed");
        return false;
    }

    speaker_i2s.setPins(I2S_BCLK_PIN, I2S_LRCLK_PIN, I2S_DATA_PIN);
    if (!speaker_i2s.begin(I2S_MODE_STD, SAMPLE_RATE,
                           I2S_DATA_BIT_WIDTH_16BIT,
                           I2S_SLOT_MODE_STEREO)) {
        Serial.printf("AUDIO: I2S init failed error=%d\n",
                      speaker_i2s.lastError());
        vQueueDelete(cue_queue);
        cue_queue = nullptr;
        return false;
    }

    if (xTaskCreatePinnedToCore(audio_task, "jcy_audio", 4096, nullptr, 1,
                                &audio_task_handle, 0) != pdPASS) {
        Serial.println("AUDIO: task creation failed");
        speaker_i2s.end();
        vQueueDelete(cue_queue);
        cue_queue = nullptr;
        return false;
    }

    audio_ready = true;
    Serial.println("AUDIO: NS4168 ready BCLK=0 LRCLK=18 DATA=17");
    return true;
}

bool ready() {
    return audio_ready;
}

void play(Cue cue) {
    if (!audio_ready || cue == Cue::None || cue_queue == nullptr) {
        return;
    }
    const uint8_t raw_cue = static_cast<uint8_t>(cue);
    if (xQueueSend(cue_queue, &raw_cue, 0) == pdTRUE) {
        return;
    }

    // Rapid tapping should sound current, not replay a long stale queue.
    uint8_t dropped = 0;
    xQueueReceive(cue_queue, &dropped, 0);
    xQueueSend(cue_queue, &raw_cue, 0);
}

}  // namespace jcy_audio
