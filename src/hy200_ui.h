#pragma once

#include <Arduino.h>
#include <lvgl.h>

#include "hy200_protocol.h"

namespace hy200_ui {

void begin(hy200::Protocol *protocol, const char *panel_version);
void loop(uint32_t now_ms);
void debug_key_left();
void debug_key_right();
void debug_key_confirm();
bool active();

}  // namespace hy200_ui
