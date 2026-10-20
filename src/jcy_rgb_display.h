#include <Arduino_DataBus.h>

#if defined(ESP32) && (CONFIG_IDF_TARGET_ESP32S3)

#ifndef _JCY_RPI_DPI_RGB_PANEL_H_
#define _JCY_RPI_DPI_RGB_PANEL_H_

#include <Arduino_GFX.h>
#include "jcy_rgb_bus.h"

class JcyRpiDpiRgbPanel : public Arduino_GFX
{
public:
  JcyRpiDpiRgbPanel(
      JcyEsp32RgbPanel *bus,
      int16_t w, uint16_t hsync_polarity, uint16_t hsync_front_porch, uint16_t hsync_pulse_width, uint16_t hsync_back_porch,
      int16_t h, uint16_t vsync_polarity, uint16_t vsync_front_porch, uint16_t vsync_pulse_width, uint16_t vsync_back_porch,
      uint16_t pclk_active_neg = 0, int32_t prefer_speed = GFX_NOT_DEFINED, bool auto_flush = true);

  void begin(int32_t speed = GFX_NOT_DEFINED) override;
  void writePixelPreclipped(int16_t x, int16_t y, uint16_t color) override;
  void writeFastVLine(int16_t x, int16_t y, int16_t h, uint16_t color) override;
  void writeFastHLine(int16_t x, int16_t y, int16_t w, uint16_t color) override;
  void writeFillRectPreclipped(int16_t x, int16_t y, int16_t w, int16_t h, uint16_t color) override;
  void draw16bitRGBBitmap(int16_t x, int16_t y, uint16_t *bitmap, int16_t w, int16_t h) override;
  void draw16bitBeRGBBitmap(int16_t x, int16_t y, uint16_t *bitmap, int16_t w, int16_t h) override;
  void flush(void) override;

  uint16_t *getFramebuffer();
  uint16_t *getBackFramebuffer();
  bool submitFrame();

protected:
  uint16_t *_framebuffer;
  uint16_t *_frontbuffer;
  uint16_t *_backbuffer;
  size_t _framebuffer_size;
  JcyEsp32RgbPanel *_bus;

  uint16_t _hsync_polarity;
  uint16_t _hsync_front_porch;
  uint16_t _hsync_pulse_width;
  uint16_t _hsync_back_porch;
  uint16_t _vsync_polarity;
  uint16_t _vsync_front_porch;
  uint16_t _vsync_pulse_width;
  uint16_t _vsync_back_porch;
  uint16_t _pclk_active_neg;
  int32_t _prefer_speed;
  bool _auto_flush;

private:
};

#endif // _JCY_RPI_DPI_RGB_PANEL_H_

#endif // #if defined(ESP32) && (CONFIG_IDF_TARGET_ESP32S3)
