#include <Arduino_DataBus.h>

#if defined(ESP32) && (CONFIG_IDF_TARGET_ESP32S3)

#ifndef _JCY_ESP32_RGB_PANEL_H_
#define _JCY_ESP32_RGB_PANEL_H_

#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_rgb.h"
#include "esp_lcd_panel_vendor.h"
#include "esp_lcd_panel_ops.h"
#include "esp32s3/rom/cache.h"
// This function is located in ROM (also see esp_rom/${target}/ld/${target}.rom.ld)
extern int Cache_WriteBack_Addr(uint32_t addr, uint32_t size);

class JcyEsp32RgbPanel : public Arduino_DataBus
{
public:
  // esp_lcd allocates two internal DMA bounce buffers. Keep this value in the
  // tracked product driver instead of an ignored vendor-library symlink.
#if defined(JCY_PANEL_B043_480X272) && JCY_PANEL_B043_480X272
  // esp_lcd requires frame pixels to be an exact multiple of bounce pixels.
  // Use 34 lines (272 / 8) to give PSRAM DMA more refill margin than the
  // original 16-line strip; underruns restart RGB scanning as vertical jumps.
  static constexpr size_t kBounceBufferLines = 34;
#elif defined(JCY_RGB_BOUNCE_LINES)
  static constexpr size_t kBounceBufferLines = JCY_RGB_BOUNCE_LINES;
#else
  static constexpr size_t kBounceBufferLines = 24;
#endif
  static constexpr size_t kBounceBufferBytesPerPanelLinePair(size_t width)
  {
    return width * kBounceBufferLines * sizeof(uint16_t) * 2U;
  }

  JcyEsp32RgbPanel(
      int8_t cs, int8_t sck, int8_t sda,
      int8_t de, int8_t vsync, int8_t hsync, int8_t pclk,
      int8_t r0, int8_t r1, int8_t r2, int8_t r3, int8_t r4,
      int8_t g0, int8_t g1, int8_t g2, int8_t g3, int8_t g4, int8_t g5,
      int8_t b0, int8_t b1, int8_t b2, int8_t b3, int8_t b4,
      bool useBigEndian = false);

  void begin(int32_t speed = GFX_NOT_DEFINED, int8_t dataMode = GFX_NOT_DEFINED) override;
  void beginWrite() override;
  void endWrite() override;
  void writeCommand(uint8_t) override;
  void writeCommand16(uint16_t) override;
  void write(uint8_t) override;
  void write16(uint16_t) override;
  void writeRepeat(uint16_t p, uint32_t len) override;
  void writePixels(uint16_t *data, uint32_t len) override;

  void writeBytes(uint8_t *data, uint32_t len) override;
  void writePattern(uint8_t *data, uint8_t len, uint32_t repeat) override;

  uint16_t *getFrameBuffer(
      uint16_t w, uint16_t h,
      uint16_t hsync_pulse_width = 18, uint16_t hsync_back_porch = 24, uint16_t hsync_front_porch = 6, uint16_t hsync_polarity = 1,
      uint16_t vsync_pulse_width = 10, uint16_t vsync_back_porch = 16, uint16_t vsync_front_porch = 4, uint16_t vsync_polarity = 1,
      uint16_t pclk_active_neg = 0, int32_t prefer_speed = GFX_NOT_DEFINED);
  uint16_t *getFrameBuffer(uint8_t index) const;
  bool submitFrame(uint16_t *framebuffer);

  // JCY1000-5: expose the RGB frame-complete ISR callback so LVGL can
  // synchronize the first dirty-buffer copy of each UI frame to VSYNC.
  void setFrameTransDoneCallback(
      esp_lcd_rgb_panel_frame_buf_complete_cb_t callback,
      void *user_ctx);

protected:
private:
  INLINE void CS_HIGH(void);
  INLINE void CS_LOW(void);
  INLINE void SCK_HIGH(void);
  INLINE void SCK_LOW(void);
  INLINE void SDA_HIGH(void);
  INLINE void SDA_LOW(void);

  int32_t _speed;
  int8_t _dataMode;
  int8_t _cs, _sck, _sda;
  int8_t _de, _vsync, _hsync, _pclk;
  int8_t _r0, _r1, _r2, _r3, _r4;
  int8_t _g0, _g1, _g2, _g3, _g4, _g5;
  int8_t _b0, _b1, _b2, _b3, _b4;
  bool _useBigEndian;

  esp_lcd_panel_handle_t _panel_handle = NULL;
  uint16_t *_framebuffers[2] = {NULL, NULL};
  uint16_t _frame_width = 0;
  uint16_t _frame_height = 0;

  PORTreg_t _csPortSet;  ///< PORT register for chip select SET
  PORTreg_t _csPortClr;  ///< PORT register for chip select CLEAR
  PORTreg_t _sckPortSet; ///< PORT register for SCK SET
  PORTreg_t _sckPortClr; ///< PORT register for SCK CLEAR
  PORTreg_t _sdaPortSet; ///< PORT register for SCK SET
  PORTreg_t _sdaPortClr; ///< PORT register for SCK CLEAR
  uint32_t _csPinMask;   ///< Bitmask for chip select
  uint32_t _sckPinMask;  ///< Bitmask for SCK
  uint32_t _sdaPinMask;  ///< Bitmask for SCK
};

#endif // _JCY_ESP32_RGB_PANEL_H_

#endif // #if defined(ESP32) && (CONFIG_IDF_TARGET_ESP32S3)
