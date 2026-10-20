#pragma once

// Keep the existing JC8048W550 build as the default.  The SH051 profile is a
// separate PlatformIO environment because its exact RGB porch timing is not
// included in the supplied module specification and still needs panel-side
// validation.
#ifndef JCY_PANEL_SH051_1024X600
#define JCY_PANEL_SH051_1024X600 0
#endif
#ifndef JCY_PANEL_B043_480X272
#define JCY_PANEL_B043_480X272 0
#endif

namespace jcy_panel {

#if JCY_PANEL_B043_480X272

#if defined(JCY_HY200C_UI) && JCY_HY200C_UI
constexpr int kUiWidth = 480;
constexpr int kUiHeight = 272;
#else
constexpr int kUiWidth = 800;
constexpr int kUiHeight = 480;
#endif

constexpr const char *kModel = "B043K124C-07A";
constexpr int kPanelWidth = 480;
constexpr int kPanelHeight = 272;
constexpr int kContentWidth = 480;
constexpr int kContentHeight = 272;
constexpr int kContentX = 0;
constexpr int kContentY = 0;
// Datasheet: 480 active clocks plus 40..320 blanking (45 typical);
// 272 active lines plus 5..128 blanking (16 typical).  Split the typical
// blanking conservatively for DE mode. Typical pixel clock is 9 MHz.
constexpr int kHsyncPolarity = 0;
constexpr int kHsyncFrontPorch = 2;
constexpr int kHsyncPulseWidth = 41;
constexpr int kHsyncBackPorch = 2;
constexpr int kVsyncPolarity = 0;
constexpr int kVsyncFrontPorch = 2;
constexpr int kVsyncPulseWidth = 10;
constexpr int kVsyncBackPorch = 4;
constexpr int kPclkActiveNeg = 0;
constexpr int kPclkHz = 9000000;

#elif JCY_PANEL_SH051_1024X600

constexpr const char *kModel = "SH051ZJWS1KL-01TBA-5102";
// The existing JCY2001D UI and its bitmap assets are authored at 800x480.
// Keep that logical canvas and scale the completed LVGL output to the native
// 1024x600 panel.  Declaring a 1024x600 logical canvas makes LVGL tile the
// 800x480 background image and leaves the legacy control coordinates confined
// to only part of the screen.
constexpr int kUiWidth = 800;
constexpr int kUiHeight = 480;
constexpr int kPanelWidth = 1024;
constexpr int kPanelHeight = 600;
constexpr int kContentWidth = 1024;
constexpr int kContentHeight = 600;
constexpr int kContentX = 0;
constexpr int kContentY = 0;
// Vendor/test-fixture geometry: H 1024+24+136+160; V 600+2+21+12.
// 32 MHz remains within the ESP32-S3 cache-safe RGB bandwidth.
constexpr int kHsyncPolarity = 0;
constexpr int kHsyncFrontPorch = 160;
constexpr int kHsyncPulseWidth = 24;
constexpr int kHsyncBackPorch = 136;
constexpr int kVsyncPolarity = 0;
constexpr int kVsyncFrontPorch = 12;
constexpr int kVsyncPulseWidth = 2;
constexpr int kVsyncBackPorch = 21;
constexpr int kPclkActiveNeg = 1;
constexpr int kPclkHz = 26000000;

#else

constexpr int kUiWidth = 800;
constexpr int kUiHeight = 480;
constexpr const char *kModel = "JC8048W550C";
constexpr int kPanelWidth = 800;
constexpr int kPanelHeight = 480;
constexpr int kContentWidth = 800;
constexpr int kContentHeight = 480;
constexpr int kContentX = 0;
constexpr int kContentY = 0;
constexpr int kHsyncPolarity = 0;
constexpr int kHsyncFrontPorch = 8;
constexpr int kHsyncPulseWidth = 4;
constexpr int kHsyncBackPorch = 8;
constexpr int kVsyncPolarity = 0;
constexpr int kVsyncFrontPorch = 8;
constexpr int kVsyncPulseWidth = 4;
constexpr int kVsyncBackPorch = 8;
constexpr int kPclkActiveNeg = 1;
constexpr int kPclkHz = 12500000;

#endif

constexpr bool kScaledUi =
    kPanelWidth != kUiWidth || kPanelHeight != kUiHeight ||
    kContentX != 0 || kContentY != 0;
constexpr bool kDownscaledUi =
    kContentWidth < kUiWidth || kContentHeight < kUiHeight;

static_assert(kContentWidth <= kPanelWidth, "UI viewport wider than panel");
static_assert(kContentHeight <= kPanelHeight, "UI viewport taller than panel");
static_assert(kContentX >= 0 && kContentY >= 0,
              "UI viewport offset must be non-negative");
static_assert(kContentX + kContentWidth <= kPanelWidth,
              "UI viewport exceeds panel width");
static_assert(kContentY + kContentHeight <= kPanelHeight,
              "UI viewport exceeds panel height");

}  // namespace jcy_panel
