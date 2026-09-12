/**
 * @file pinprobe_main.cpp
 * @brief X4 Pro VBUS/USB-detect pin hunt — standalone diagnostic firmware.
 *
 * Built ONLY by the `x4pro_pinprobe` env (normal envs exclude src/probe/).
 * Goal: find the GPIO that follows charger/USB VBUS presence so
 * BoardConfig::XTEINK_X4_PRO can get a real `usbDetect` pin and
 * WakeupReason::AfterUSBPower stops being dead code (see
 * freeink-sdk/docs/xteink-x4pro-support.md#rtc--usb--battery — "The VBUS/USB-detect
 * pin remains not conclusively identified").
 *
 * Method: every candidate pin is sampled under INPUT_PULLDOWN and then
 * INPUT_PULLUP. A pin that reads the same level under both pulls is externally
 * DRIVEN; one that follows the pull is floating. A pin whose driven-level tracks
 * plug/unplug of a charger is the detect line. ADC-capable candidates are also
 * sampled raw, because the Sticky senses VBUS as an analog divider
 * (PWR_IN_VOLT) and the X4 Pro may do the same.
 *
 * The live table is painted on the e-paper panel because serial-over-USB dies
 * with the cable — the cable IS the experiment. Test with a dumb wall charger,
 * not just a computer: HWCDC only sees data-capable hosts.
 *
 * Candidates deliberately EXCLUDED: display SPI 12/11/13/18/14/6, SDMMC
 * 41/42/40, SD enable 5, buttons 0/7/3, I2C 39/38, touch RST 4 / power rail 2,
 * frontlight 8/9, native USB D-/D+ 19/20 (support doc: never probe), master
 * rail latch 1, flash 26-32, octal PSRAM 33-37.
 */

#if FREEINK_DEVICE_X4PRO

#include <Arduino.h>
#include <BoardConfig.h>
#include <EInkDisplay.h>
#include <XteinkDetect.h>
#include <esp_attr.h>
#include <esp_system.h>

#include <cstdarg>
#include <cstdio>
#include <cstring>

// The inherited -DFREEINK_UC8279_X4_CONFIG=x4proPanelConfig flag makes the UC8279
// driver reference freeink::x4proPanelConfig(), defined in lib/hal_x4pro. This env
// never includes HAL headers, so name one here purely to make the LDF link that
// library (the linker then pulls X4ProPanelConfig.o from its archive).
#include "../../lib/hal_x4pro/X4ProGrayScale.h"

namespace {

// ---------------------------------------------------------------------------
// Pins under test
// ---------------------------------------------------------------------------

struct ProbePin {
  uint8_t pin;
  bool adc;         // also sample analogRead (S3: ADC1 = GPIO1-10, ADC2 = GPIO11-20)
  const char* tag;  // short note shown in the table (uppercase only — see font)
};

constexpr ProbePin kProbes[] = {
    {10, true, "TP-INT"},   // GT911 INT (touch stays unpowered here); the stale usbDetect candidate
    {15, true, ""},
    {16, true, ""},
    {17, true, ""},
    {21, false, "CHGSTAT"},  // charger STAT, active-high = charging; test with a FULL battery too
    {43, false, "U0TX"},     // UART0 pads — console is HWCDC, so these are free to read
    {44, false, "U0RX"},
    {45, false, "SD-CS"},    // SPI-view SD CS, unused in SDMMC mode; no SD is mounted here
    {46, false, "STRAP"},
    {47, false, ""},
    {48, false, ""},
};
constexpr size_t kProbeCount = sizeof(kProbes) / sizeof(kProbes[0]);

struct PinState {
  bool driven = false;
  bool level = false;
  uint16_t adcRaw = 0;
  uint16_t changes = 0;
  unsigned long lastChangeMs = 0;
  // debounce: a change is accepted only after two identical consecutive samples
  bool pendingDriven = false;
  bool pendingLevel = false;
  uint8_t pendingCount = 0;
  uint16_t lastEventAdc = 0;
  bool primed = false;  // first sample seeds state without logging an event
};
PinState g_state[kProbeCount];

// ADC steps smaller than this are noise, not a VBUS divider appearing.
constexpr uint16_t kAdcEventDelta = 300;

// ---------------------------------------------------------------------------
// Boot bookkeeping — survives soft resets, so a mid-test reset is visible
// ---------------------------------------------------------------------------

constexpr uint32_t kBootMagic = 0x50524F42;  // "PROB"
RTC_NOINIT_ATTR uint32_t g_bootMagic;
RTC_NOINIT_ATTR uint32_t g_bootCount;

const char* resetReasonName(const esp_reset_reason_t r) {
  switch (r) {
    case ESP_RST_POWERON: return "POWERON";
    case ESP_RST_SW: return "SW";
    case ESP_RST_PANIC: return "PANIC";
    case ESP_RST_INT_WDT: return "IWDT";
    case ESP_RST_TASK_WDT: return "TWDT";
    case ESP_RST_WDT: return "WDT";
    case ESP_RST_DEEPSLEEP: return "DSLEEP";
    case ESP_RST_BROWNOUT: return "BROWNOUT";
    case ESP_RST_SDIO: return "SDIO";
    case ESP_RST_UNKNOWN: return "UNKNOWN";
    default: return "OTHER";
  }
}

// ---------------------------------------------------------------------------
// Event ring — the last few transitions, painted under the table
// ---------------------------------------------------------------------------

struct Event {
  char text[40];
};
constexpr size_t kEventMax = 5;
Event g_events[kEventMax];
size_t g_eventNext = 0;
size_t g_eventCount = 0;

void pushEvent(const char* fmt, ...) {
  Event& e = g_events[g_eventNext];
  va_list args;
  va_start(args, fmt);
  vsnprintf(e.text, sizeof(e.text), fmt, args);
  va_end(args);
  g_eventNext = (g_eventNext + 1) % kEventMax;
  if (g_eventCount < kEventMax) ++g_eventCount;
  Serial.printf("[PINPROBE][EVENT] %s\n", e.text);
}

// ---------------------------------------------------------------------------
// Tiny built-in 5x7 font (column bytes, bit0 = top row) — no font stack needed
// ---------------------------------------------------------------------------

constexpr char kFontChars[] = "0123456789ABCDEFGHIJKLMNOPQRSTUVWXYZ -.:=+>*?/()";
constexpr uint8_t kFontCols[][5] = {
    {0x3E, 0x51, 0x49, 0x45, 0x3E}, {0x00, 0x42, 0x7F, 0x40, 0x00}, {0x42, 0x61, 0x51, 0x49, 0x46},
    {0x21, 0x41, 0x45, 0x4B, 0x31}, {0x18, 0x14, 0x12, 0x7F, 0x10}, {0x27, 0x45, 0x45, 0x45, 0x39},
    {0x3C, 0x4A, 0x49, 0x49, 0x30}, {0x01, 0x71, 0x09, 0x05, 0x03}, {0x36, 0x49, 0x49, 0x49, 0x36},
    {0x06, 0x49, 0x49, 0x29, 0x1E}, {0x7E, 0x11, 0x11, 0x11, 0x7E}, {0x7F, 0x49, 0x49, 0x49, 0x36},
    {0x3E, 0x41, 0x41, 0x41, 0x22}, {0x7F, 0x41, 0x41, 0x22, 0x1C}, {0x7F, 0x49, 0x49, 0x49, 0x41},
    {0x7F, 0x09, 0x09, 0x09, 0x01}, {0x3E, 0x41, 0x49, 0x49, 0x7A}, {0x7F, 0x08, 0x08, 0x08, 0x7F},
    {0x00, 0x41, 0x7F, 0x41, 0x00}, {0x20, 0x40, 0x41, 0x3F, 0x01}, {0x7F, 0x08, 0x14, 0x22, 0x41},
    {0x7F, 0x40, 0x40, 0x40, 0x40}, {0x7F, 0x02, 0x0C, 0x02, 0x7F}, {0x7F, 0x04, 0x08, 0x10, 0x7F},
    {0x3E, 0x41, 0x41, 0x41, 0x3E}, {0x7F, 0x09, 0x09, 0x09, 0x06}, {0x3E, 0x41, 0x51, 0x21, 0x5E},
    {0x7F, 0x09, 0x19, 0x29, 0x46}, {0x46, 0x49, 0x49, 0x49, 0x31}, {0x01, 0x01, 0x7F, 0x01, 0x01},
    {0x3F, 0x40, 0x40, 0x40, 0x3F}, {0x1F, 0x20, 0x40, 0x20, 0x1F}, {0x3F, 0x40, 0x38, 0x40, 0x3F},
    {0x63, 0x14, 0x08, 0x14, 0x63}, {0x07, 0x08, 0x70, 0x08, 0x07}, {0x61, 0x51, 0x49, 0x45, 0x43},
    {0x00, 0x00, 0x00, 0x00, 0x00}, {0x08, 0x08, 0x08, 0x08, 0x08}, {0x00, 0x60, 0x60, 0x00, 0x00},
    {0x00, 0x36, 0x36, 0x00, 0x00}, {0x14, 0x14, 0x14, 0x14, 0x14}, {0x08, 0x08, 0x3E, 0x08, 0x08},
    {0x00, 0x41, 0x22, 0x14, 0x08}, {0x14, 0x08, 0x3E, 0x08, 0x14}, {0x02, 0x01, 0x51, 0x09, 0x06},
    {0x20, 0x10, 0x08, 0x04, 0x02}, {0x00, 0x1C, 0x22, 0x41, 0x00}, {0x00, 0x41, 0x22, 0x1C, 0x00},
};

const uint8_t* glyphFor(char c) {
  if (c >= 'a' && c <= 'z') c = static_cast<char>(c - 'a' + 'A');
  const char* hit = strchr(kFontChars, c);
  if (hit == nullptr || c == '\0') return kFontCols[strchr(kFontChars, '?') - kFontChars];
  return kFontCols[hit - kFontChars];
}

// ---------------------------------------------------------------------------
// Framebuffer text rendering (landscape-native 800x480, 1bpp MSB-first, 1=white)
// ---------------------------------------------------------------------------

EInkDisplay* g_display = nullptr;

constexpr int kScale = 3;
constexpr int kCharW = 6 * kScale;  // 5 columns + 1 gap
constexpr int kLineH = 8 * kScale;  // 7 rows + 1 leading
constexpr int kLeft = 8;

void inkPixel(uint8_t* fb, const int x, const int y) {
  if (x < 0 || x >= EInkDisplay::DISPLAY_WIDTH || y < 0 || y >= EInkDisplay::DISPLAY_HEIGHT) return;
  fb[y * EInkDisplay::DISPLAY_WIDTH_BYTES + x / 8] &= static_cast<uint8_t>(~(0x80 >> (x % 8)));
}

void drawChar(uint8_t* fb, const int x, const int y, const char c) {
  const uint8_t* cols = glyphFor(c);
  for (int cx = 0; cx < 5; ++cx) {
    for (int cy = 0; cy < 7; ++cy) {
      if ((cols[cx] >> cy) & 1) {
        for (int sx = 0; sx < kScale; ++sx) {
          for (int sy = 0; sy < kScale; ++sy) {
            inkPixel(fb, x + cx * kScale + sx, y + cy * kScale + sy);
          }
        }
      }
    }
  }
}

void drawText(uint8_t* fb, int x, const int y, const char* text) {
  for (const char* c = text; *c != '\0'; ++c) {
    if (*c != ' ') drawChar(fb, x, y, *c);
    x += kCharW;
    if (x + kCharW > EInkDisplay::DISPLAY_WIDTH) break;
  }
}

// ---------------------------------------------------------------------------
// Sampling
// ---------------------------------------------------------------------------

bool g_usbPlugged = false;

void samplePin(const size_t i) {
  const ProbePin& probe = kProbes[i];
  PinState& st = g_state[i];

  pinMode(probe.pin, INPUT_PULLDOWN);
  delayMicroseconds(300);
  const bool pulledDown = digitalRead(probe.pin) == HIGH;
  pinMode(probe.pin, INPUT_PULLUP);
  delayMicroseconds(300);
  const bool pulledUp = digitalRead(probe.pin) == HIGH;
  pinMode(probe.pin, INPUT);

  const bool driven = pulledDown == pulledUp;
  const bool level = pulledDown;

  if (probe.adc) {
    st.adcRaw = static_cast<uint16_t>(analogRead(probe.pin));
  }

  if (!st.primed) {
    st.primed = true;
    st.driven = driven;
    st.level = level;
    st.lastEventAdc = st.adcRaw;
    return;
  }

  if (driven != st.driven || (driven && level != st.level)) {
    if (st.pendingCount > 0 && driven == st.pendingDriven && level == st.pendingLevel) {
      ++st.pendingCount;
    } else {
      st.pendingDriven = driven;
      st.pendingLevel = level;
      st.pendingCount = 1;
    }
    if (st.pendingCount >= 2) {
      const unsigned long secs = millis() / 1000;
      char from[6];
      char to[6];
      snprintf(from, sizeof(from), st.driven ? "%d" : "FLT", st.level ? 1 : 0);
      snprintf(to, sizeof(to), driven ? "%d" : "FLT", level ? 1 : 0);
      pushEvent("T+%04luS P%u %s>%s", secs, probe.pin, from, to);
      st.driven = driven;
      st.level = level;
      ++st.changes;
      st.lastChangeMs = millis();
      st.pendingCount = 0;
    }
  } else {
    st.pendingCount = 0;
  }

  if (probe.adc) {
    const int delta = static_cast<int>(st.adcRaw) - static_cast<int>(st.lastEventAdc);
    if (delta > kAdcEventDelta || delta < -static_cast<int>(kAdcEventDelta)) {
      pushEvent("T+%04luS P%u ADC %u>%u", millis() / 1000, probe.pin, st.lastEventAdc, st.adcRaw);
      st.lastEventAdc = st.adcRaw;
      ++st.changes;
      st.lastChangeMs = millis();
    }
  }
}

// Returns true when anything changed since the previous call (drives redraws).
bool sampleAll() {
  static uint16_t lastTotalChanges = 0;
  static bool lastUsb = false;

  for (size_t i = 0; i < kProbeCount; ++i) samplePin(i);

  g_usbPlugged = HWCDC::isPlugged();
  if (g_usbPlugged != lastUsb) {
    pushEvent("T+%04luS USB-CDC %s", millis() / 1000, g_usbPlugged ? "PLUG" : "UNPLUG");
    lastUsb = g_usbPlugged;
  }

  uint16_t totalChanges = 0;
  for (size_t i = 0; i < kProbeCount; ++i) totalChanges += g_state[i].changes;
  const bool dirty = totalChanges != lastTotalChanges;
  lastTotalChanges = totalChanges;
  return dirty;
}

// ---------------------------------------------------------------------------
// Screen
// ---------------------------------------------------------------------------

void renderScreen() {
  uint8_t* fb = g_display->getFrameBuffer();
  if (fb == nullptr) return;
  memset(fb, 0xFF, EInkDisplay::BUFFER_SIZE);

  char line[48];
  int y = 4;

  snprintf(line, sizeof(line), "X4PRO PIN PROBE  RST=%s  BOOT=%lu",
           resetReasonName(esp_reset_reason()), static_cast<unsigned long>(g_bootCount));
  drawText(fb, kLeft, y, line);
  y += kLineH;

  snprintf(line, sizeof(line), "UP=%05luS  USB-CDC=%c", millis() / 1000, g_usbPlugged ? 'Y' : 'N');
  drawText(fb, kLeft, y, line);
  y += kLineH + kLineH / 2;

  drawText(fb, kLeft, y, "PIN  DRV LVL   ADC  CHG  LAST  NOTE");
  y += kLineH;

  for (size_t i = 0; i < kProbeCount; ++i) {
    const ProbePin& probe = kProbes[i];
    const PinState& st = g_state[i];

    char adcText[6] = "   -";
    if (probe.adc) snprintf(adcText, sizeof(adcText), "%4u", st.adcRaw);

    char lastText[6] = "   -";
    if (st.changes > 0) {
      snprintf(lastText, sizeof(lastText), "%4lu", (millis() - st.lastChangeMs) / 1000);
    }

    snprintf(line, sizeof(line), "P%-3u  %c   %c  %s  %3u  %s  %s", probe.pin,
             st.driven ? 'Y' : 'N', st.driven ? (st.level ? '1' : '0') : '-', adcText,
             st.changes > 999 ? 999 : st.changes, lastText, probe.tag);
    drawText(fb, kLeft, y, line);
    y += kLineH;
  }

  y += kLineH / 2;
  for (size_t i = 0; i < g_eventCount; ++i) {
    // newest last: walk the ring from oldest retained entry
    const size_t idx = (g_eventNext + kEventMax - g_eventCount + i) % kEventMax;
    drawText(fb, kLeft, y, g_events[idx].text);
    y += kLineH;
  }
  if (g_eventCount == 0) drawText(fb, kLeft, y, "NO TRANSITIONS YET - PLUG THE CHARGER");
}

void logTableToSerial() {
  Serial.printf("[PINPROBE] up=%lus usb=%d |", millis() / 1000, g_usbPlugged ? 1 : 0);
  for (size_t i = 0; i < kProbeCount; ++i) {
    const PinState& st = g_state[i];
    if (kProbes[i].adc) {
      Serial.printf(" P%u=%s/%u", kProbes[i].pin, st.driven ? (st.level ? "1" : "0") : "F", st.adcRaw);
    } else {
      Serial.printf(" P%u=%s", kProbes[i].pin, st.driven ? (st.level ? "1" : "0") : "F");
    }
  }
  Serial.println();
}

}  // namespace

void setup() {
  Serial.begin(115200);
  delay(50);

  if (g_bootMagic != kBootMagic) {
    g_bootMagic = kBootMagic;
    g_bootCount = 0;
  }
  ++g_bootCount;

  Serial.printf("[PINPROBE] boot #%lu reset=%s\n", static_cast<unsigned long>(g_bootCount),
                resetReasonName(esp_reset_reason()));

  BoardConfig::holdPowerRails();
  freeink::applyXteinkDisplayController();

  const auto& pins = BoardConfig::ACTIVE.display;
  g_display = new EInkDisplay(pins.sclk, pins.mosi, pins.cs, pins.dc, pins.rst, pins.busy);
  g_display->begin();

  pushEvent("T+0000S BOOT %lu RST=%s", static_cast<unsigned long>(g_bootCount),
            resetReasonName(esp_reset_reason()));

  sampleAll();
  renderScreen();
  g_display->displayBuffer(EInkDisplay::FULL_REFRESH);
}

void loop() {
  static unsigned long lastRenderMs = 0;
  static unsigned long lastSerialMs = 0;
  static bool dirty = false;
  static uint16_t partialRenders = 0;

  dirty = sampleAll() || dirty;

  const unsigned long now = millis();
  const bool heartbeat = now - lastRenderMs >= 10000;  // keep uptime moving as liveness proof
  if ((dirty && now - lastRenderMs >= 800) || heartbeat) {
    renderScreen();
    if (dirty || partialRenders >= 20) {
      // transitions get a fresh full pass so the row is unambiguous; the periodic
      // full also purges fast-refresh ghosting
      g_display->displayBuffer(partialRenders >= 20 || dirty ? EInkDisplay::FULL_REFRESH
                                                             : EInkDisplay::FAST_REFRESH);
      partialRenders = 0;
    } else {
      g_display->displayBuffer(EInkDisplay::FAST_REFRESH);
      ++partialRenders;
    }
    lastRenderMs = now;
    dirty = false;
  }

  if (now - lastSerialMs >= 5000) {
    logTableToSerial();
    lastSerialMs = now;
  }

  delay(100);
}

#endif  // FREEINK_DEVICE_X4PRO
