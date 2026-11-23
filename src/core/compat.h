// compat.h - portability macros for the portable core (src/core).
// OWNER: group A. SHARED CONTRACT - change only by agreement of all groups.
// Rule: nothing in src/core may include Arduino.h, M5*.h or any ESP-IDF header
// except through this file.
#pragma once
#include <stdint.h>
#include <stddef.h>

#if defined(ESP_PLATFORM) || defined(ARDUINO_ARCH_ESP32)
  #include "esp_attr.h"
  // Put on the DEFINITION (in the .cpp) of hot functions only, never on inline
  // header functions. Hot set: renderWorld, renderJet, expand2x, drawLine.
  #define JET_HOT IRAM_ATTR
  #define JET_DEVICE 1
#else
  #define JET_HOT
  #define JET_DEVICE 0
#endif

#if defined(__GNUC__) || defined(__clang__)
  #define JET_LIKELY(x)   __builtin_expect(!!(x), 1)
  #define JET_UNLIKELY(x) __builtin_expect(!!(x), 0)
  #define JET_NOINLINE    __attribute__((noinline))
#else
  #define JET_LIKELY(x)   (x)
  #define JET_UNLIKELY(x) (x)
  #define JET_NOINLINE    __declspec(noinline)
#endif

#ifndef JET_SIM
  #define JET_SIM 0
#endif

namespace kj {

inline uint16_t bswap16(uint16_t v) { return (uint16_t)((v >> 8) | (v << 8)); }

// RGB888 (0xRRGGBB) -> RGB565, NOT swapped.
inline uint16_t rgb565(uint32_t rgb) {
  uint32_t r = (rgb >> 16) & 0xFF, g = (rgb >> 8) & 0xFF, b = rgb & 0xFF;
  return (uint16_t)(((r & 0xF8) << 8) | ((g & 0xFC) << 3) | (b >> 3));
}
// RGB888 -> RGB565 byte-swapped (big-endian in memory) = what the ST7789 DMA path wants.
inline uint16_t rgb565sw(uint32_t rgb) { return bswap16(rgb565(rgb)); }

// Inverse used by the simulator: byte-swapped 565 -> RGB888 with bit replication.
inline uint32_t rgb565swToRgb888(uint16_t sw) {
  uint16_t c = bswap16(sw);
  uint32_t r = (c >> 11) & 31, g = (c >> 5) & 63, b = c & 31;
  r = (r << 3) | (r >> 2); g = (g << 2) | (g >> 4); b = (b << 3) | (b >> 2);
  return (r << 16) | (g << 8) | b;
}

}  // namespace kj
