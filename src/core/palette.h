// palette.h - the 256-entry indexed palette: fixed index map, world (fogged) block,
// per-time-of-day base colours, per-frame effects (flash / mist / fade), device LUT.
// OWNER: group B (palette.cpp holds all colour tables). SHARED CONTRACT (index map).
//
// Index map:
//   0..79   fixed entries (never fogged), named in namespace pal below
//   80..255 world block: pal::world(fog, base) = 80 + fog*22 + base, fog 0..7, base 0..21
// Every drawer writes these indices; nothing outside palette.cpp knows actual colours.
#pragma once
#include <stdint.h>
#include "types.h"

namespace kj {
namespace pal {

// ---- fixed (0..79)
constexpr uint8_t EMPTY = 0;          // "not yet written"; LUT shows it as haze (safety net)
constexpr uint8_t HAZE = 1;           // far sea / below-horizon fill
constexpr uint8_t SKY0 = 2;           // 2..9: sky bands, 0 = horizon ... 7 = zenith
constexpr uint8_t STAR_BRIGHT = 10, STAR_MID = 11, STAR_DIM = 12, STAR_WARM = 13, MILKY = 14;
constexpr uint8_t MOON_DISC = 15, MOON_CRATER = 16, MOON_GLOW_IN = 17, MOON_GLOW_OUT = 18;
//   (at sunset MOON_* entries hold the sun disc / glow colours)
constexpr uint8_t FOAM = 19, GLINT_A = 20, GLINT_B = 21, MOONPATH = 22;
constexpr uint8_t SH_SEA = 23, SH_SHALLOW = 24, SH_SAND = 25, SH_GRASS = 26, SH_TREE = 27,
                  SH_ROCK = 28, SH_SNOW = 29;          // shadowed variants (jet + cloud shadow)
constexpr uint8_t JET_OUTLINE = 30;
constexpr uint8_t JET_BODY0 = 31;     // 31..37: body tones 0 (dark) .. 6 (light)
constexpr uint8_t JET_ACCENT0 = 38;   // 38..41: accent tones 0..3
constexpr uint8_t JET_CANOPY0 = 42;   // 42..44: canopy tones 0..2
constexpr uint8_t JET_NOZZLE = 45, JET_GLOW = 46;
constexpr uint8_t NAV_RED = 47, NAV_GREEN = 48, STROBE = 49;
constexpr uint8_t FLAME0 = 50;        // 50..55: flame hot (0) .. cool (5)
constexpr uint8_t TRAIL0 = 56;        // 56..58: trail young .. old
constexpr uint8_t SMOKE0 = 59;        // 59..61: smoke dark .. light
constexpr uint8_t WAKE_A = 62, WAKE_B = 63;  // bioluminescent at night, foam by day
constexpr uint8_t RING_GOLD = 64, RING_GOLD_DARK = 65, RING_CYAN = 66, RING_GLOW = 67;
constexpr uint8_t BEACON = 68, SPEEDLINE = 69;
constexpr uint8_t HUD_WHITE = 70, HUD_AMBER = 71, HUD_RED = 72, HUD_GREEN = 73, HUD_SHADOW = 74,
                  HUD_CYAN = 75;      // HUD entries are exempt from flash/mist/fade
constexpr uint8_t MIST = 76, SPARK = 77, SPRAY = 78, RESERVED79 = 79;

constexpr uint8_t JET_FIRST = JET_OUTLINE, JET_LAST = STROBE;  // 30..49 = "jet body" range
constexpr uint8_t HUD_FIRST = HUD_WHITE, HUD_LAST = HUD_CYAN;

inline bool isSky(uint8_t i) { return i >= SKY0 && i < SKY0 + 8; }
inline bool isJetBody(uint8_t i) { return i >= JET_BODY0 && i <= STROBE; }   // excludes outline
inline bool isFlame(uint8_t i) { return i >= FLAME0 && i < FLAME0 + 6; }

// ---- world block
constexpr uint8_t WORLD0 = 80;
constexpr int WORLD_BASES = 22;
enum Base : uint8_t {
  B_SEA_DEEP = 0, B_SEA_MID, B_SEA_SHALLOW, B_SURF,
  B_SAND, B_SAND_SIDE,
  B_GRASS_LIT, B_GRASS, B_GRASS_DARK, B_GRASS_SIDE,
  B_TREE_TOP, B_TREE, B_TREE_SIDE,
  B_ROCK_LIT, B_ROCK, B_ROCK_DARK, B_ROCK_SIDE,
  B_SNOW, B_SNOW_SIDE,
  B_CLOUD_TOP, B_CLOUD_SIDE, B_CLOUD_UNDER
};
inline uint8_t world(int fog, int base) { return (uint8_t)(WORLD0 + fog * WORLD_BASES + base); }
inline bool isWorld(uint8_t i) { return i >= WORLD0; }
inline int worldFog(uint8_t i) { return (i - WORLD0) / WORLD_BASES; }
inline int worldBase(uint8_t i) { return (i - WORLD0) % WORLD_BASES; }

}  // namespace pal

// The shadow remap is index-based and identical for every Tod; renderWorld (cloud shadows)
// uses this static table so it does not need a Palette instance. Palette::shade() == table.
const uint8_t* shadeTable();

// Per-frame palette effects (all 0 = off). HUD entries are never affected.
struct PalFx {
  float flash;         // 0..1 lerp toward flashRgb (crash 1.0, perfect 0.35, tod change 0.5)
  uint32_t flashRgb;   // 0xRRGGBB
  float mist;          // 0..1 lerp toward the MIST colour (in-cloud)
  float fade;          // 0..1 lerp toward black (screen transitions)
};

class Palette {
 public:
  void init();                       // builds all three base tables (sunset graded from day)
  void setTod(Tod t);                // selects the base table and rebuilds shade map
  Tod tod() const { return tod_; }
  void setFx(const PalFx& fx) { fx_ = fx; }
  void finalize();                   // once per frame: base + fx -> lut2x_ / rgb888_ (~256 lerps)
  // Device LUT: entry = (c << 16) | c, c = byte-swapped RGB565 of the final colour.
  const uint32_t* lut2x() const { return lut2x_; }
  // Final colours as RGB888 triplets (sim PNGs, serial screenshot).
  const uint8_t* rgb888() const { return rgb888_; }
  // Shadow remap: world indices at fog 0..2 and the fixed sea entries map to their SH_* entry;
  // everything else maps to itself. Idempotent.
  uint8_t shade(uint8_t idx) const { return shade_[idx]; }
  uint32_t baseRgb(uint8_t idx) const { return base_[(int)tod_][idx]; }

 private:
  Tod tod_ = Tod::Night;
  PalFx fx_ = {0.0f, 0xFFF4E0u, 0.0f, 0.0f};
  uint32_t base_[3][256] = {};       // 0xRRGGBB per Tod, fog block precomputed
  uint32_t lut2x_[256] = {};
  uint8_t rgb888_[768] = {};
  uint8_t shade_[256] = {};
  // finalize() skips the rebuild while (tod_, fx_) are unchanged since its last run.
  PalFx lastFx_ = {0.0f, 0u, 0.0f, 0.0f};
  bool finalized_ = false;
};

}  // namespace kj
