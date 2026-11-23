// palette.cpp - all colour tables of MOONSKIM (night / sunset / day), vividness, fog block,
// shadow remap and the per-frame effect lerps that produce the device LUT.
// OWNER: group B. See DESIGN.md section 12.
//
// Every colour here is authored for this game (original). Sunset is graded from the day
// table except for the entries listed as authored in section 12.
#include "palette.h"

#include <string.h>

#include "config.h"
#include "compat.h"
#include "mathx.h"

namespace kj {

namespace {

// ------------------------------------------------------------------ shadow remap (constexpr)
// World indices at fog 0..2 map to their SH_* entry; the fixed sea entries map to SH_SEA /
// SH_SHALLOW; everything else maps to itself. Cloud tops map to the cloud side tone of the
// same fog level (the jet shadow on a cloud deck). Applying the map twice changes nothing.
constexpr uint8_t shadeOfBase(int base) {
  return base == pal::B_SEA_DEEP || base == pal::B_SEA_MID           ? pal::SH_SEA
         : base == pal::B_SEA_SHALLOW || base == pal::B_SURF         ? pal::SH_SHALLOW
         : base == pal::B_SAND || base == pal::B_SAND_SIDE           ? pal::SH_SAND
         : base >= pal::B_GRASS_LIT && base <= pal::B_GRASS_SIDE     ? pal::SH_GRASS
         : base >= pal::B_TREE_TOP && base <= pal::B_TREE_SIDE       ? pal::SH_TREE
         : base >= pal::B_ROCK_LIT && base <= pal::B_ROCK_SIDE       ? pal::SH_ROCK
         : base == pal::B_SNOW || base == pal::B_SNOW_SIDE           ? pal::SH_SNOW
                                                                     : (uint8_t)0;
}

struct ShadeTab {
  uint8_t t[256];
  constexpr ShadeTab() : t() {
    for (int i = 0; i < 256; ++i) t[i] = (uint8_t)i;
    t[pal::FOAM] = pal::SH_SHALLOW;
    t[pal::GLINT_A] = pal::SH_SEA;
    t[pal::GLINT_B] = pal::SH_SEA;
    t[pal::MOONPATH] = pal::SH_SEA;
    for (int f = 0; f <= 2; ++f) {
      for (int b = 0; b < pal::WORLD_BASES; ++b) {
        int idx = pal::WORLD0 + f * pal::WORLD_BASES + b;
        uint8_t s = shadeOfBase(b);
        if (s != 0) {
          t[idx] = s;
        } else if (b == pal::B_CLOUD_TOP) {
          t[idx] = (uint8_t)(pal::WORLD0 + f * pal::WORLD_BASES + pal::B_CLOUD_SIDE);
        }
      }
    }
  }
};

constexpr ShadeTab kShade{};
// DRAM copy of kShade for the world march's cloud-shadow lookups (a constexpr table lives in
// flash, which is slow on a cache miss). Filled by Palette::init.
uint8_t gShadeRam[256];
bool gShadeRamReady = false;

// ------------------------------------------------------------------ authored tables
constexpr int NB = pal::WORLD_BASES;

// World bases, order = pal::Base.
constexpr uint32_t NIGHT_BASE[NB] = {
    0x0B2266, 0x0E2C78, 0x134088, 0x1E5296,  // sea deep, mid, shallow, surf
    0x3E4C6A, 0x2C3650,                      // sand, sand side
    0x24503A, 0x1A4030, 0x13322A, 0x10291F,  // grass lit, grass, dark, side
    0x1C4A34, 0x143A2C, 0x0A2019,            // tree top, tree, tree side
    0x3A4466, 0x2C3555, 0x222A46, 0x1A2038,  // rock lit, rock, dark, side
    0x6C82C0, 0x3E5288,                      // snow, snow side
    0x6A7CB2, 0x44568C, 0x24315C,            // cloud top, side, under
};
constexpr uint32_t DAY_BASE[NB] = {
    0x2366B0, 0x2E7EC8, 0x44A2D6, 0x6ACAD2,
    0xECD8A0, 0xC8B078,
    0x84CA4E, 0x5EAE36, 0x44902E, 0x377630,
    0x44A042, 0x2C7C38, 0x1A4E2C,
    0xB0AAA0, 0x908A84, 0x74706C, 0x5E5A5C,
    0xF4F8FF, 0xB8C8E2,
    0xFFFFFF, 0xB4C8E6, 0x8CA0C6,
};

// Fixed entries that are identical for every time of day.
struct Fixed { uint8_t idx; uint32_t rgb; };
constexpr Fixed COMMON[] = {
    {pal::JET_BODY0 + 0, 0x12161E}, {pal::JET_BODY0 + 1, 0x2A303C}, {pal::JET_BODY0 + 2, 0x474F5E},
    {pal::JET_BODY0 + 3, 0x6A7486}, {pal::JET_BODY0 + 4, 0x929DB0}, {pal::JET_BODY0 + 5, 0xBCC5D4},
    {pal::JET_BODY0 + 6, 0xE6ECF4},
    {pal::JET_ACCENT0 + 0, 0x4A1014}, {pal::JET_ACCENT0 + 1, 0x8E1C24},
    {pal::JET_ACCENT0 + 2, 0xD02E36}, {pal::JET_ACCENT0 + 3, 0xFF6A62},
    {pal::JET_CANOPY0 + 0, 0x6A4A12}, {pal::JET_CANOPY0 + 1, 0xC08A24},
    {pal::JET_CANOPY0 + 2, 0xFFD86A},
    {pal::JET_NOZZLE, 0x2E2F36}, {pal::JET_GLOW, 0xFF9A40},
    {pal::NAV_RED, 0xFF3030}, {pal::NAV_GREEN, 0x30FF70}, {pal::STROBE, 0xFFFFFF},
    {pal::FLAME0 + 0, 0xFFFFF0}, {pal::FLAME0 + 1, 0xFFE890}, {pal::FLAME0 + 2, 0xFFB040},
    {pal::FLAME0 + 3, 0xFF6A1C}, {pal::FLAME0 + 4, 0xC0301C}, {pal::FLAME0 + 5, 0x5A1E2A},
    {pal::RING_GOLD, 0xFFC83C}, {pal::RING_GOLD_DARK, 0xA0661C}, {pal::RING_CYAN, 0x5FE0FF},
    {pal::RING_GLOW, 0xFFF0A0},
    {pal::HUD_WHITE, 0xF0F4FF}, {pal::HUD_AMBER, 0xFFB000}, {pal::HUD_RED, 0xFF3A30},
    {pal::HUD_GREEN, 0x50FF70}, {pal::HUD_SHADOW, 0x0A0C18}, {pal::HUD_CYAN, 0x5FE0FF},
    {pal::RESERVED79, 0x000000},
};

constexpr Fixed NIGHT_FIXED[] = {
    {pal::HAZE, 0x1A2A58},
    {pal::SKY0 + 0, 0x2A3C76}, {pal::SKY0 + 1, 0x24346A}, {pal::SKY0 + 2, 0x1E2C5E},
    {pal::SKY0 + 3, 0x182452}, {pal::SKY0 + 4, 0x141E48}, {pal::SKY0 + 5, 0x0F173A},
    {pal::SKY0 + 6, 0x0A112C}, {pal::SKY0 + 7, 0x06091C},
    {pal::STAR_BRIGHT, 0xF2F5FF}, {pal::STAR_MID, 0xA4B2E2}, {pal::STAR_DIM, 0x5E6A9C},
    {pal::STAR_WARM, 0xFFE2B0}, {pal::MILKY, 0x1C2452},
    {pal::MOON_DISC, 0xF4F1DC}, {pal::MOON_CRATER, 0xC9C6B4}, {pal::MOON_GLOW_IN, 0x4A5FA2},
    {pal::MOON_GLOW_OUT, 0x2E3F78},
    {pal::FOAM, 0x7FA8E0}, {pal::GLINT_A, 0x6E90E0}, {pal::GLINT_B, 0xD8E6FF},
    {pal::MOONPATH, 0x2A4C9A},
    {pal::SH_SEA, 0x071640}, {pal::SH_SHALLOW, 0x0B2258}, {pal::SH_SAND, 0x222A40},
    {pal::SH_GRASS, 0x0A1A14}, {pal::SH_TREE, 0x08140F}, {pal::SH_ROCK, 0x141A2C},
    {pal::SH_SNOW, 0x2C3A66},
    {pal::JET_OUTLINE, 0x1C2650},
    {pal::TRAIL0 + 0, 0xDCE6FF}, {pal::TRAIL0 + 1, 0x9EB4E8}, {pal::TRAIL0 + 2, 0x6680C0},
    {pal::SMOKE0 + 0, 0x1A1C24}, {pal::SMOKE0 + 1, 0x2E323E}, {pal::SMOKE0 + 2, 0x4A505E},
    {pal::WAKE_A, 0x3CF0D0}, {pal::WAKE_B, 0x1E9E9A},
    {pal::BEACON, 0xFFC060}, {pal::SPEEDLINE, 0x9FB8FF},
    {pal::MIST, 0x6A7CB2}, {pal::SPARK, 0xFFF0A0}, {pal::SPRAY, 0xDCEBFF},
};

constexpr Fixed DAY_FIXED[] = {
    {pal::HAZE, 0xBADEF4},
    {pal::FOAM, 0xF2FCFF}, {pal::GLINT_A, 0x7CC0EA}, {pal::GLINT_B, 0xEEFAFF},
    {pal::MOONPATH, 0x8CCCF0},
    {pal::JET_OUTLINE, 0x0E1020},
    {pal::TRAIL0 + 0, 0xFFFFFF}, {pal::TRAIL0 + 1, 0xD6E8FA}, {pal::TRAIL0 + 2, 0xAACCF0},
    {pal::SMOKE0 + 0, 0x3A3C44}, {pal::SMOKE0 + 1, 0x5A5E68}, {pal::SMOKE0 + 2, 0x8A909C},
    {pal::WAKE_A, 0xF2FCFF}, {pal::WAKE_B, 0xCFEAF6},
    {pal::BEACON, 0xFFC060}, {pal::SPEEDLINE, 0xFFFFFF},
    {pal::MIST, 0xFFFFFF}, {pal::SPARK, 0xFFF0A0}, {pal::SPRAY, 0xFFFFFF},
};

// Sunset entries that are authored (not graded from day).
constexpr Fixed SUNSET_FIXED[] = {
    {pal::HAZE, 0xF6B282},
    {pal::MOON_DISC, 0xFFF2C4}, {pal::MOON_CRATER, 0xFFE6A8}, {pal::MOON_GLOW_IN, 0xFFDA92},
    {pal::MOON_GLOW_OUT, 0xF8B880},
    {pal::FOAM, 0xFFE2D6}, {pal::GLINT_A, 0xFFAA6E}, {pal::GLINT_B, 0xFFE2AA},
    {pal::MOONPATH, 0xE89A80},
    {pal::JET_OUTLINE, 0x2A1426},
    {pal::BEACON, 0xFFC060}, {pal::SPEEDLINE, 0xFFFFFF},
};

// Day entries the sunset grade is applied to (beyond the world bases).
constexpr uint8_t SUNSET_GRADED[] = {
    pal::SH_SEA, pal::SH_SHALLOW, pal::SH_SAND, pal::SH_GRASS, pal::SH_TREE, pal::SH_ROCK,
    pal::SH_SNOW,
    pal::TRAIL0 + 0, pal::TRAIL0 + 1, pal::TRAIL0 + 2,
    pal::SMOKE0 + 0, pal::SMOKE0 + 1, pal::SMOKE0 + 2,
    pal::WAKE_A, pal::WAKE_B, pal::MIST, pal::SPARK, pal::SPRAY,
};

// ------------------------------------------------------------------ colour helpers
inline int chR(uint32_t c) { return (int)((c >> 16) & 0xFF); }
inline int chG(uint32_t c) { return (int)((c >> 8) & 0xFF); }
inline int chB(uint32_t c) { return (int)(c & 0xFF); }
inline int clamp255(int v) { return v < 0 ? 0 : (v > 255 ? 255 : v); }
inline uint32_t pack(int r, int g, int b) {
  return ((uint32_t)clamp255(r) << 16) | ((uint32_t)clamp255(g) << 8) | (uint32_t)clamp255(b);
}
inline int roundi(float v) { return (int)(v < 0.0f ? v - 0.5f : v + 0.5f); }

uint32_t lerpRgb(uint32_t a, uint32_t b, float t) {
  float r = (float)chR(a) + ((float)chR(b) - (float)chR(a)) * t;
  float g = (float)chG(a) + ((float)chG(b) - (float)chG(a)) * t;
  float bl = (float)chB(a) + ((float)chB(b) - (float)chB(a)) * t;
  return pack(roundi(r), roundi(g), roundi(bl));
}

uint32_t scaleRgb(uint32_t c, float kr, float kg, float kb) {
  return pack(roundi((float)chR(c) * kr), roundi((float)chG(c) * kg), roundi((float)chB(c) * kb));
}

// Sunset grade (section 12).
uint32_t gradeSunset(uint32_t c) {
  float r = (float)chR(c), g = (float)chG(c), b = (float)chB(c);
  float r2 = 0.95f * r + 0.10f * g + 12.0f;
  float g2 = 0.72f * g + 0.05f * r + 4.0f;
  float b2 = 0.70f * b + 0.08f * r + 10.0f;
  return pack(roundi(r2), roundi(g2), roundi(b2));
}

// Vividness: saturation x1.10 around luma, then contrast x1.06 around mid grey.
uint32_t vivid(uint32_t c) {
  float r = (float)chR(c), g = (float)chG(c), b = (float)chB(c);
  float l = 0.299f * r + 0.587f * g + 0.114f * b;
  r = l + (r - l) * 1.10f;
  g = l + (g - l) * 1.10f;
  b = l + (b - l) * 1.10f;
  r = (r - 128.0f) * 1.06f + 128.0f;
  g = (g - 128.0f) * 1.06f + 128.0f;
  b = (b - 128.0f) * 1.06f + 128.0f;
  return pack(roundi(r), roundi(g), roundi(b));
}

void applyFixed(uint32_t* t, const Fixed* f, int n) {
  for (int i = 0; i < n; ++i) t[f[i].idx] = f[i].rgb;
}

template <int N>
constexpr int countOf(const Fixed (&)[N]) { return N; }

// Builds one Tod's 80 fixed entries and 22 world bases (before vividness).
void buildNight(uint32_t* fixed, uint32_t* bases) {
  for (int i = 0; i < 80; ++i) fixed[i] = 0;
  applyFixed(fixed, COMMON, countOf(COMMON));
  applyFixed(fixed, NIGHT_FIXED, countOf(NIGHT_FIXED));
  for (int b = 0; b < NB; ++b) bases[b] = NIGHT_BASE[b];
}

void buildDay(uint32_t* fixed, uint32_t* bases) {
  for (int i = 0; i < 80; ++i) fixed[i] = 0;
  applyFixed(fixed, COMMON, countOf(COMMON));
  applyFixed(fixed, DAY_FIXED, countOf(DAY_FIXED));
  for (int b = 0; b < NB; ++b) bases[b] = DAY_BASE[b];
  // Sky: horizon -> zenith, two segments.
  const uint32_t A = 0xC4E6F8, B = 0x78BCEE, C = 0x3C8CDC;
  for (int i = 0; i < 4; ++i) fixed[pal::SKY0 + i] = lerpRgb(A, B, (float)i / 3.0f);
  for (int i = 4; i < 8; ++i) fixed[pal::SKY0 + i] = lerpRgb(B, C, (float)(i - 3) / 4.0f);
  // Day shadows: base x (0.62, 0.67, 0.80).
  fixed[pal::SH_SEA] = scaleRgb(DAY_BASE[pal::B_SEA_MID], 0.62f, 0.67f, 0.80f);
  fixed[pal::SH_SHALLOW] = scaleRgb(DAY_BASE[pal::B_SEA_SHALLOW], 0.62f, 0.67f, 0.80f);
  fixed[pal::SH_SAND] = scaleRgb(DAY_BASE[pal::B_SAND], 0.62f, 0.67f, 0.80f);
  fixed[pal::SH_GRASS] = scaleRgb(DAY_BASE[pal::B_GRASS], 0.62f, 0.67f, 0.80f);
  fixed[pal::SH_TREE] = scaleRgb(DAY_BASE[pal::B_TREE], 0.62f, 0.67f, 0.80f);
  fixed[pal::SH_ROCK] = scaleRgb(DAY_BASE[pal::B_ROCK], 0.62f, 0.67f, 0.80f);
  fixed[pal::SH_SNOW] = scaleRgb(DAY_BASE[pal::B_SNOW], 0.62f, 0.67f, 0.80f);
  // Unused by day (no stars, no disc): park them on sky tones so a stray write is invisible.
  fixed[pal::STAR_BRIGHT] = fixed[pal::SKY0 + 6];
  fixed[pal::STAR_MID] = fixed[pal::SKY0 + 6];
  fixed[pal::STAR_DIM] = fixed[pal::SKY0 + 6];
  fixed[pal::STAR_WARM] = fixed[pal::SKY0 + 6];
  fixed[pal::MILKY] = fixed[pal::SKY0 + 6];
  fixed[pal::MOON_DISC] = 0xFFFFF4;
  fixed[pal::MOON_CRATER] = 0xF4F8FF;
  fixed[pal::MOON_GLOW_IN] = fixed[pal::SKY0 + 1];
  fixed[pal::MOON_GLOW_OUT] = fixed[pal::SKY0 + 2];
}

void buildSunset(uint32_t* fixed, uint32_t* bases) {
  uint32_t dayFixed[80], dayBases[NB];
  buildDay(dayFixed, dayBases);
  for (int i = 0; i < 80; ++i) fixed[i] = dayFixed[i];
  for (int b = 0; b < NB; ++b) bases[b] = gradeSunset(dayBases[b]);
  for (unsigned i = 0; i < sizeof(SUNSET_GRADED); ++i) {
    uint8_t idx = SUNSET_GRADED[i];
    fixed[idx] = gradeSunset(dayFixed[idx]);
  }
  applyFixed(fixed, SUNSET_FIXED, countOf(SUNSET_FIXED));
  // Sky: three-point ramp horizon -> zenith.
  const uint32_t A = 0xFFC07E, B = 0xEA7680, C = 0x52428F;
  for (int i = 0; i < 4; ++i) fixed[pal::SKY0 + i] = lerpRgb(A, B, (float)i / 3.0f);
  for (int i = 4; i < 8; ++i) fixed[pal::SKY0 + i] = lerpRgb(B, C, (float)(i - 3) / 4.0f);
  // No stars at sunset.
  fixed[pal::STAR_BRIGHT] = fixed[pal::SKY0 + 6];
  fixed[pal::STAR_MID] = fixed[pal::SKY0 + 6];
  fixed[pal::STAR_DIM] = fixed[pal::SKY0 + 6];
  fixed[pal::STAR_WARM] = fixed[pal::SKY0 + 6];
  fixed[pal::MILKY] = fixed[pal::SKY0 + 6];
}

}  // namespace

const uint8_t* shadeTable() { return gShadeRamReady ? gShadeRam : kShade.t; }

void Palette::init() {
  memcpy(gShadeRam, kShade.t, sizeof(gShadeRam));
  gShadeRamReady = true;
  finalized_ = false;
  for (int t = 0; t < (int)Tod::Count; ++t) {
    uint32_t fixed[80], bases[NB];
    if (t == (int)Tod::Night) buildNight(fixed, bases);
    else if (t == (int)Tod::Day) buildDay(fixed, bases);
    else buildSunset(fixed, bases);
    // Vividness is baked into every authored and graded colour.
    for (int i = 0; i < 80; ++i) fixed[i] = vivid(fixed[i]);
    for (int b = 0; b < NB; ++b) bases[b] = vivid(bases[b]);
    uint32_t* dst = base_[t];
    for (int i = 0; i < 80; ++i) dst[i] = fixed[i];
    dst[pal::EMPTY] = fixed[pal::HAZE];  // safety net: an EMPTY pixel shows as haze
    const uint32_t haze = fixed[pal::HAZE];
    for (int f = 0; f < cfg::FOG_LEVELS; ++f) {
      for (int b = 0; b < NB; ++b) dst[pal::world(f, b)] = lerpRgb(bases[b], haze, cfg::FOG_T[f]);
    }
  }
  setTod(tod_);
  finalize();
}

void Palette::setTod(Tod t) {
  if ((unsigned)t >= (unsigned)Tod::Count) t = Tod::Night;
  if (t != tod_) finalized_ = false;
  tod_ = t;
  memcpy(shade_, kShade.t, sizeof(shade_));
}

void Palette::finalize() {
  // Nothing changed since the last call (the common case): the LUT is already up to date.
  if (finalized_ && lastFx_.flash == fx_.flash && lastFx_.flashRgb == fx_.flashRgb && lastFx_.mist == fx_.mist &&
      lastFx_.fade == fx_.fade)
    return;
  finalized_ = true;
  lastFx_ = fx_;
  const uint32_t* src = base_[(int)tod_];
  // Integer weights 0..256 for the three effect lerps.
  const int wFlash = (int)(sat(fx_.flash) * 256.0f + 0.5f);
  const int wMist = (int)(sat(fx_.mist) * 256.0f + 0.5f);
  const int wFade = (int)(sat(fx_.fade) * 256.0f + 0.5f);
  const int fr = chR(fx_.flashRgb), fg = chG(fx_.flashRgb), fb = chB(fx_.flashRgb);
  const uint32_t mistRgb = src[pal::MIST];
  const int mr = chR(mistRgb), mg = chG(mistRgb), mb = chB(mistRgb);
  const bool any = (wFlash | wMist | wFade) != 0;
  for (int i = 0; i < 256; ++i) {
    uint32_t c = src[i];
    int r = chR(c), g = chG(c), b = chB(c);
    if (any && !(i >= pal::HUD_FIRST && i <= pal::HUD_LAST)) {
      if (wFlash) {
        r = (r * (256 - wFlash) + fr * wFlash) >> 8;
        g = (g * (256 - wFlash) + fg * wFlash) >> 8;
        b = (b * (256 - wFlash) + fb * wFlash) >> 8;
      }
      if (wMist) {
        r = (r * (256 - wMist) + mr * wMist) >> 8;
        g = (g * (256 - wMist) + mg * wMist) >> 8;
        b = (b * (256 - wMist) + mb * wMist) >> 8;
      }
      if (wFade) {
        r = (r * (256 - wFade)) >> 8;
        g = (g * (256 - wFade)) >> 8;
        b = (b * (256 - wFade)) >> 8;
      }
    }
    uint32_t rgb = ((uint32_t)r << 16) | ((uint32_t)g << 8) | (uint32_t)b;
    uint32_t sw = rgb565sw(rgb);
    lut2x_[i] = (sw << 16) | sw;
    rgb888_[i * 3 + 0] = (uint8_t)r;
    rgb888_[i * 3 + 1] = (uint8_t)g;
    rgb888_[i * 3 + 2] = (uint8_t)b;
  }
}

}  // namespace kj
