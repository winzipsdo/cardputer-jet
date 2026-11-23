// world.cpp - procedural island world: 12 stamps (64x64 cells) + 2x2 max mips, a 32x32 macro
// table, a 128x128 cloud map, beacons, spawn search and the world CRC.
// OWNER: group B. See DESIGN.md section 7.1.
//
// Generation is INTEGER ONLY (Q16 value noise, integer Taylor sin/cos, integer sqrt, kj::hash*,
// kj::Rng). Floats appear only in the runtime queries (groundHeight, cloudThick, ...), never in
// anything that feeds the CRC, so the CRC is bit-identical on MSVC and xtensa gcc.
#include "world.h"

#include <string.h>
#include <math.h>

#include "compat.h"
#include "mathx.h"

namespace kj {

namespace {

constexpr int SN = cfg::STAMP_N;            // 64
constexpr int SCELLS = SN * SN;             // 4096
constexpr int MN = cfg::MIP_N;              // 32
constexpr int MCELLS = MN * MN;             // 1024
constexpr int CN = cfg::CLOUD_N;            // 128
constexpr int Q16 = 65536;

// ------------------------------------------------------------------ storage (.bss, DRAM)
uint8_t gStamps[cfg::STAMP_COUNT * SCELLS];
uint8_t gMips[cfg::STAMP_COUNT * MCELLS];
uint8_t gMacro[cfg::MACRO_N * cfg::MACRO_N];
// The cloud map doubles as the per-stamp elevation scratch (uint16 Q16) during generation;
// clouds are generated last and fully overwrite it.
union CloudStore {
  uint8_t b[CN * CN];
  uint16_t e[CN * CN / 2];
};
CloudStore gCloud;
static_assert(sizeof(uint16_t) * SCELLS <= sizeof(CloudStore), "elevation scratch must fit");

// Stamp id of each stamp-local beacon stored in World::beacons_.
uint8_t gBeaconStamp[cfg::BEACONS_MAX];
// Beacon indices grouped by stamp (ascending index within a stamp): stamp id's beacons are
// gStampBeacon[gStampBeaconStart[id] .. gStampBeaconStart[id + 1]). Built by generate().
uint8_t gStampBeacon[cfg::BEACONS_MAX];
uint8_t gStampBeaconStart[cfg::STAMP_COUNT + 1];

// Coarse occupancy tables for the renderer's march (render_world.cpp skips whole column
// chunks with them). Derived data: rebuilt by generate(), not part of the CRC.
//   gTerrainBlk[stamp][by*4+bx]: 16x16-cell block, bits 0-3 = max level, bit 7 = every cell
//                                 is CELL_DEEP, bit 6 = every cell is water (cellIsWater;
//                                 findSpawn only - the renderer masks it off)
//                                 (stamp-local block coordinates, untransposed)
//   gCloudBlk[by*16+bx]          : 8x8 cloud-cell block, max thickness
constexpr int TB = 16, TBN = SN / TB;   // 4x4 blocks per stamp
constexpr int CB = 8, CBN = CN / CB;    // 16x16 cloud blocks
uint8_t gTerrainBlk[cfg::STAMP_COUNT * TBN * TBN];
uint8_t gCloudBlk[CBN * CBN];

// ------------------------------------------------------------------ CRC32 table (constexpr)
struct CrcTab {
  uint32_t t[256];
  constexpr CrcTab() : t() {
    for (uint32_t i = 0; i < 256; ++i) {
      uint32_t c = i;
      for (int k = 0; k < 8; ++k) c = (c & 1u) ? (0xEDB88320u ^ (c >> 1)) : (c >> 1);
      t[i] = c;
    }
  }
};
constexpr CrcTab kCrc{};

uint32_t crcUpdate(uint32_t crc, const uint8_t* p, size_t n) {
  for (size_t i = 0; i < n; ++i) crc = kCrc.t[(crc ^ p[i]) & 0xFFu] ^ (crc >> 8);
  return crc;
}

// ------------------------------------------------------------------ integer math helpers
// Integer square root (floor) of a 64-bit value.
uint32_t isqrt64(uint64_t v) {
  uint64_t r = 0, bit = (uint64_t)1 << 62;
  while (bit > v) bit >>= 2;
  while (bit != 0) {
    if (v >= r + bit) {
      v -= r + bit;
      r = (r >> 1) + bit;
    } else {
      r >>= 1;
    }
    bit >>= 2;
  }
  return (uint32_t)r;
}

// sin / cos of a Q16 angle in radians, |a| <= ~1.7 (Taylor series, integer only, Q16 result).
int32_t isinQ16(int32_t a) {
  int64_t x = a;
  int64_t x2 = (x * x) / Q16;
  int64_t t = x, s = x;
  t = -((t * x2) / Q16) / 6;   s += t;
  t = -((t * x2) / Q16) / 20;  s += t;
  t = -((t * x2) / Q16) / 42;  s += t;
  t = -((t * x2) / Q16) / 72;  s += t;
  return (int32_t)s;
}
int32_t icosQ16(int32_t a) {
  int64_t x = a;
  int64_t x2 = (x * x) / Q16;
  int64_t t = Q16, s = Q16;
  t = -((t * x2) / Q16) / 2;   s += t;
  t = -((t * x2) / Q16) / 12;  s += t;
  t = -((t * x2) / Q16) / 30;  s += t;
  t = -((t * x2) / Q16) / 56;  s += t;
  t = -((t * x2) / Q16) / 90;  s += t;
  return (int32_t)s;
}
constexpr int32_t PI_Q16 = 205887;      // pi * 65536
constexpr int32_t HALF_PI_Q16 = 102944;

// cos/sin of an angle in [0, pi) via the [-pi/2, pi/2] series.
void icosSinHalfTurn(int32_t a, int32_t& c, int32_t& s) {
  int32_t b = a - HALF_PI_Q16;  // a = b + pi/2: cos a = -sin b, sin a = cos b
  c = -isinQ16(b);
  s = icosQ16(b);
}

// Smoothstep of a Q16 value in [0, 65536].
inline int32_t smoothQ16(int32_t f) {
  int64_t ff = (int64_t)f * f / Q16;
  return (int32_t)(ff * (3 * Q16 - 2 * (int64_t)f) / Q16);
}

// Value noise on a 64-periodic lattice; returns Q16 in [0, 65535].
int32_t valueNoise(uint32_t seed, int x, int y, int cellSize, int octave) {
  const int period = SN / cellSize;
  const int ix = x / cellSize, iy = y / cellSize;
  const int fxq = ((x % cellSize) * Q16) / cellSize;
  const int fyq = ((y % cellSize) * Q16) / cellSize;
  const int ix1 = (ix + 1) % period, iy1 = (iy + 1) % period;
  const int32_t oseed = (int32_t)(seed + (uint32_t)octave * 0x3C6EF372u);
  const int32_t v00 = (int32_t)(hash3i(ix, iy, oseed) & 0xFFFFu);
  const int32_t v10 = (int32_t)(hash3i(ix1, iy, oseed) & 0xFFFFu);
  const int32_t v01 = (int32_t)(hash3i(ix, iy1, oseed) & 0xFFFFu);
  const int32_t v11 = (int32_t)(hash3i(ix1, iy1, oseed) & 0xFFFFu);
  const int64_t sx = smoothQ16(fxq), sy = smoothQ16(fyq);
  const int64_t a = v00 + ((int64_t)(v10 - v00) * sx) / Q16;
  const int64_t b = v01 + ((int64_t)(v11 - v01) * sx) / Q16;
  return (int32_t)(a + ((b - a) * sy) / Q16);
}

// Q16 fractions used by the classifier (DESIGN.md 7.1, step 3..6).
constexpr int32_t D_DEEP = 4588;      // 0.07
constexpr int32_t D_MID = 1966;       // 0.03
constexpr int32_t D_SHALLOW = 524;    // 0.008
constexpr int32_t BEACH = 786;        // 0.012
constexpr int32_t SLOPE_TAU = 459;    // 0.007
constexpr int32_t HR_GRASS = 32768;   // 0.50
constexpr int32_t HR_ROCK = 51118;    // 0.78
constexpr int32_t TREE_NOISE = 36045; // 0.55
constexpr int32_t SEA_MIN = 6554;     // sea level is never below 0.10 (keeps a DEEP rim)
constexpr int32_t LAND_MIN_Q16 = (int32_t)(cfg::STAMP_LAND_MIN * 65536.0f);  // compile-time
constexpr int32_t LAND_MAX_Q16 = (int32_t)(cfg::STAMP_LAND_MAX * 65536.0f);
constexpr int ISLAND_PERMILLE = (int)(cfg::MACRO_ISLAND_FRACTION * 1000.0f + 0.5f);
constexpr int CLOUD_RANK_LIMIT = (int)(cfg::CLOUD_COVER * 16.0f * 1000.0f + 0.5f);

inline bool inStamp(int x, int y) { return (unsigned)x < (unsigned)SN && (unsigned)y < (unsigned)SN; }
inline int clampCell(int v) { return v < 0 ? 0 : (v > SN - 1 ? SN - 1 : v); }
inline bool isWaterMat(uint8_t c) { return (c & 15) <= mat::SURF; }
inline uint8_t mk(int level, int m) { return (uint8_t)((level << 4) | m); }

bool adjWater4(const uint8_t* s, int x, int y) {
  static const int DX[4] = {1, -1, 0, 0}, DY[4] = {0, 0, 1, -1};
  for (int k = 0; k < 4; ++k) {
    int nx = x + DX[k], ny = y + DY[k];
    if (!inStamp(nx, ny)) continue;
    if (isWaterMat(s[ny * SN + nx])) return true;
  }
  return false;
}

// ------------------------------------------------------------------ one island stamp
void buildStamp(int id, uint32_t worldSeed, uint8_t* out, Beacon* beacons, int& nBeacons) {
  const uint32_t sseed = hash32(worldSeed + (uint32_t)id);
  Rng rng(sseed ^ 0xA5A5F00Du);
  uint16_t* E = gCloud.e;

  // ---- radial falloff parameters (Q8 cells, Q16 angle)
  const int32_t jx = (int32_t)(rng.next() % 4097u) - 2048;   // +-8 cells
  const int32_t jy = (int32_t)(rng.next() % 4097u) - 2048;
  const int32_t cxq = (SN / 2) * 256 + jx, cyq = (SN / 2) * 256 + jy;
  const int32_t el = 179 + (int32_t)(rng.next() % 155u);      // 0.70 .. 1.30 (Q8)
  int32_t rq = 4096 + (int32_t)(rng.next() % 2561u);          // 16 .. 26 cells (Q8)
  const int32_t ang = (int32_t)(rng.next() % (uint32_t)PI_Q16);
  int32_t ca, sa;
  icosSinHalfTurn(ang, ca, sa);
  // Keep the whole footprint inside the 4-cell DEEP border.
  {
    const int32_t ajit = (jx < 0 ? -jx : jx) > (jy < 0 ? -jy : jy) ? (jx < 0 ? -jx : jx)
                                                                    : (jy < 0 ? -jy : jy);
    const int32_t room = (SN / 2 - cfg::STAMP_BORDER - 1) * 256 - ajit;  // Q8 cells
    const int32_t elMax = el > 256 ? el : 256;
    const int32_t extent = (int32_t)((int64_t)rq * 95 / 100 * elMax / 256);
    if (extent > room) rq = (int32_t)((int64_t)room * 256 / elMax * 100 / 95);
  }
  const int64_t rr = (int64_t)rq * 95 / 100;  // falloff reaches 0 at 0.95 r

  // ---- elevation: 4 octaves of value noise x radial falloff
  static const int OCT_CELL[4] = {32, 16, 8, 4};
  static const int32_t OCT_AMP[4] = {65536, 32768, 16384, 7864};
  const int32_t ampSum = 65536 + 32768 + 16384 + 7864;
  int footprint = 0;
  int32_t emax = 0;
  for (int y = 0; y < SN; ++y) {
    for (int x = 0; x < SN; ++x) {
      int32_t e = 0;
      if (x >= cfg::STAMP_BORDER && y >= cfg::STAMP_BORDER && x < SN - cfg::STAMP_BORDER &&
          y < SN - cfg::STAMP_BORDER) {
        const int64_t dx = (int64_t)x * 256 + 128 - cxq, dy = (int64_t)y * 256 + 128 - cyq;
        int64_t u = (dx * ca + dy * sa) / Q16;
        int64_t v = (-dx * sa + dy * ca) / Q16;
        u = u * 256 / el;
        v = v * el / 256;
        const int64_t t2 = (u * u + v * v) * Q16 / (rr * rr);  // Q16 (d/r)^2
        if (t2 < Q16) {
          const int32_t t = (int32_t)isqrt64((uint64_t)t2 << 16);  // Q16 d/r
          const int32_t fall = Q16 - smoothQ16(t > Q16 ? Q16 : t);
          int64_t n = 0;
          for (int o = 0; o < 4; ++o) n += (int64_t)valueNoise(sseed, x, y, OCT_CELL[o], o) * OCT_AMP[o];
          n = n / ampSum;  // Q16
          e = (int32_t)(n * fall / Q16);
          if (e > 65535) e = 65535;
          if (e > 0) ++footprint;
        }
      }
      E[y * SN + x] = (uint16_t)e;
      if (e > emax) emax = e;
    }
  }

  // ---- sea level from a 256-bin histogram: land fraction of the footprint in [0.30, 0.45]
  int hist[256];
  for (int i = 0; i < 256; ++i) hist[i] = 0;
  for (int i = 0; i < SCELLS; ++i) ++hist[E[i] >> 8];
  const int32_t frac = LAND_MIN_Q16 + (int32_t)(rng.next() % (uint32_t)(LAND_MAX_Q16 - LAND_MIN_Q16 + 1));
  const int target = (int)(((int64_t)footprint * frac) / Q16);
  int acc = 0, bin = 255;
  for (; bin > 0; --bin) {
    acc += hist[bin];
    if (acc >= target) break;
  }
  int32_t sea = bin << 8;
  if (sea < SEA_MIN) sea = SEA_MIN;
  if (sea > emax - 2 * BEACH) sea = emax - 2 * BEACH;  // always some land
  if (sea < SEA_MIN) sea = SEA_MIN;
  const int32_t hf = 40632 + (int32_t)(rng.next() % 24905u);  // height factor 0.62 .. 1.0
  int32_t landSpan = emax - sea - BEACH;
  if (landSpan < 1) landSpan = 1;

  // ---- classify
  for (int y = 0; y < SN; ++y) {
    for (int x = 0; x < SN; ++x) {
      const int32_t e = E[y * SN + x];
      const uint32_t h = hash3i(x, y, (int32_t)sseed);
      uint8_t c;
      if (e <= sea) {
        const int32_t d = sea - e;
        int m = d > D_DEEP ? mat::DEEP : d > D_MID ? mat::MID : d > D_SHALLOW ? mat::SHALLOW : mat::SURF;
        c = mk(0, m);
      } else if (e - sea < BEACH) {
        c = mk(0, (h & 255u) < 33u ? mat::SPECK : mat::SAND);  // 13 % specks
      } else {
        int32_t hr = (int32_t)(((int64_t)(e - sea - BEACH) * hf) / landSpan);
        if (hr > Q16 - 1) hr = Q16 - 1;
        int level, kind;  // kind 0 grass, 1 rock, 2 snow
        if (hr < HR_GRASS) {
          level = 1 + (int)(((int64_t)hr * 8) / HR_GRASS);
          if (level > 8) level = 8;
          kind = 0;
        } else if (hr < HR_ROCK) {
          level = 9 + (int)(((int64_t)(hr - HR_GRASS) * 4) / (HR_ROCK - HR_GRASS));
          if (level > 12) level = 12;
          kind = 1;
        } else {
          level = 13 + (int)(((int64_t)(hr - HR_ROCK) * 3) / (Q16 - HR_ROCK));
          if (level > 15) level = 15;
          kind = 2;
        }
        // Baked slope light from the (-x,-y) side: g < -tau lit, g > tau dark.
        const int32_t g = (int32_t)E[clampCell(y - 2) * SN + clampCell(x - 2)] -
                          (int32_t)E[clampCell(y + 2) * SN + clampCell(x + 2)];
        int shade = g < -SLOPE_TAU ? 0 : (g > SLOPE_TAU ? 2 : 1);  // 0 lit, 1 normal, 2 dark
        if (((h >> 8) & 7u) == 0) shade = shade == 1 ? ((h >> 11) & 1u ? 0 : 2) : 1;  // 1/8 sprinkle
        int m;
        if (kind == 0) m = shade == 0 ? mat::GRASS_L : shade == 1 ? mat::GRASS : mat::GRASS_D;
        else if (kind == 1) m = shade == 0 ? mat::ROCK_L : shade == 1 ? mat::ROCK : mat::ROCK_D;
        else m = shade == 2 ? mat::SNOW_D : mat::SNOW;
        c = mk(level, m);
      }
      out[y * SN + x] = c;
    }
  }

  // ---- beach ring: low land touching water becomes sand (level 0)
  for (int y = 0; y < SN; ++y) {
    for (int x = 0; x < SN; ++x) {
      const uint8_t c = out[y * SN + x];
      if (isWaterMat(c) || (c >> 4) > 2) continue;
      const int m = c & 15;
      if (m != mat::GRASS_L && m != mat::GRASS && m != mat::GRASS_D) continue;
      if (!adjWater4(out, x, y)) continue;
      const uint32_t h = hash3i(x, y, (int32_t)(sseed ^ 0x51u));
      if ((c >> 4) == 2 && (h & 1u)) continue;  // keep some low grass banks
      out[y * SN + x] = mk(0, (h & 255u) < 33u ? mat::SPECK : mat::SAND);
    }
  }

  // ---- coastal water ring: SURF next to land, SHALLOW within 2, MID within 3 (Chebyshev)
  for (int y = 0; y < SN; ++y) {
    for (int x = 0; x < SN; ++x) {
      const uint8_t c = out[y * SN + x];
      if (!isWaterMat(c)) continue;
      int best = 4;
      for (int dy = -3; dy <= 3 && best > 1; ++dy) {
        for (int dx = -3; dx <= 3; ++dx) {
          const int nx = x + dx, ny = y + dy;
          if (!inStamp(nx, ny) || isWaterMat(out[ny * SN + nx])) continue;
          const int ad = dx < 0 ? -dx : dx, bd = dy < 0 ? -dy : dy;
          const int d = ad > bd ? ad : bd;
          if (d < best) best = d;
        }
      }
      int want = best == 1 ? mat::SURF : best == 2 ? mat::SHALLOW : best == 3 ? mat::MID : mat::DEEP;
      // Diagonal-only contact gets shallow, not surf (reads cleaner on terraces).
      if (best == 1 && !(inStamp(x + 1, y) && !isWaterMat(out[y * SN + x + 1])) &&
          !(inStamp(x - 1, y) && !isWaterMat(out[y * SN + x - 1])) &&
          !(inStamp(x, y + 1) && !isWaterMat(out[(y + 1) * SN + x])) &&
          !(inStamp(x, y - 1) && !isWaterMat(out[(y - 1) * SN + x])))
        want = mat::SHALLOW;
      if (want > (c & 15)) out[y * SN + x] = mk(0, want);
    }
  }

  // ---- trees: jittered 5-cell grid on low grass, gated by a second noise and a 40 % hash
  for (int gy = 0; gy < SN / 5; ++gy) {
    for (int gx = 0; gx < SN / 5; ++gx) {
      const uint32_t h = hash3i(gx, gy, (int32_t)(sseed ^ 0x7EE5u));
      const int tx = gx * 5 + 2 + (int)(h % 3u) - 1;
      const int ty = gy * 5 + 2 + (int)((h >> 4) % 3u) - 1;
      if (tx < 1 || ty < 1 || tx >= SN - 1 || ty >= SN - 1) continue;
      const uint8_t c = out[ty * SN + tx];
      const int m = c & 15, lvl = c >> 4;
      if (m < mat::GRASS_L || m > mat::GRASS_D || lvl < 1 || lvl > 5) continue;
      if (((h >> 8) % 100u) >= 40u) continue;
      if (valueNoise(sseed ^ 0x0F0Fu, tx, ty, 8, 5) <= TREE_NOISE) continue;
      for (int dy = -1; dy <= 1; ++dy) {
        for (int dx = -1; dx <= 1; ++dx) {
          uint8_t& rc = out[(ty + dy) * SN + tx + dx];
          const int rm = rc & 15, rl = rc >> 4;
          if (dx == 0 && dy == 0) continue;
          if (rm < mat::GRASS_L || rm > mat::TREE) continue;  // only on grass / other trees
          const int nl = rl > lvl + 1 ? rl : lvl + 1;
          if (rm == mat::TREE_TOP) continue;
          rc = mk(nl, mat::TREE);
        }
      }
      out[ty * SN + tx] = mk(lvl + 2, mat::TREE_TOP);
    }
  }

  // ---- border forced DEEP
  for (int y = 0; y < SN; ++y) {
    for (int x = 0; x < SN; ++x) {
      if (x < cfg::STAMP_BORDER || y < cfg::STAMP_BORDER || x >= SN - cfg::STAMP_BORDER ||
          y >= SN - cfg::STAMP_BORDER)
        out[y * SN + x] = CELL_DEEP;
    }
  }

  // ---- beacons: 1-2 on sand or low grass next to water
  int cand = 0;
  for (int y = cfg::STAMP_BORDER; y < SN - cfg::STAMP_BORDER; ++y) {
    for (int x = cfg::STAMP_BORDER; x < SN - cfg::STAMP_BORDER; ++x) {
      const uint8_t c = out[y * SN + x];
      const int m = c & 15, l = c >> 4;
      const bool ok = (m == mat::SAND || ((m >= mat::GRASS_L && m <= mat::GRASS_D) && l <= 2));
      if (ok && adjWater4(out, x, y)) ++cand;
    }
  }
  if (cand > 0) {
    const int want = 1 + (int)(rng.next() & 1u);
    int placedX[2] = {-100, -100}, placedY[2] = {-100, -100};
    int placed = 0;
    for (int attempt = 0; attempt < 8 && placed < want && nBeacons < cfg::BEACONS_MAX; ++attempt) {
      int pick = (int)(rng.next() % (uint32_t)cand);
      for (int y = cfg::STAMP_BORDER; y < SN - cfg::STAMP_BORDER && pick >= 0; ++y) {
        for (int x = cfg::STAMP_BORDER; x < SN - cfg::STAMP_BORDER; ++x) {
          const uint8_t c = out[y * SN + x];
          const int m = c & 15, l = c >> 4;
          const bool ok = (m == mat::SAND || ((m >= mat::GRASS_L && m <= mat::GRASS_D) && l <= 2));
          if (!ok || !adjWater4(out, x, y)) continue;
          if (pick-- != 0) continue;
          bool farEnough = true;
          for (int k = 0; k < placed; ++k) {
            const int ddx = x - placedX[k], ddy = y - placedY[k];
            if (ddx * ddx + ddy * ddy < 16 * 16) farEnough = false;
          }
          if (farEnough) {
            Beacon& b = beacons[nBeacons];
            b.x = ((float)x + 0.5f) * cfg::CELL_U;
            b.y = ((float)y + 0.5f) * cfg::CELL_U;
            b.z = (float)cfg::HEIGHT_U[l];
            b.phase = (uint8_t)(hash3i(x, y, id) & 255u);
            gBeaconStamp[nBeacons] = (uint8_t)id;
            ++nBeacons;
            placedX[placed] = x;
            placedY[placed] = y;
            ++placed;
          }
          pick = -1;
          break;
        }
      }
    }
  }
}

// 2x2 max mip: the cell with the largest byte (level first, then material) wins.
void buildMip(const uint8_t* s, uint8_t* m) {
  for (int y = 0; y < MN; ++y) {
    for (int x = 0; x < MN; ++x) {
      uint8_t best = 0;
      for (int k = 0; k < 4; ++k) {
        const uint8_t c = s[(y * 2 + (k >> 1)) * SN + x * 2 + (k & 1)];
        if (c > best) best = c;
      }
      m[y * MN + x] = best;
    }
  }
}

// ------------------------------------------------------------------ macro table
inline int macroAt(int tx, int ty) {
  return gMacro[(ty & (cfg::MACRO_N - 1)) * cfg::MACRO_N + (tx & (cfg::MACRO_N - 1))];
}

bool clashes(int tx, int ty, int id) {
  static const int DX[4] = {1, -1, 0, 0}, DY[4] = {0, 0, 1, -1};
  for (int k = 0; k < 4; ++k) {
    const int n = macroAt(tx + DX[k], ty + DY[k]);
    if (n != cfg::MACRO_OCEAN && (n & 15) == id) return true;
  }
  return false;
}

void buildMacro(uint32_t seed) {
  Rng rng(hash32(seed ^ 0x3A11C0DEu));
  const int N = cfg::MACRO_N;
  for (int i = 0; i < N * N; ++i) {
    const uint32_t r = rng.next();
    const uint32_t r2 = rng.next();
    if ((int)(r % 1000u) < ISLAND_PERMILLE) {
      gMacro[i] = (uint8_t)((r2 % (uint32_t)cfg::STAMP_COUNT) | ((r2 >> 16) & 1u ? cfg::MACRO_TRANSPOSE : 0u));
    } else {
      gMacro[i] = cfg::MACRO_OCEAN;
    }
  }
  // Spawn tile is open ocean with islands on its four sides.
  const int sx = N / 2, sy = N / 2;
  gMacro[sy * N + sx] = cfg::MACRO_OCEAN;
  static const int DX[4] = {1, -1, 0, 0}, DY[4] = {0, 0, 1, -1};
  for (int k = 0; k < 4; ++k) {
    uint8_t& m = gMacro[(sy + DY[k]) * N + sx + DX[k]];
    if (m == cfg::MACRO_OCEAN) {
      const uint32_t r = rng.next();
      m = (uint8_t)((r % (uint32_t)cfg::STAMP_COUNT) | ((r >> 16) & 1u ? cfg::MACRO_TRANSPOSE : 0u));
    }
  }
  // No two equal stamp ids 4-adjacent (with wrap). One pass suffices: a changed entry always
  // differs from all current neighbours.
  for (int ty = 0; ty < N; ++ty) {
    for (int tx = 0; tx < N; ++tx) {
      uint8_t& m = gMacro[ty * N + tx];
      if (m == cfg::MACRO_OCEAN) continue;
      if (!clashes(tx, ty, m & 15)) continue;
      const int start = (int)(hash2i(tx, ty) % (uint32_t)cfg::STAMP_COUNT);
      for (int k = 0; k < cfg::STAMP_COUNT; ++k) {
        const int id = (start + k) % cfg::STAMP_COUNT;
        if (!clashes(tx, ty, id)) {
          m = (uint8_t)(id | (m & cfg::MACRO_TRANSPOSE));
          break;
        }
      }
    }
  }
}

// ------------------------------------------------------------------ clouds
void buildClouds(uint32_t seed) {
  uint8_t* C = gCloud.b;
  memset(C, 0, sizeof(gCloud.b));
  Rng rng(hash32(seed ^ 0xC10D5EEDu));
  for (int k = 0; k < cfg::CLOUD_CLUSTERS; ++k) {
    const int rank = (int)(rng.next() & 15u);
    const int32_t ccx = (int32_t)(rng.next() % (uint32_t)(CN * 256));   // Q8 cloud cells
    const int32_t ccy = (int32_t)(rng.next() % (uint32_t)(CN * 256));
    const int32_t ang = -26214 + (int32_t)(rng.next() % 91751u);       // 0.3 +- 0.7 rad (Q16)
    const int nPuffs = 3 + (int)(rng.next() % 5u);                      // 3..7
    const int32_t len = 1024 + (int32_t)(rng.next() % 2049u);           // 4..12 cells (Q8)
    const int32_t ca = icosQ16(ang), sa = isinQ16(ang);
    int32_t prad[7];
    for (int p = 0; p < nPuffs; ++p) prad[p] = 384 + (int32_t)(rng.next() % 513u);  // 1.5..3.5 (Q8)
    if (rank * 1000 >= CLOUD_RANK_LIMIT) continue;  // not visible at this cover
    for (int p = 0; p < nPuffs; ++p) {
      const int32_t t = nPuffs == 1 ? 0 : (int32_t)(p * 2 * Q16 / (nPuffs - 1)) - Q16;  // -1..1 Q16
      const int32_t at = t < 0 ? -t : t;
      const int64_t along = (int64_t)t * (len / 2) / Q16;  // Q8
      const int32_t pcx = ccx + (int32_t)(along * ca / Q16);
      const int32_t pcy = ccy + (int32_t)(along * sa / Q16);
      const int32_t r = (int32_t)((int64_t)prad[p] * (Q16 - (int64_t)at * 45 / 100) / Q16);  // Q8
      if (r < 64) continue;
      int32_t tmax = r * 12 / 10;  // 0.6 r * 2, Q8 layers
      if (tmax < 512) tmax = 512;
      if (tmax > 1536) tmax = 1536;
      const int x0 = (pcx - r) >> 8, x1 = (pcx + r) >> 8;
      const int y0 = (pcy - r) >> 8, y1 = (pcy + r) >> 8;
      for (int yy = y0 - 1; yy <= y1 + 1; ++yy) {
        for (int xx = x0 - 1; xx <= x1 + 1; ++xx) {
          const int64_t dx = (int64_t)xx * 256 + 128 - pcx, dy = (int64_t)yy * 256 + 128 - pcy;
          const int64_t d2 = (dx * dx + dy * dy) * Q16 / ((int64_t)r * r);  // Q16
          if (d2 >= Q16) continue;
          const int64_t s = isqrt64((uint64_t)(Q16 - d2) << 16);  // Q16 sqrt(1 - d^2)
          int th = (int)((s * tmax / Q16 + 128) >> 8);
          if (th > cfg::CLOUD_MAX_LAYERS) th = cfg::CLOUD_MAX_LAYERS;
          if (th <= 0) continue;
          uint8_t& cell = C[((yy & (CN - 1)) << 7) | (xx & (CN - 1))];
          if (th > cell) cell = (uint8_t)th;
        }
      }
    }
  }
}

void buildBlockTables() {
  for (int id = 0; id < cfg::STAMP_COUNT; ++id) {
    const uint8_t* st = gStamps + id * SCELLS;
    for (int by = 0; by < TBN; ++by) {
      for (int bx = 0; bx < TBN; ++bx) {
        int maxL = 0;
        bool deep = true, water = true;
        for (int y = by * TB; y < by * TB + TB; ++y) {
          for (int x = bx * TB; x < bx * TB + TB; ++x) {
            const uint8_t c = st[y * SN + x];
            if ((c >> 4) > maxL) maxL = c >> 4;
            if (c != CELL_DEEP) deep = false;
            if (!cellIsWater(c)) water = false;
          }
        }
        gTerrainBlk[id * TBN * TBN + by * TBN + bx] = (uint8_t)(maxL | (deep ? 0x80 : 0) | (water ? 0x40 : 0));
      }
    }
  }
  for (int by = 0; by < CBN; ++by) {
    for (int bx = 0; bx < CBN; ++bx) {
      uint8_t m = 0;
      for (int y = by * CB; y < by * CB + CB; ++y)
        for (int x = bx * CB; x < bx * CB + CB; ++x)
          if (gCloud.b[y * CN + x] > m) m = gCloud.b[y * CN + x];
      gCloudBlk[by * CBN + bx] = m;
    }
  }
}

}  // namespace

// Accessors for render_world.cpp (same owner; declared there, not in the shared header).
const uint8_t* worldTerrainBlocks() { return gTerrainBlk; }
const uint8_t* worldCloudBlocks() { return gCloudBlk; }

// ================================================================== World
void World::generate(uint32_t seed) {
  stamps_ = gStamps;
  mips_ = gMips;
  macro_ = gMacro;
  clouds_ = gCloud.b;
  nBeacons_ = 0;
  for (int id = 0; id < cfg::STAMP_COUNT; ++id) {
    uint8_t* s = gStamps + id * SCELLS;
    buildStamp(id, seed, s, beacons_, nBeacons_);
    buildMip(s, gMips + id * MCELLS);
  }
  buildMacro(seed);
  buildClouds(seed);  // overwrites the elevation scratch
  buildBlockTables();
  {
    int n = 0;
    for (int id = 0; id < cfg::STAMP_COUNT; ++id) {
      gStampBeaconStart[id] = (uint8_t)n;
      for (int b = 0; b < nBeacons_; ++b)
        if (gBeaconStamp[b] == id) gStampBeacon[n++] = (uint8_t)b;
    }
    gStampBeaconStart[cfg::STAMP_COUNT] = (uint8_t)n;
  }
  cloudOffX = 0.0f;
  cloudOffY = 0.0f;
}

uint32_t World::crc() const {
  uint32_t c = 0xFFFFFFFFu;
  c = crcUpdate(c, gStamps, sizeof(gStamps));
  c = crcUpdate(c, gMips, sizeof(gMips));
  c = crcUpdate(c, gMacro, sizeof(gMacro));
  c = crcUpdate(c, gCloud.b, sizeof(gCloud.b));
  return c ^ 0xFFFFFFFFu;
}

float World::groundHeight(float x, float y) const {
  const uint8_t c = cell((int)floorf(x * (1.0f / cfg::CELL_U)), (int)floorf(y * (1.0f / cfg::CELL_U)));
  return (float)cfg::HEIGHT_U[c >> 4];
}

bool World::isWater(float x, float y) const {
  return cellIsWater(cell((int)floorf(x * (1.0f / cfg::CELL_U)), (int)floorf(y * (1.0f / cfg::CELL_U))));
}

float World::maxGroundAround(float x, float y, float r) const {
  float m = 0.0f;
  for (int j = -1; j <= 1; ++j) {
    for (int i = -1; i <= 1; ++i) {
      const float h = groundHeight(x + (float)i * r, y + (float)j * r);
      if (h > m) m = h;
    }
  }
  return m;
}

void World::setTime(float t) {
  const float period = (float)cfg::CLOUD_N * cfg::CLOUD_CELL_U;  // 1024 u
  cloudOffX = fmodf(cfg::CLOUD_DRIFT_X * t, period);
  cloudOffY = fmodf(cfg::CLOUD_DRIFT_Y * t, period);
  if (cloudOffX < 0.0f) cloudOffX += period;
  if (cloudOffY < 0.0f) cloudOffY += period;
}

int World::cloudThick(float x, float y) const {
  const int cx = (int)floorf((x - cloudOffX) * (1.0f / cfg::CLOUD_CELL_U));
  const int cy = (int)floorf((y - cloudOffY) * (1.0f / cfg::CLOUD_CELL_U));
  return cloudCell(cx, cy);
}

bool World::insideCloud(float x, float y, float z) const {
  const int k = cloudThick(x, y);
  if (k <= 0) return false;
  return z >= cfg::CLOUD_BASE && z <= cfg::CLOUD_BASE + cfg::CLOUD_LAYER_U * (float)k;
}

// ------------------------------------------------------------------ spawn search
namespace {
// 16 headings psi_k = k*pi/8: forward = (sin psi, -cos psi), Q8.
constexpr int DIR_SIN_Q8[16] = {0, 98, 181, 237, 256, 237, 181, 98, 0, -98, -181, -237, -256, -237, -181, -98};
constexpr int DIR_COS_Q8[16] = {256, 237, 181, 98, 0, -98, -181, -237, -256, -237, -181, -98, 0, 98, 181, 237};
}  // namespace

// gTerrainBlk flags of the 16x16-cell block (bx, by) (world block coordinates, wrapped):
// 0x80 every cell CELL_DEEP, 0x40 every cell water. Ocean tiles are both.
uint32_t World::blockFlags(int bx, int by) const {
  bx &= (cfg::WORLD_CELLS / TB) - 1;
  by &= (cfg::WORLD_CELLS / TB) - 1;
  const uint8_t t = macro_[(by >> 2) * cfg::MACRO_N + (bx >> 2)];
  if (t == cfg::MACRO_OCEAN) return 0xC0;
  int lx = bx & (TBN - 1), ly = by & (TBN - 1);
  if (t & cfg::MACRO_TRANSPOSE) { const int s = lx; lx = ly; ly = s; }
  return gTerrainBlk[(t & 15) * TBN * TBN + ly * TBN + lx] & 0xC0u;
}

void World::findSpawn(uint32_t salt, float& x, float& y, float& yaw) const {
  int ocx = cfg::WORLD_CELLS / 2, ocy = cfg::WORLD_CELLS / 2;
  if (salt != 0) {
    const uint32_t h = hash32(salt * 0x9E3779B1u + 0x5EEDu);
    ocx += (int)(h & 1023u) - 512;
    ocy += (int)((h >> 10) & 1023u) - 512;
  }
  // Direction order: nearest to LIGHT_AZ first (ties toward the moon / sun).
  int kl = (int)floorf(cfg::LIGHT_AZ * (8.0f / PI) + 0.5f);
  kl = ((kl % 16) + 16) % 16;
  int order[16];
  order[0] = kl;
  for (int i = 1, n = 1; n < 16; ++i) {
    order[n++] = (kl - i + 16) % 16;
    if (n < 16) order[n++] = (kl + i) % 16;
  }
  const int minSteps = 30, maxSteps = 75;  // 120 u .. 300 u in 4 u cells
  // Tiers: headings within 1, 3, then 8 steps (22.5 deg each) of LIGHT_AZ, so the moon / sun is
  // ahead at the start whenever the world allows it.
  static const int TIER_DIRS[3] = {3, 7, 16};
  for (int tier = 0; tier < 3; ++tier) {
    const int ndir = TIER_DIRS[tier];
    for (int r = 0; r <= 160; ++r) {
      const int per = r == 0 ? 1 : 8 * r;
      for (int i = 0; i < per; ++i) {
        int dx, dy;
        if (r == 0) { dx = 0; dy = 0; }
        else if (i < 2 * r + 1) { dx = -r + i; dy = -r; }
        else if (i < 4 * r + 2) { dx = -r + (i - (2 * r + 1)); dy = r; }
        else if (i < 6 * r + 1) { dx = -r; dy = -r + 1 + (i - (4 * r + 2)); }
        else { dx = r; dy = -r + 1 + (i - (6 * r + 1)); }
        const int cx = ocx + dx, cy = ocy + dy;
        // Open water: this cell and its 8 neighbours are DEEP or MID. (All of them lie in one
        // all-CELL_DEEP 16x16 block: open without lookups.)
        bool open = true;
        const int ix = cx & 15, iy = cy & 15;
        if (!(ix >= 1 && ix <= 14 && iy >= 1 && iy <= 14 && (blockFlags(cx >> 4, cy >> 4) & 0x80u))) {
          for (int j = -1; j <= 1 && open; ++j)
            for (int k = -1; k <= 1; ++k)
              if ((cell(cx + k, cy + j) & 15) > mat::MID) { open = false; break; }
        }
        if (!open) continue;
        for (int oi = 0; oi < ndir; ++oi) {
          const int d = order[oi];
          const int32_t sx = DIR_SIN_Q8[d], sy = -DIR_COS_Q8[d];
          int32_t px = cx * 256 + 128 + sx, py = cy * 256 + 128 + sy;
          int hit = -1;
          for (int s = 1; s <= maxSteps;) {
            // Steps inside an all-water 16x16-cell block cannot hit: jump to the first step
            // outside the block (same first hit as testing every step).
            if (blockFlags(px >> 12, py >> 12) & 0x40u) {
              int k = maxSteps;  // further steps that stay inside this block
              if (sx > 0) { const int kk = (int)(((((px >> 12) + 1) << 12) - 1 - px) / sx); if (kk < k) k = kk; }
              else if (sx < 0) { const int kk = (int)((px - ((px >> 12) << 12)) / -sx); if (kk < k) k = kk; }
              if (sy > 0) { const int kk = (int)(((((py >> 12) + 1) << 12) - 1 - py) / sy); if (kk < k) k = kk; }
              else if (sy < 0) { const int kk = (int)((py - ((py >> 12) << 12)) / -sy); if (kk < k) k = kk; }
              s += k + 1;
              px += (k + 1) * sx;
              py += (k + 1) * sy;
              continue;
            }
            if (!cellIsWater(cell(px >> 8, py >> 8))) { hit = s; break; }
            ++s;
            px += sx;
            py += sy;
          }
          if (hit >= minSteps) {
            x = wrapPos(((float)cx + 0.5f) * cfg::CELL_U);
            y = wrapPos(((float)cy + 0.5f) * cfg::CELL_U);
            yaw = wrapPi((float)d * (PI / 8.0f));
            return;
          }
        }
      }
    }
  }
  x = wrapPos(((float)ocx + 0.5f) * cfg::CELL_U);
  y = wrapPos(((float)ocy + 0.5f) * cfg::CELL_U);
  yaw = cfg::LIGHT_AZ;
}

// ------------------------------------------------------------------ beacons
int World::beaconsNear(float x, float y, float r, Beacon* out, int maxN) const {
  if (maxN <= 0 || nBeacons_ == 0) return 0;
  const float T = (float)cfg::STAMP_N * cfg::CELL_U;  // 256 u per macro tile
  const int tx0 = (int)floorf((x - r) / T), tx1 = (int)floorf((x + r) / T);
  const int ty0 = (int)floorf((y - r) / T), ty1 = (int)floorf((y + r) / T);
  const float r2 = r * r;
  int n = 0;
  for (int ty = ty0; ty <= ty1; ++ty) {
    for (int tx = tx0; tx <= tx1; ++tx) {
      const int mtx = tx & (cfg::MACRO_N - 1), mty = ty & (cfg::MACRO_N - 1);
      const uint8_t m = macro_[mty * cfg::MACRO_N + mtx];
      if (m == cfg::MACRO_OCEAN) continue;
      const int sid = m & 15;
      if (sid >= cfg::STAMP_COUNT) continue;
      for (int j = gStampBeaconStart[sid]; j < gStampBeaconStart[sid + 1]; ++j) {
        const int b = gStampBeacon[j];
        float lx = beacons_[b].x, ly = beacons_[b].y;
        if (m & cfg::MACRO_TRANSPOSE) { const float s = lx; lx = ly; ly = s; }
        const float wx = (float)tx * T + lx, wy = (float)ty * T + ly;
        const float dx = wrapd(wx - x), dy = wrapd(wy - y);
        if (dx * dx + dy * dy > r2) continue;
        Beacon& o = out[n++];
        o.x = wrapPos(wx);
        o.y = wrapPos(wy);
        o.z = beacons_[b].z;
        o.phase = (uint8_t)(beacons_[b].phase + (uint8_t)(hash2i(mtx, mty) & 255u));
        if (n >= maxN) return n;
      }
    }
  }
  return n;
}

}  // namespace kj
