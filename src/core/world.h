// world.h - procedural island world (stamps on a macro table) + cloud map + queries.
// OWNER: group B (world.cpp). SHARED CONTRACT.
//
// Terrain cell byte = (level << 4) | material. level 0..15 -> cfg::HEIGHT_U[level] u.
// 1 cell = cfg::CELL_U (4) u; world wraps every 2048 cells (8192 u).
// Storage (static .bss in world.cpp, DRAM, never flash-const):
//   stamps[12][64*64] 49,152 B | mips[12][32*32] 12,288 B | macro[32*32] 1,024 B
//   clouds[128*128]  16,384 B (byte = thickness 0..6 layers)
// World generation uses INTEGER math only (Q16 value noise, kj::hash*, kj::Rng) so the
// world CRC is bit-identical on PC (MSVC) and device (xtensa gcc). Both log it at boot.
#pragma once
#include <stdint.h>
#include "config.h"
#include "types.h"

namespace kj {

namespace mat {
enum : uint8_t {
  DEEP = 0, MID = 1, SHALLOW = 2, SURF = 3,    // level is always 0 for these
  SAND = 4, SPECK = 5,                          // level 0 (beach)
  GRASS_L = 6, GRASS = 7, GRASS_D = 8,          // lit / normal / dark (baked slope light)
  TREE_TOP = 9, TREE = 10,
  ROCK_L = 11, ROCK = 12, ROCK_D = 13,
  SNOW = 14, SNOW_D = 15
};
}
inline uint8_t cellLevel(uint8_t c) { return c >> 4; }
inline uint8_t cellMat(uint8_t c) { return c & 15; }
inline bool cellIsWater(uint8_t c) { return (c & 15) <= mat::SURF; }
constexpr uint8_t CELL_DEEP = 0;  // level 0, DEEP

struct Beacon {
  float x, y, z;     // world position of the light (top of its cell)
  uint8_t phase;     // blink phase 0..255
};

class World {
 public:
  // Generates stamps, mips, macro table, cloud map, beacon list. ~150 ms on device [E].
  void generate(uint32_t seed);
  uint32_t crc() const;  // CRC32 over stamps + mips + macro + clouds

  // ---- terrain (cx, cy are cell coordinates; any int, wrapped internally)
  inline uint8_t cell(int cx, int cy) const;
  inline uint8_t cellMip(int mx, int my) const;  // mip cells are 2x2 terrain cells (8 u)
  float groundHeight(float x, float y) const;    // u; 0 over water. Nearest cell (matches visuals)
  bool isWater(float x, float y) const;
  float maxGroundAround(float x, float y, float r) const;  // max over a 3x3 probe at +-r

  // ---- clouds (drift applied through setTime)
  void setTime(float t);                         // updates cloudOffX/Y = drift * t
  inline uint8_t cloudCell(int ccx, int ccy) const;   // raw map cell, wrapped
  int cloudThick(float x, float y) const;        // layers 0..6 at world (x,y), drift included
  bool insideCloud(float x, float y, float z) const;  // z within [BASE, BASE + 3*thick]
  float cloudOffX = 0.0f, cloudOffY = 0.0f;

  // ---- gameplay helpers
  // Open water, land 120..300 u away; yaw faces that land. salt varies the pick (respawns).
  void findSpawn(uint32_t salt, float& x, float& y, float& yaw) const;
  int beaconsNear(float x, float y, float r, Beacon* out, int maxN) const;
  int beaconCount() const { return nBeacons_; }

  // Raw pointers for the renderer's inner loop.
  const uint8_t* stamps() const { return stamps_; }
  const uint8_t* mips() const { return mips_; }
  const uint8_t* macro() const { return macro_; }
  const uint8_t* clouds() const { return clouds_; }

 private:
  uint32_t blockFlags(int bx, int by) const;  // 16x16-cell block: 0x80 all CELL_DEEP, 0x40 all water
  uint8_t* stamps_ = nullptr;  // [STAMP_COUNT][STAMP_N*STAMP_N]
  uint8_t* mips_ = nullptr;    // [STAMP_COUNT][MIP_N*MIP_N]
  uint8_t* macro_ = nullptr;   // [MACRO_N*MACRO_N]; MACRO_OCEAN or (stamp id | MACRO_TRANSPOSE?)
  uint8_t* clouds_ = nullptr;  // [CLOUD_N*CLOUD_N]
  Beacon beacons_[cfg::BEACONS_MAX] = {};
  int nBeacons_ = 0;
};

inline uint8_t World::cell(int cx, int cy) const {
  cx &= cfg::WORLD_CELLS - 1;
  cy &= cfg::WORLD_CELLS - 1;
  uint8_t t = macro_[(cy >> 6) * cfg::MACRO_N + (cx >> 6)];
  if (t == cfg::MACRO_OCEAN) return CELL_DEEP;
  int lx = cx & (cfg::STAMP_N - 1), ly = cy & (cfg::STAMP_N - 1);
  if (t & cfg::MACRO_TRANSPOSE) { int s = lx; lx = ly; ly = s; }
  return stamps_[(t & 15) * (cfg::STAMP_N * cfg::STAMP_N) + (ly << 6) + lx];
}

inline uint8_t World::cellMip(int mx, int my) const {
  mx &= (cfg::WORLD_CELLS / 2) - 1;
  my &= (cfg::WORLD_CELLS / 2) - 1;
  uint8_t t = macro_[(my >> 5) * cfg::MACRO_N + (mx >> 5)];
  if (t == cfg::MACRO_OCEAN) return CELL_DEEP;
  int lx = mx & (cfg::MIP_N - 1), ly = my & (cfg::MIP_N - 1);
  if (t & cfg::MACRO_TRANSPOSE) { int s = lx; lx = ly; ly = s; }
  return mips_[(t & 15) * (cfg::MIP_N * cfg::MIP_N) + (ly << 5) + lx];
}

inline uint8_t World::cloudCell(int ccx, int ccy) const {
  return clouds_[((ccy & (cfg::CLOUD_N - 1)) << 7) | (ccx & (cfg::CLOUD_N - 1))];
}

}  // namespace kj
