// render_world.cpp - Comanche-style front-to-back column march over the island world, with
// terraced walls, per-step fog, sea effects (surf foam, glints, moon/sun glitter path), the
// cloud slab (tops, walls, undersides) and cloud shadows. OWNER: group B. DESIGN.md 7.2 / 7.3.
//
// Hot path rules: floats per depth step (one divide), integers per column (Q16.16 cell walk).
//
// ESP32-S3 cost structure (all of it gives the same pixels and depths as a plain per-column march;
// every skipped sample is one that could not have changed a pixel):
//  - The 120 columns are processed in chunks of 8. Each chunk keeps the max of its columns'
//    occlusion rows (colY); a chunk is skipped when even the tallest terrain (or the tallest
//    terrain in the 16x16-cell blocks its sample segment touches, world.cpp tables) projects at
//    or below that row. This removes the near-field steps where the sea is still below the frame
//    and the far-field steps where the sea row did not advance, which were most of the samples.
//  - Chunks over all-deep blocks need no lookups (ocean fast path); chunks inside one macro tile
//    use a fixed stamp pointer (two shifts and a load per sample).
//  - When all 8 columns draw from the same colY at the same level (open sea, flat sand), the rows
//    are written as two 32-bit words (write-if-EMPTY merged branch-free) instead of 8 column
//    walks; foam / glints / shadow / glitter path are computed per column in short passes.
//  - Cloud chunks are skipped when the slab rows are already covered or the 8x8-cloud-cell
//    blocks under the segment are all clear.
//  - Per-frame state lives in one stack struct addressed through a base register; lookups and
//    drawing are separate functions so the sampling loops do not spill.
#include "render_world.h"

#include <string.h>
#include <math.h>

#include "compat.h"
#include "config.h"
#include "mathx.h"
#include "palette.h"

namespace kj {

// world.cpp (same owner): coarse occupancy tables rebuilt by World::generate.
const uint8_t* worldTerrainBlocks();  // [STAMP_COUNT][4*4] max level | 0x80 all CELL_DEEP
const uint8_t* worldCloudBlocks();    // [16*16] max cloud thickness per 8x8 cloud cells

namespace {

#if defined(__GNUC__) || defined(__clang__)
#define RW_INLINE inline __attribute__((always_inline))
typedef uint32_t __attribute__((may_alias)) RwU32;
#else
#define RW_INLINE __forceinline
typedef uint32_t RwU32;
#endif
// 32-bit access to a 4-byte ALIGNED address. (A 4-byte memcpy is NOT inlined by the xtensa
// toolchain: each one was a call to the ROM memcpy, ~25 of them per chunk call.)
RW_INLINE uint32_t ld32(const void* p) { return *static_cast<const RwU32*>(p); }
RW_INLINE void st32(void* p, uint32_t v) { *static_cast<RwU32*>(p) = v; }

constexpr int NLEV = 16;
constexpr int NCL = cfg::CLOUD_MAX_LAYERS + 1;  // cloud layer rows 0..6
constexpr float DV_SCALE = 1.0f;                // escape hatch 1 (section 19): 1.4
// Minimum depth step (u at S = 1 px/u; divided by S). cfg::MARCH_DV_MIN (0.5) spends ~75 of
// the 180 steps between the camera and the jet, where only tall terrain can be on screen; half
// a terrain cell is enough there (the column fill is continuous, so a coarser step never leaves
// holes). While only the cloud slab is live (terrain still below the frame) half a cloud cell.
constexpr float DV_MIN_U = 2.0f > cfg::MARCH_DV_MIN ? 2.0f : cfg::MARCH_DV_MIN;
constexpr float DV_MIN_CLOUD_U = 4.0f;
// Clouds (8 u cells) are sampled when the march has moved at least half a cloud cell and, while
// terrain is live, about two terrain steps since the last cloud sample (escape hatch 3, always
// on). The faces span the rows between two sampled steps, so nothing is skipped.
constexpr float CLOUD_MIN_GAP_U = 3.9f;

constexpr int CHUNK = 8, NCHUNK = LW / CHUNK;
static_assert(LW % CHUNK == 0, "chunks must tile the frame width");
constexpr int TBLK_SHIFT = 20;  // Q16.16 terrain cells -> 16-cell blocks
constexpr int TBLK_N = cfg::WORLD_CELLS / 16;  // 128 blocks per world axis
constexpr int CBLK_SHIFT = 19;  // Q16.16 cloud cells -> 8-cell cloud blocks
constexpr int CBLK_N = cfg::CLOUD_N / 8;       // 16
constexpr int NO_PATH = 4 * LH;  // moonYMin value for "no glitter path in this column"
constexpr uint32_t NO_KEY = 0xFFFFFFFFu;

// ------------------------------------------------------------------ static tables (.bss/.data)
uint8_t gNoise64[64 * 64];  // animation texture for foam / glints

// Material -> palette base for top faces and side walls. Read only when the fog level changes
// (at most 8 times per frame) into March::topIdx / sideIdx, which the loops use.
constexpr uint8_t TOP_BASE[16] = {
    pal::B_SEA_DEEP, pal::B_SEA_MID, pal::B_SEA_SHALLOW, pal::B_SURF,
    pal::B_SAND, pal::B_SAND_SIDE,
    pal::B_GRASS_LIT, pal::B_GRASS, pal::B_GRASS_DARK,
    pal::B_TREE_TOP, pal::B_TREE,
    pal::B_ROCK_LIT, pal::B_ROCK, pal::B_ROCK_DARK,
    pal::B_SNOW, pal::B_SNOW_SIDE,
};
constexpr uint8_t SIDE_BASE[16] = {
    pal::B_SEA_DEEP, pal::B_SEA_MID, pal::B_SEA_SHALLOW, pal::B_SURF,
    pal::B_SAND_SIDE, pal::B_SAND_SIDE,
    pal::B_GRASS_SIDE, pal::B_GRASS_SIDE, pal::B_GRASS_SIDE,
    pal::B_TREE_SIDE, pal::B_TREE_SIDE,
    pal::B_ROCK_SIDE, pal::B_ROCK_SIDE, pal::B_ROCK_SIDE,
    pal::B_SNOW_SIDE, pal::B_SNOW_SIDE,
};

// Everything the column loops touch, in one struct (one base register, small offsets).
struct March {
  uint8_t* px;
  uint8_t* dp;
  const uint8_t* macro;
  const uint8_t* stamps;
  const uint8_t* mips;
  const uint8_t* clouds;
  const uint8_t* shade;
  const uint8_t* tblk;
  const uint8_t* cblk;
  // per step
  int rowNow[NLEV];
  int rowPrev[NLEV];
  int cNow[NCL];
  int cPrev[NCL];
  int lim;              // no terrain from this step on reaches a row above lim
  int t36, t6, t4, t8;  // animation phases
  uint8_t topIdx[16];   // fogged palette index per material (top face)
  uint8_t sideIdx[16];  // (side wall)
  uint32_t stx, sty;    // terrain walk step per column (Q16.16 cells)
  uint32_t cstx, csty;  // cloud / cloud-shadow walk step per column (Q16.16 cloud cells)
  int cloudLo;          // topmost row the cloud slab can touch at this step
  uint32_t cTopMask;    // bit k: a cloud of thickness k shows its top face
  uint8_t cIdxTop, cIdxSide, cIdxUnder;
  bool cBelow;          // eye below the slab: undersides
  uint8_t dq;
  bool wordOk;          // px and depth are 4-byte aligned (word writes in drawUniform)
  bool nearFx;
  bool doShadow;
  bool cloudPrevSampled;
  // per column / chunk
  int16_t colY[LW];        // highest row filled so far by terrain (LH = nothing)
  int16_t moonYMin[LW];    // first row of the glitter path in this column (NO_PATH = none)
  uint8_t cloudPrevK[LW];  // cloud thickness sampled in this column at the previous step
  int16_t chunkMaxY[NCHUNK];
  uint8_t chunkDone[NCHUNK];
  uint32_t chunkKey[NCHUNK];   // block range of the chunk's last block lookup (NO_KEY = none)
  uint8_t chunkInfo[NCHUNK];   // blockInfo for chunkKey
  // counters
  uint32_t samples, fills;
};

// Row of a world height at the current step: horizon + m*(eyeZ - z), floored and clamped.
inline int rowOf(float r) {
  if (r < -1024.0f) r = -1024.0f;
  if (r > 1024.0f) r = 1024.0f;
  return (int)(r + 2048.0f) - 2048;
}

inline uint32_t toFix(float v) { return (uint32_t)(int32_t)v; }

// Fills rows [a, b) of column x with idx where the pixel is still EMPTY (clouds).
// checker: only pixels with (x+y)&1 (thin cloud rims).
RW_INLINE uint32_t fillSpan(uint8_t* px, uint8_t* dp, int x, int a, int b, uint8_t idx, uint8_t dq,
                            bool checker) {
  if (a < 0) a = 0;
  if (b > LH) b = LH;
  uint32_t n = 0;
  int p = a * LW + x;
  if (checker) {
    // only (x + y) odd: start at the first such row and step two rows
    if (((x + a) & 1) == 0) { ++a; p += LW; }
    for (int y = a; y < b; y += 2, p += 2 * LW) {
      if (px[p] == pal::EMPTY) {
        px[p] = idx;
        dp[p] = dq;
        ++n;
      }
    }
    return n;
  }
  for (int y = a; y < b; ++y, p += LW) {
    if (px[p] == pal::EMPTY) {
      px[p] = idx;
      dp[p] = dq;
      ++n;
    }
  }
  return n;
}

// Block coordinate range [lo, hi] touched by the samples a, a + st, ..., a + (n-1) st of one
// axis (Q16.16 accumulators, wrapping; blocks are masked by the caller).
RW_INLINE void blockRange(uint32_t a, uint32_t st, int n, int shift, int& lo, int& hi) {
  const int b0 = (int)((int32_t)a >> shift);
  const int32_t frac = (int32_t)(a & ((1u << shift) - 1u));
  const int b1 = b0 + (int)((frac + (int32_t)(st * (uint32_t)(n - 1))) >> shift);
  lo = b0 < b1 ? b0 : b1;
  hi = b0 < b1 ? b1 : b0;
}

// Terrain block info under the samples of a range of block coordinates: bits 0-3 max level,
// bit 7 set when every cell is CELL_DEEP.
JET_NOINLINE uint32_t JET_HOT blockInfo(const March& M, int bx0, int bx1, int by0, int by1) {
  uint32_t maxL = 0, deep = 0x80;
  for (int by = by0; by <= by1; ++by) {
    const int byw = by & (TBLK_N - 1);
    for (int bx = bx0; bx <= bx1; ++bx) {
      const int bxw = bx & (TBLK_N - 1);
      const uint32_t tile = M.macro[(byw >> 2) * cfg::MACRO_N + (bxw >> 2)];
      uint32_t info = 0x80;
      if (tile != cfg::MACRO_OCEAN) {
        int lx = bxw & 3, ly = byw & 3;
        if (tile & cfg::MACRO_TRANSPOSE) { const int t = lx; lx = ly; ly = t; }
        info = M.tblk[(tile & 15) * 16 + (ly << 2) + lx];
      }
      if ((info & 15) > maxL) maxL = info & 15;
      deep &= info;
    }
  }
  return maxL | deep;
}

// True if any terrain cell under the n samples (wx, wy) + i (stx, sty) can reach a row above
// maxY. Also true ("maybe") when the samples touch more than 4 blocks.
JET_NOINLINE bool JET_HOT terrainReach(const March& M, uint32_t wx, uint32_t wy, uint32_t stx, uint32_t sty, int n,
                            int maxY) {
  int bx0, bx1, by0, by1;
  blockRange(wx, stx, n, TBLK_SHIFT, bx0, bx1);
  blockRange(wy, sty, n, TBLK_SHIFT, by0, by1);
  if ((bx1 - bx0 + 1) * (by1 - by0 + 1) > 4) return true;
  return M.rowNow[blockInfo(M, bx0, bx1, by0, by1) & 15] < maxY;
}

// Block info for one chunk's samples, cached per chunk (the segment moves a little per step and
// usually stays on the same blocks). Returns 0xFF ("unknown") when more than 9 blocks.
RW_INLINE uint32_t chunkInfo(March& M, int c, uint32_t wx, uint32_t wy, uint32_t stx, uint32_t sty) {
  int bx0, bx1, by0, by1;
  blockRange(wx, stx, CHUNK, TBLK_SHIFT, bx0, bx1);
  blockRange(wy, sty, CHUNK, TBLK_SHIFT, by0, by1);
  if ((bx1 - bx0 + 1) * (by1 - by0 + 1) > 9) return 0xFF;
  const uint32_t key = (uint32_t)(bx0 & 127) | ((uint32_t)(bx1 & 127) << 7) | ((uint32_t)(by0 & 127) << 14) |
                       ((uint32_t)(by1 & 127) << 21);
  if (M.chunkKey[c] != key) {
    M.chunkKey[c] = key;
    M.chunkInfo[c] = (uint8_t)blockInfo(M, bx0, bx1, by0, by1);
  }
  return M.chunkInfo[c];
}

// True if any cloud cell under the n cloud samples is non-empty (or, as for terrainReach,
// "maybe" when more than maxBlocks blocks are touched).
RW_INLINE bool cloudReach(const March& M, uint32_t cx, uint32_t cy, uint32_t cstx, uint32_t csty, int n,
                          int maxBlocks) {
  int bx0, bx1, by0, by1;
  blockRange(cx, cstx, n, CBLK_SHIFT, bx0, bx1);
  blockRange(cy, csty, n, CBLK_SHIFT, by0, by1);
  if ((bx1 - bx0 + 1) * (by1 - by0 + 1) > maxBlocks) return true;
  for (int by = by0; by <= by1; ++by) {
    const uint8_t* row = M.cblk + (by & (CBLK_N - 1)) * CBLK_N;
    for (int bx = bx0; bx <= bx1; ++bx)
      if (row[bx & (CBLK_N - 1)]) return true;
  }
  return false;
}

// Terrain samples for columns [x0, x0 + CHUNK), in two passes so the sampling loop does not
// spill: terrainLookup looks up every column and stores its cell byte in cs[i] when the terrain
// there rises above colY (else NO_DRAW); terrainDraw draws those. wx/wy: terrain walk (Q16.16
// cells) of column x0; swx/swy: cloud-shadow walk of column x0. Finished columns (colY <= lim)
// need no test: their colY is at or above every row terrain can still reach.
constexpr uint16_t NO_DRAW = 0xFFFF;
RW_INLINE bool terrainLookup(const March& M, int x0, uint32_t ax, uint32_t ay, uint16_t* cs, bool useMip) {
  const uint32_t stx = M.stx, sty = M.sty;
  const uint8_t* const macro = M.macro;
  const uint8_t* const cells = useMip ? M.mips : M.stamps;
  const int* const rowNow = M.rowNow;
  const int16_t* const colY = M.colY + x0;
  uint32_t all = NO_DRAW;
  for (int i = 0; i < CHUNK; ++i, ax += stx, ay += sty) {
    uint32_t c;
    if (useMip) {
      const uint32_t cellX = (ax >> 17) & (cfg::WORLD_CELLS / 2 - 1);
      const uint32_t cellY = (ay >> 17) & (cfg::WORLD_CELLS / 2 - 1);
      const uint32_t tile = macro[(cellY >> 5) * cfg::MACRO_N + (cellX >> 5)];
      if (tile == cfg::MACRO_OCEAN) {
        c = CELL_DEEP;
      } else {
        uint32_t lx = cellX & (cfg::MIP_N - 1), ly = cellY & (cfg::MIP_N - 1);
        if (tile & cfg::MACRO_TRANSPOSE) { const uint32_t t = lx; lx = ly; ly = t; }
        c = cells[(tile & 15) * (cfg::MIP_N * cfg::MIP_N) + (ly << 5) + lx];
      }
    } else {
      const uint32_t cellX = (ax >> 16) & (cfg::WORLD_CELLS - 1);
      const uint32_t cellY = (ay >> 16) & (cfg::WORLD_CELLS - 1);
      const uint32_t tile = macro[(cellY >> 6) * cfg::MACRO_N + (cellX >> 6)];
      if (tile == cfg::MACRO_OCEAN) {
        c = CELL_DEEP;
      } else {
        uint32_t lx = cellX & (cfg::STAMP_N - 1), ly = cellY & (cfg::STAMP_N - 1);
        if (tile & cfg::MACRO_TRANSPOSE) { const uint32_t t = lx; lx = ly; ly = t; }
        c = cells[(tile & 15) * (cfg::STAMP_N * cfg::STAMP_N) + (ly << 6) + lx];
      }
    }
    const uint32_t v = rowNow[c >> 4] < colY[i] ? c : NO_DRAW;
    cs[i] = (uint16_t)v;
    all &= v;
  }
  return all != NO_DRAW;
}

// Same result as terrainLookup when all CHUNK samples lie in one macro tile (the usual case: a
// tile is 256 u, a chunk segment at most ~72 u): the stamp (or mip) pointer and the transpose
// are fixed, so each sample is two shifts and one load. Returns -1 if the samples span tiles.
RW_INLINE int terrainLookupTile(const March& M, int x0, uint32_t ax, uint32_t ay, uint16_t* cs, bool useMip) {
  uint32_t stx = M.stx, sty = M.sty;
  const uint32_t n1 = (uint32_t)(CHUNK - 1);
  const uint32_t tx = (ax >> 22) & (cfg::MACRO_N - 1), ty = (ay >> 22) & (cfg::MACRO_N - 1);
  if ((((ax + stx * n1) >> 22) & (cfg::MACRO_N - 1)) != tx || (((ay + sty * n1) >> 22) & (cfg::MACRO_N - 1)) != ty)
    return -1;
  const uint32_t tile = M.macro[ty * cfg::MACRO_N + tx];
  const int* const rowNow = M.rowNow;
  const int16_t* const colY = M.colY + x0;
  uint32_t all = NO_DRAW;
  if (tile == cfg::MACRO_OCEAN) {
    const int row0 = rowNow[0];
    for (int i = 0; i < CHUNK; ++i) {
      const uint32_t v = row0 < colY[i] ? (uint32_t)CELL_DEEP : NO_DRAW;
      cs[i] = (uint16_t)v;
      all &= v;
    }
    return all != NO_DRAW ? 1 : 0;
  }
  // index = (ly << rowShift) | lx with (lx, ly) from (ax, ay), swapped for transposed tiles
  if (tile & cfg::MACRO_TRANSPOSE) {
    uint32_t t = ax; ax = ay; ay = t;
    t = stx; stx = sty; sty = t;
  }
  const int sh = useMip ? 17 : 16;
  const uint32_t mask = useMip ? (uint32_t)(cfg::MIP_N - 1) : (uint32_t)(cfg::STAMP_N - 1);
  const int rowShift = useMip ? 5 : 6;
  const uint8_t* const cells = useMip ? M.mips + (tile & 15) * (cfg::MIP_N * cfg::MIP_N)
                                      : M.stamps + (tile & 15) * (cfg::STAMP_N * cfg::STAMP_N);
  for (int i = 0; i < CHUNK; ++i, ax += stx, ay += sty) {
    const uint32_t c = cells[(((ay >> sh) & mask) << rowShift) | ((ax >> sh) & mask)];
    const uint32_t v = rowNow[c >> 4] < colY[i] ? c : NO_DRAW;
    cs[i] = (uint16_t)v;
    all &= v;
  }
  return all != NO_DRAW ? 1 : 0;
}

// Fills rows [y0, y1) of one column going up from row y1 - 1 (p, d point at that row) with idx
// where the pixel is still EMPTY. Returns the number of pixels written.
RW_INLINE uint32_t fillUp(uint8_t*& p, uint8_t*& d, int n, uint8_t idx, uint8_t dq) {
  uint32_t f = 0;
  for (int k = 0; k < n; ++k, p -= LW, d -= LW) {
    if (*p != pal::EMPTY) continue;
    *p = idx;
    *d = dq;
    ++f;
  }
  return f;
}

// Fast path of terrainDraw for the common sea / flat case: every column of the chunk draws, at
// the same level and from the same colY. The rows are then the same for all 8 columns, so they
// are written as two 32-bit words per row (write-if-EMPTY merged branch-free) instead of 8
// column walks; glitter-path rows override single bytes. Same pixels and depths as the general
// path.
#if defined(__GNUC__) || defined(__clang__)
#define RW_ALIGNED4(p) static_cast<uint8_t*>(__builtin_assume_aligned((p), 4))
#else
#define RW_ALIGNED4(p) (p)
#endif

// Merges 4 palette bytes into a word of px where the byte is EMPTY (0), and dq into the same
// bytes of the depth word. Returns the number of bytes written.
RW_INLINE uint32_t mergeWord(uint8_t* p, uint8_t* d, uint32_t idx4, uint32_t dq4) {
  uint32_t w;
  w = ld32(p);
  if (w == 0u) {
    st32(p, idx4);
    st32(d, dq4);
    return 4;
  }
  const uint32_t z = ~(((w & 0x7F7F7F7Fu) + 0x7F7F7F7Fu) | w) & 0x80808080u;  // bit 7: byte == 0
  if (!z) return 0;
  const uint32_t m = (z - (z >> 7)) | z;  // 0xFF in the EMPTY bytes
  uint32_t dw;
  dw = ld32(d);
  w |= idx4 & m;
  dw = (dw & ~m) | (dq4 & m);
  st32(p, w);
  st32(d, dw);
  return ((z >> 7) * 0x01010101u) >> 24;
}

// Draws the uniform chunk: all columns start at cy0 and sample level lvl. cs == nullptr means
// every sample is CELL_DEEP (ocean chunk). The per-column indices are built in short separate
// passes (base, foam / glints, cloud shadow, glitter path) so each loop stays in registers.
RW_INLINE void drawRows(March& M, int x0, const uint16_t* cs, uint32_t lvl, int cy0, uint32_t wx, uint32_t wy,
                        uint32_t swx, uint32_t swy, bool useMip) {
  alignas(4) uint8_t top[CHUNK];
  alignas(4) uint8_t side[CHUNK];
  alignas(4) uint8_t path[CHUNK];
  int pathStart[CHUNK];  // first (lowest) row index of this column's glitter-path rows
  int pathMin = NO_PATH;
  uint32_t samples = 0;
  const int sh = useMip ? 17 : 16;  // Q16.16 walk -> (mip) cell
  // ---- base indices
  bool anyWater;
  if (!cs) {
    const uint32_t t4 = 0x01010101u * M.topIdx[CELL_DEEP], s4 = 0x01010101u * M.sideIdx[CELL_DEEP];
    st32(top, t4);
    st32(top + 4, t4);
    st32(side, s4);
    st32(side + 4, s4);
    anyWater = true;
  } else {
    anyWater = false;
    for (int i = 0; i < CHUNK; ++i) {
      const int mt = cs[i] & 15;
      top[i] = M.topIdx[mt];
      side[i] = M.sideIdx[mt];
      anyWater |= mt <= mat::SURF;
    }
  }
  // ---- foam / glints on near water (same for every row of a sample)
  if (anyWater && M.nearFx) {
    uint32_t ax = wx, ay = wy;
    const uint32_t stx = M.stx, sty = M.sty;
    const int t6 = M.t6;
    for (int i = 0; i < CHUNK; ++i, ax += stx, ay += sty) {
      const int mt = cs ? cs[i] & 15 : (int)CELL_DEEP;
      if (mt > mat::SURF) continue;
      const int ny = (int)(ay >> sh) & 63;
      if (mt == mat::SURF && ((gNoise64[(ny << 6) | ((int)(ax >> sh) & 63)] + M.t36) & 255) < 110) {
        top[i] = pal::FOAM;
        continue;
      }
      const int qx = (int)(ax >> 14);
      const int n1 = gNoise64[(ny << 6) | ((qx + t6) & 63)];
      if (n1 > 251) {  // sparse: denser thresholds read as snow on the 120x68 frame
        if (n1 > 254) top[i] = pal::GLINT_B;
        else if (gNoise64[((((int)(ay >> sh) + 17) & 63) << 6) | ((qx - M.t4) & 63)] > 128) top[i] = pal::GLINT_A;
      }
    }
  }
  // ---- cloud shadow on top faces (day / sunset, fog <= 2)
  if (M.doShadow) {
    uint32_t sx = swx, sy = swy;
    const uint32_t cstx = M.cstx, csty = M.csty;
    const uint8_t* const clouds = M.clouds;
    const uint8_t* const shade = M.shade;
    for (int i = 0; i < CHUNK; ++i, sx += cstx, sy += csty) {
      const int k = clouds[(((sy >> 16) & (cfg::CLOUD_N - 1)) << 7) | ((sx >> 16) & (cfg::CLOUD_N - 1))];
      if (k > 1) top[i] = shade[top[i]];
    }
    samples += CHUNK;
  }
  // ---- glitter path: rows y >= moonYMin of water columns; one hash per sample
  const uint32_t zero = 0;
  st32(path, zero);
  st32(path + 4, zero);
  for (int i = 0; i < CHUNK; ++i) pathStart[i] = NO_PATH;
  if (anyWater) {
    for (int i = 0; i < CHUNK; ++i) {
      const int ym = M.moonYMin[x0 + i];
      if (ym >= cy0) continue;
      if (cs && (cs[i] & 15) > mat::SURF) continue;
      const uint32_t ax = wx + M.stx * (uint32_t)i, ay = wy + M.sty * (uint32_t)i;
      const int cellX = useMip ? (int)(ax >> 17) & (cfg::WORLD_CELLS / 2 - 1) : (int)(ax >> 16) & (cfg::WORLD_CELLS - 1);
      const int cellY = useMip ? (int)(ay >> 17) & (cfg::WORLD_CELLS / 2 - 1) : (int)(ay >> 16) & (cfg::WORLD_CELLS - 1);
      const int h = (int)(hash3i(cellX, cellY, M.t8) & 255u);
      int pathIdx;
      if (M.nearFx) pathIdx = h < 40 ? pal::GLINT_B : h < 90 ? pal::GLINT_A : h < 150 ? pal::MOONPATH : 0;
      else pathIdx = h < 25 ? pal::GLINT_A : h < 110 ? pal::MOONPATH : 0;
      if (pathIdx > 0) {
        path[i] = (uint8_t)pathIdx;
        pathStart[i] = ym;
        if (ym < pathMin) pathMin = ym;
      }
    }
  }
  uint32_t top0, top1, side0, side1, path0, path1;
  top0 = ld32(top);
  top1 = ld32(top + 4);
  side0 = ld32(side);
  side1 = ld32(side + 4);
  path0 = ld32(path);
  path1 = ld32(path + 4);
  const uint32_t dq4 = 0x01010101u * M.dq;
  const int yTop = M.rowNow[lvl];
  const int y0 = yTop < 0 ? 0 : yTop;
  const int yWall = M.rowPrev[lvl];
  uint32_t fills = 0;
  uint8_t* p = RW_ALIGNED4(M.px + (cy0 - 1) * LW + x0);
  uint8_t* d = RW_ALIGNED4(M.dp + (cy0 - 1) * LW + x0);
  for (int y = cy0 - 1; y >= y0; --y, p -= LW, d -= LW) {
    const bool wall = y >= yWall;
    uint32_t i0 = wall ? side0 : top0, i1 = wall ? side1 : top1;
    if (y >= pathMin) {  // glitter-path bytes of the columns whose path covers this row
      alignas(4) uint8_t pm[CHUNK];
      for (int i = 0; i < CHUNK; ++i) pm[i] = y >= pathStart[i] ? 0xFF : 0x00;
      uint32_t m0, m1;
      m0 = ld32(pm);
      m1 = ld32(pm + 4);
      i0 = (i0 & ~m0) | (path0 & m0);
      i1 = (i1 & ~m1) | (path1 & m1);
    }
    fills += mergeWord(p, d, i0, dq4);
    fills += mergeWord(p + 4, d + 4, i1, dq4);
  }
  int16_t* const cy = M.colY + x0;
  for (int i = 0; i < CHUNK; ++i) cy[i] = (int16_t)y0;
  M.samples += samples;
  M.fills += fills;
}

// Fast path of terrainDraw: returns false (nothing drawn) when the chunk does not qualify.
RW_INLINE bool drawUniform(March& M, int x0, const uint16_t* cs, uint32_t wx, uint32_t wy, uint32_t swx,
                           uint32_t swy, bool useMip) {
  const int16_t* const colY = M.colY + x0;
  const int cy0 = colY[0];
  const uint32_t lvl = (uint32_t)cs[0] >> 4;
  for (int i = 0; i < CHUNK; ++i) {
    if (cs[i] == NO_DRAW || ((uint32_t)cs[i] >> 4) != lvl || colY[i] != cy0) return false;
  }
  drawRows(M, x0, cs, lvl, cy0, wx, wy, swx, swy, useMip);
  return true;
}

JET_NOINLINE void JET_HOT terrainDraw(March& M, int x0, const uint16_t* cs, uint32_t wx,
                                      uint32_t wy, uint32_t swx, uint32_t swy, bool useMip) {
  if (M.wordOk && drawUniform(M, x0, cs, wx, wy, swx, swy, useMip)) return;
  uint32_t samples = 0, fills = 0;
  for (int i = 0; i < CHUNK; ++i) {
    const uint32_t c = cs[i];
    if (c == NO_DRAW) continue;
    const int x = x0 + i;
    const int lvl = (int)(c >> 4), mt = (int)(c & 15);
    const int cy0 = M.colY[x];
    const int yTop = M.rowNow[lvl];
    const int y0 = yTop < 0 ? 0 : yTop;
    const int yWall = M.rowPrev[lvl];
    uint8_t topIdx = M.topIdx[mt];
    const uint8_t sideIdx = M.sideIdx[mt];
    int ym = NO_PATH;
    int cellX = 0, cellY = 0;
    if (mt <= mat::SURF) {  // water: foam / glints near, glitter path rows
      ym = M.moonYMin[x];
      if (M.nearFx || ym < cy0) {
        const uint32_t ax = wx + M.stx * (uint32_t)i, ay = wy + M.sty * (uint32_t)i;
        cellX = useMip ? (int)(ax >> 17) & (cfg::WORLD_CELLS / 2 - 1) : (int)(ax >> 16) & (cfg::WORLD_CELLS - 1);
        cellY = useMip ? (int)(ay >> 17) & (cfg::WORLD_CELLS / 2 - 1) : (int)(ay >> 16) & (cfg::WORLD_CELLS - 1);
        if (M.nearFx) {  // same for every row of this sample
          const int ny = cellY & 63;
          if (mt == mat::SURF && ((gNoise64[(ny << 6) | (cellX & 63)] + M.t36) & 255) < 110) {
            topIdx = pal::FOAM;
          } else {
            const int qx = (int)(ax >> 14);
            const int n1 = gNoise64[(ny << 6) | ((qx + M.t6) & 63)];
            if (n1 > 251) {  // sparse: denser thresholds read as snow on the 120x68 frame
              if (n1 > 254) topIdx = pal::GLINT_B;
              else if (gNoise64[(((cellY + 17) & 63) << 6) | ((qx - M.t4) & 63)] > 128) topIdx = pal::GLINT_A;
            }
          }
        }
      }
    }
    // Cloud shadow on top faces (day / sunset, fog <= 2).
    if (M.doShadow) {
      const uint32_t sx = swx + M.cstx * (uint32_t)i, sy = swy + M.csty * (uint32_t)i;
      const int k = M.clouds[(((sy >> 16) & (cfg::CLOUD_N - 1)) << 7) | ((sx >> 16) & (cfg::CLOUD_N - 1))];
      ++samples;
      if (k > 1) topIdx = M.shade[topIdx];
    }
    const uint8_t dq = M.dq;
    const int off = (cy0 - 1) * LW + x;
    uint8_t* p = M.px + off;
    uint8_t* d = M.dp + off;
    int y = cy0;  // next row to draw is y - 1
    if (ym < cy0) {
      // Glitter path rows (y >= ym) of this sample: one hash decides their index.
      const int h = (int)(hash3i(cellX, cellY, M.t8) & 255u);
      int pathIdx;
      if (M.nearFx) pathIdx = h < 40 ? pal::GLINT_B : h < 90 ? pal::GLINT_A : h < 150 ? pal::MOONPATH : 0;
      else pathIdx = h < 25 ? pal::GLINT_A : h < 110 ? pal::MOONPATH : 0;
      const int yStop = ym > y0 ? ym : y0;
      if (pathIdx > 0) {
        fills += fillUp(p, d, y - yStop, (uint8_t)pathIdx, dq);
        y = yStop;
      }
    }
    // Wall rows (y >= yWall), then top-face rows.
    const int yW = yWall > y0 ? yWall : y0;
    if (y > yW) {
      fills += fillUp(p, d, y - yW, sideIdx, dq);
      y = yW;
    }
    fills += fillUp(p, d, y - y0, topIdx, dq);
    M.colY[x] = (int16_t)y0;
  }
  M.samples += samples;
  M.fills += fills;
}

// Chunk entirely over CELL_DEEP blocks: no lookups, every sample is level 0 deep water.
JET_NOINLINE bool JET_HOT terrainOcean(March& M, int x0, uint32_t wx, uint32_t wy, uint32_t swx, uint32_t swy,
                                       bool useMip) {
  uint16_t cs[CHUNK];
  const int row0 = M.rowNow[0];
  const int16_t* const colY = M.colY + x0;
  if (M.wordOk) {  // usual case: the sea row advanced over a chunk with one common colY
    const int cy0 = colY[0];
    bool uni = row0 < cy0;
    for (int i = 1; i < CHUNK && uni; ++i) uni = colY[i] == cy0;
    if (uni) {
      drawRows(M, x0, nullptr, 0, cy0, wx, wy, swx, swy, useMip);
      return true;
    }
  }
  uint32_t all = NO_DRAW;
  for (int i = 0; i < CHUNK; ++i) {
    const uint32_t v = row0 < colY[i] ? (uint32_t)CELL_DEEP : NO_DRAW;
    cs[i] = (uint16_t)v;
    all &= v;
  }
  if (all == NO_DRAW) return false;
  terrainDraw(M, x0, cs, wx, wy, swx, swy, useMip);
  return true;
}

// Returns true if any column's occlusion row moved.
JET_NOINLINE bool JET_HOT terrainFull(March& M, int x0, uint32_t wx, uint32_t wy, uint32_t swx, uint32_t swy) {
  uint16_t cs[CHUNK];
  M.samples += CHUNK;
  int any = terrainLookupTile(M, x0, wx, wy, cs, false);
  if (any < 0) any = terrainLookup(M, x0, wx, wy, cs, false) ? 1 : 0;
  if (!any) return false;
  terrainDraw(M, x0, cs, wx, wy, swx, swy, false);
  return true;
}
JET_NOINLINE bool JET_HOT terrainMip(March& M, int x0, uint32_t wx, uint32_t wy, uint32_t swx, uint32_t swy) {
  uint16_t cs[CHUNK];
  M.samples += CHUNK;
  int any = terrainLookupTile(M, x0, wx, wy, cs, true);
  if (any < 0) any = terrainLookup(M, x0, wx, wy, cs, true) ? 1 : 0;
  if (!any) return false;
  terrainDraw(M, x0, cs, wx, wy, swx, swy, true);
  return true;
}

// Cloud samples for columns [x0, x0 + CHUNK) (two passes, as above). cx/cy: cloud walk of x0.
JET_NOINLINE void JET_HOT cloudColumns(March& M, int x0, uint32_t cx, uint32_t cy) {
  uint8_t ks[CHUNK], kps[CHUNK];
  uint32_t mask = 0, samples = 0;
  {
    const uint8_t* const clouds = M.clouds;
    const int16_t* const colY = M.colY + x0;
    uint8_t* const prevK = M.cloudPrevK + x0;
    const uint32_t cstx = M.cstx, csty = M.csty;
    const int lo = M.cloudLo;
    const bool prevOk = M.cloudPrevSampled;
    for (int i = 0; i < CHUNK; ++i, cx += cstx, cy += csty) {
      if (lo >= colY[i]) {  // everything this step could draw is already covered
        prevK[i] = 0;
        continue;
      }
      const int k = clouds[(((cy >> 16) & (cfg::CLOUD_N - 1)) << 7) | ((cx >> 16) & (cfg::CLOUD_N - 1))];
      ++samples;
      const int kp = prevOk ? prevK[i] : 0;
      prevK[i] = (uint8_t)k;
      if (k == 0) continue;
      ks[i] = (uint8_t)k;
      kps[i] = (uint8_t)kp;
      mask |= 1u << i;
    }
  }
  M.samples += samples;
  if (!mask) return;
  uint8_t* const px = M.px;
  uint8_t* const dp = M.dp;
  const uint8_t dq = M.dq;
  uint32_t fills = 0;
  for (int i = 0; i < CHUNK; ++i) {
    if (!((mask >> i) & 1u)) continue;
    const int x = x0 + i, k = ks[i], kp = kps[i];
    if (k > kp) fills += fillSpan(px, dp, x, M.cPrev[k], M.cPrev[kp], M.cIdxSide, dq, false);
    if ((M.cTopMask >> k) & 1u) fills += fillSpan(px, dp, x, M.cNow[k], M.cPrev[k], M.cIdxTop, dq, k == 1);
    if (M.cBelow) fills += fillSpan(px, dp, x, M.cPrev[0], M.cNow[0], M.cIdxUnder, dq, k == 1);
  }
  M.fills += fills;
}

}  // namespace

void worldRenderInit(uint32_t seed) {
  const uint32_t s = hash32(seed ^ 0x0F0A3D17u);
  for (int i = 0; i < 64 * 64; ++i) gNoise64[i] = (uint8_t)(hash32(s + (uint32_t)i * 0x9E3779B1u) >> 24);
}

void JET_HOT renderWorld(FrameBuf& fb, const View& v, const World& w, Tod tod, const SeaFx& sea,
                         RenderStats& st) {
  March M;  // on the stack: the column loops address it through a register
  M.px = fb.px;
  M.dp = fb.depth;
  M.wordOk = (((uintptr_t)fb.px | (uintptr_t)fb.depth) & 3u) == 0u;
  M.macro = w.macro();
  M.stamps = w.stamps();
  M.mips = w.mips();
  M.clouds = w.clouds();
  M.shade = shadeTable();
  M.tblk = worldTerrainBlocks();
  M.cblk = worldCloudBlocks();

  const float S = v.S, dist = v.dist, q = v.q, hz = v.horizon, Cx = v.Cx;
  const float cps = v.cpsi, sps = v.spsi;
  const float ez = v.alt + v.T * dist;  // eye height (same as project())
  const float denMin = cfg::MARCH_START_DEN;
  const float viewDist = v.viewDist > dist * (denMin + 0.05f) ? v.viewDist : dist * (denMin + 0.05f);
  const float denMax = viewDist / dist;
  const float fogStart = v.fogStart < viewDist - 1.0f ? v.fogStart : viewDist - 1.0f;
  const float fogInv = 1.0f / (viewDist - fogStart);
  const float ZT = (float)cfg::ZTOP;
  const float CB = cfg::CLOUD_BASE, CT = cfg::CLOUD_TOP, CL = cfg::CLOUD_LAYER_U;

  // ---- where the march can start
  const float BIG = 1.0e9f;
  bool terrainOn = true;
  float denT = denMin;
  if (ez > ZT) {
    if (hz >= (float)LH) terrainOn = false;  // all terrain is below the eye and below the frame
    else denT = S * (ez - ZT) / ((float)LH - hz);
  }
  bool cloudOn = true;
  float denC = denMin;
  if (ez < CB) {
    if (hz > 0.0f) denC = S * (CB - ez) / hz;
    else cloudOn = false;  // the whole slab projects above the top row
  } else if (ez > CT) {
    if (hz < (float)LH) denC = S * (ez - CT) / ((float)LH - hz);
    else cloudOn = false;
  }
  float den0 = BIG;
  if (terrainOn && denT < den0) den0 = denT;
  if (cloudOn && denC < den0) den0 = denC;
  if (den0 < denMin) den0 = denMin;
  if (den0 >= denMax || (!terrainOn && !cloudOn)) return;

  // ---- per-frame tables
  float ezh[NLEV];
  for (int l = 0; l < NLEV; ++l) ezh[l] = ez - (float)cfg::HEIGHT_U[l];
  float ezc[NCL];
  for (int j = 0; j < NCL; ++j) ezc[j] = ez - (CB + CL * (float)j);
  // Farthest row tall terrain can reach (eye above ZTOP: rows of ZTOP fall toward the horizon).
  const int rowFar15 = rowOf(hz + (S / denMax) * ezh[NLEV - 1]);

  for (int x = 0; x < LW; ++x) {
    M.colY[x] = (int16_t)LH;
    M.cloudPrevK[x] = 0;
  }
  int remaining = terrainOn ? NCHUNK : 0;  // chunks with a column that can still change
  for (int c = 0; c < NCHUNK; ++c) {
    M.chunkMaxY[c] = (int16_t)LH;
    M.chunkDone[c] = terrainOn ? 0 : 1;
    M.chunkKey[c] = NO_KEY;
  }

  // Glitter path: |x - moonX| < 3 + 0.25 (y - horizon)  <=>  y > horizon + 4(|x - moonX| - 3).
  const bool pathOn = sea.moonPath && sea.moonX > -500;
  for (int x = 0; x < LW; ++x) {
    int ym = NO_PATH;
    if (pathOn) {
      int ax = x - sea.moonX;
      if (ax < 0) ax = -ax;
      const float lim = hz + 4.0f * (float)(ax - 3);
      ym = rowOf(lim) + 1;
      if (ym < 0) ym = 0;
    }
    M.moonYMin[x] = (int16_t)(ym > NO_PATH ? NO_PATH : ym);
  }

  const bool shadowsOn = sea.cloudShadows && tod != Tod::Night;
  const float shK = cfg::SHADOW_K[(int)tod] * CB;  // ground offset of a cloud's shadow (u)
  const uint32_t shOff = toFix(shK * (65536.0f / cfg::CLOUD_CELL_U));
  const float t = sea.time;
  M.t36 = (int)(36.0f * t);
  M.t6 = (int)(6.0f * t);
  M.t4 = (int)(4.0f * t);
  M.t8 = (int)(8.0f * t);
  const float altStep = v.alt > 8.0f ? v.alt : 8.0f;
  float stepDen = v.T + q * altStep;
  if (stepDen < 0.02f) stepDen = 0.02f;
  const float stepK = 1.0f / (S * stepDen);
  const float FX = 65536.0f / cfg::CELL_U, FC = 65536.0f / cfg::CLOUD_CELL_U;
  const float cfx = v.fx - w.cloudOffX, cfy = v.fy - w.cloudOffY;
  const bool below = ez < CB;
  // A cloud of thickness k shows its top face when the eye is above CB + CL*k (bit k).
  M.cTopMask = 0;
  for (int k = 0; k < NCL; ++k)
    if (ez > CB + CL * (float)k) M.cTopMask |= 1u << k;
  M.cBelow = below;
  const int hzRow = rowOf(hz);

  // ---- rows at the starting depth become the "previous" rows
  float den = den0;
  {
    const float m0 = S / den;
    for (int l = 0; l < NLEV; ++l) M.rowPrev[l] = rowOf(hz + m0 * ezh[l]);
    for (int j = 0; j < NCL; ++j) M.cPrev[j] = rowOf(hz + m0 * ezc[j]);
  }
  M.cloudPrevSampled = false;
  float lastCloudZc = 0.0f;
  const float sInv = 1.0f / (S > 0.4f ? S : 0.4f);
  const float invS = 1.0f / S;
  bool terrainLive = terrainOn && M.rowPrev[NLEV - 1] < LH;

  // Fog level: (int)(7 sqrt(ff) + 0.5), ff non-decreasing along the march. The exact formula is
  // only evaluated once ff gets near the next level's threshold.
  int fog = 0;
  float ffNext = -1.0f;
  int fogTab = -1;  // fog level the topIdx / sideIdx tables were built for

  uint32_t steps = 0;
  M.samples = 0;
  M.fills = 0;
  int clo = 0, chi = NCHUNK - 1;  // live chunk range
  while ((int)steps < cfg::MARCH_MAX_STEPS) {
    // ---- advance (dv in u along the view direction)
    float dv = den * den * stepK;
    const float dvMax = cfg::MARCH_DV_MAXK * den * invS;
    if (dv > dvMax) dv = dvMax;     // far field: at most 2.5 px of lateral travel per step
    const float dvMin = (terrainLive ? DV_MIN_U : DV_MIN_CLOUD_U) * sInv;
    if (dv < dvMin) dv = dvMin;  // near field (dvMax < dvMin there): the minimum wins
    den += dv * DV_SCALE * q;
    if (den > denMax) break;
    ++steps;

    const float invDen = 1.0f / den;  // the one divide of this step
    const float m = S * invDen;
    const float zc = dist * den;
    const float vv = zc - dist;
    for (int l = 0; l < NLEV; ++l) M.rowNow[l] = rowOf(hz + m * ezh[l]);
    for (int j = 0; j < NCL; ++j) M.cNow[j] = rowOf(hz + m * ezc[j]);
    {
      float ff = (zc - fogStart) * fogInv;
      ff = ff < 0.0f ? 0.0f : (ff > 1.0f ? 1.0f : ff);
      if (ff >= ffNext) {
        fog = (int)(7.0f * sqrtf(ff) + 0.5f);
        if (fog > 7) fog = 7;
        const float th = ((float)fog + 0.5f) * (1.0f / 7.0f);
        ffNext = fog < 7 ? th * th * 0.9999f : 2.0f;
      }
    }
    if (fog != fogTab) {
      fogTab = fog;
      const int off = pal::WORLD0 + fog * pal::WORLD_BASES;
      for (int i = 0; i < 16; ++i) {
        M.topIdx[i] = (uint8_t)(off + TOP_BASE[i]);
        M.sideIdx[i] = (uint8_t)(off + SIDE_BASE[i]);
      }
    }
    const int fogOff = pal::WORLD0 + fog * pal::WORLD_BASES;
    M.dq = depthQ(zc);
    const float pxU = den * invS;  // world u per pixel across
    const float a0 = (0.5f - Cx) * pxU;

    // ---- terrain
    const bool doTerrain = remaining > 0 && M.rowNow[NLEV - 1] < LH;
    const bool cloudRows =
        cloudOn && !(M.cNow[0] < 0 && M.cPrev[0] < 0) && !(M.cNow[NCL - 1] >= LH && M.cPrev[NCL - 1] >= LH);
    const float cloudGap = zc - lastCloudZc;
    const bool cloudStep =
        cloudRows && (!M.cloudPrevSampled ||
                      (cloudGap >= CLOUD_MIN_GAP_U * sInv && (!doTerrain || cloudGap >= 1.9f * dv * DV_SCALE)));
    M.doShadow = shadowsOn && fog <= 2 && doTerrain;
    uint32_t cwx = 0, cwy = 0, cstx = 0, csty = 0;
    if (cloudStep || M.doShadow) {
      cwx = toFix((cfx + a0 * cps + vv * sps) * FC);
      cwy = toFix((cfy + a0 * sps - vv * cps) * FC);
      cstx = toFix(cps * pxU * FC);
      csty = toFix(sps * pxU * FC);
    }
    M.cstx = cstx;
    M.csty = csty;
    if (doTerrain) {
      int lim = M.rowNow[NLEV - 1] < rowFar15 ? M.rowNow[NLEV - 1] : rowFar15;
      if (lim < 0) lim = 0;
      M.lim = lim;
      const bool useMip = m < cfg::MIP_M_THRESHOLD;
      M.nearFx = zc < cfg::NEAR_FX_DIST;
      const uint32_t wx = toFix((v.fx + a0 * cps + vv * sps) * FX);
      const uint32_t wy = toFix((v.fy + a0 * sps - vv * cps) * FX);
      const uint32_t stx = toFix(cps * pxU * FX), sty = toFix(sps * pxU * FX);
      M.stx = stx;
      M.sty = sty;
      const uint32_t swx = cwx - shOff, swy = cwy - shOff;
      const int row15 = M.rowNow[NLEV - 1], row0 = M.rowNow[0];
      // One block test for the whole live range first (cheap while the segment is short): if
      // no terrain under it reaches the highest colY, every chunk the sea cannot reach is idle.
      int rowReach = -1;  // -1 not tested yet, 0 nothing reachable, 1 maybe
      for (int c = clo; c <= chi; ++c) {
        if (M.chunkDone[c]) continue;
        int my = M.chunkMaxY[c];
        if (my <= lim) {  // every column finished
          M.chunkDone[c] = 1;
          --remaining;
          continue;
        }
        if (row15 >= my) continue;  // even the tallest terrain is at or below every colY
        // Q16 accumulators are linear in x (uint32 wraps).
        const uint32_t ux = (uint32_t)(c * CHUNK);
        const uint32_t cwxc = wx + stx * ux, cwyc = wy + sty * ux;
        if (row0 >= my) {  // the sea cannot reach this chunk: only terrain under it can
          if (rowReach < 0) {
            int maxYAll = 0;  // colY only decreases, so this bounds every later chunk too
            for (int k = clo; k <= chi; ++k)
              if (!M.chunkDone[k] && M.chunkMaxY[k] > maxYAll) maxYAll = M.chunkMaxY[k];
            const uint32_t u0 = (uint32_t)(clo * CHUNK);
            rowReach = terrainReach(M, wx + stx * u0, wy + sty * u0, stx, sty, (chi - clo + 1) * CHUNK, maxYAll)
                           ? 1 : 0;
          }
          if (rowReach == 0) continue;
        }
        const uint32_t info = chunkInfo(M, c, cwxc, cwyc, stx, sty);
        if (info != 0xFF && M.rowNow[info & 15] >= my) continue;  // nothing under it reaches colY
        const uint32_t cswx = swx + cstx * ux, cswy = swy + csty * ux;
        bool moved;
        if (info == 0x80) moved = terrainOcean(M, c * CHUNK, cwxc, cwyc, cswx, cswy, useMip);
        else if (useMip) moved = terrainMip(M, c * CHUNK, cwxc, cwyc, cswx, cswy);
        else moved = terrainFull(M, c * CHUNK, cwxc, cwyc, cswx, cswy);
        if (moved) {
          my = 0;
          const int16_t* cy = M.colY + c * CHUNK;
          for (int i = 0; i < CHUNK; ++i)
            if (cy[i] > my) my = cy[i];
          M.chunkMaxY[c] = (int16_t)my;
          if (my <= lim) {
            M.chunkDone[c] = 1;
            --remaining;
          }
        }
      }
      while (clo <= chi && M.chunkDone[clo]) ++clo;
      while (chi >= clo && M.chunkDone[chi]) --chi;
    }

    // ---- clouds (write only EMPTY pixels, never move colY)
    if (cloudStep) {
      M.cIdxTop = (uint8_t)(fogOff + pal::B_CLOUD_TOP);
      M.cIdxSide = (uint8_t)(fogOff + pal::B_CLOUD_SIDE);
      M.cIdxUnder = (uint8_t)(fogOff + pal::B_CLOUD_UNDER);
      // Topmost row any part of the slab occupies at this step (rows grow downward).
      int lo = M.cNow[NCL - 1] < M.cPrev[NCL - 1] ? M.cNow[NCL - 1] : M.cPrev[NCL - 1];
      if (M.cNow[0] < lo) lo = M.cNow[0];
      if (M.cPrev[0] < lo) lo = M.cPrev[0];
      M.cloudLo = lo;
      // Whole row first: no cloud under the segment means every column samples k = 0.
      if (!cloudReach(M, cwx, cwy, cstx, csty, LW, 4)) {
        memset(M.cloudPrevK, 0, LW);
      } else {
        for (int c = 0; c < NCHUNK; ++c) {
          const uint32_t ux = (uint32_t)(c * CHUNK);
          const uint32_t cx = cwx + cstx * ux, cy = cwy + csty * ux;
          if (lo >= M.chunkMaxY[c] || !cloudReach(M, cx, cy, cstx, csty, CHUNK, 16)) {
            // every column is covered or samples a clear cell: no fill, previous k becomes 0
            uint8_t* pk = M.cloudPrevK + c * CHUNK;
            for (int i = 0; i < CHUNK; ++i) pk[i] = 0;
            continue;
          }
          cloudColumns(M, c * CHUNK, cx, cy);
        }
      }
      M.cloudPrevSampled = true;
      lastCloudZc = zc;
      for (int j = 0; j < NCL; ++j) M.cPrev[j] = M.cNow[j];
    } else if (!cloudRows) {
      // Slab off-screen at this step: the next sampled step starts fresh (front walls drawn).
      M.cloudPrevSampled = false;
      for (int j = 0; j < NCL; ++j) M.cPrev[j] = M.cNow[j];
    }

    for (int l = 0; l < NLEV; ++l) M.rowPrev[l] = M.rowNow[l];
    terrainLive = doTerrain;

    // ---- early out: terrain finished and no cloud row can still reach an open pixel
    if (remaining == 0) {
      if (!cloudOn) break;
      int maxColY = 0;
      for (int c = 0; c < NCHUNK; ++c)
        if (M.chunkMaxY[c] > maxColY) maxColY = M.chunkMaxY[c];
      const int clo2 = M.cNow[NCL - 1] < hzRow ? M.cNow[NCL - 1] : hzRow;
      const int chi2 = M.cNow[0] > hzRow ? M.cNow[0] : hzRow;
      if (chi2 < 0 || clo2 >= maxColY) break;
    }
  }

  st.steps += steps;
  st.samples += M.samples;
  st.fills += M.fills;
}

}  // namespace kj
