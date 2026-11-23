// expand.cpp - 2x nearest-neighbour palette expand, 120x68 indices -> 240x135 swapped RGB565.
// OWNER: group A. Shared by the device present path and the simulator PNG path.
//
// lut2x[i] = (c << 16) | c with c = byte-swapped RGB565, so ONE 32-bit store writes the two
// horizontally doubled output pixels (both halves are equal, so host endianness does not
// matter). The second output row of each logical row is a 480 B memcpy of the first.
// Logical row 67 emits only output row 134 (135 = 2*68 - 1).
#include "expand.h"

#include <string.h>

#include "compat.h"
#include "config.h"

namespace kj {

namespace {

constexpr int OUT_W = cfg::SCREEN_W;                 // 240
constexpr int OUT_H = cfg::SCREEN_H;                 // 135
constexpr int IN_W = cfg::LW;                        // 120
constexpr int ROW_BYTES = OUT_W * (int)sizeof(uint16_t);  // 480
static_assert(OUT_W == 2 * IN_W, "expand2x expects a 2x horizontal ratio");
static_assert(OUT_H == 2 * cfg::LH - 1, "expand2x expects 135 = 2*68 - 1 output rows");
static_assert((IN_W & 3) == 0, "row loop is unrolled by 4");

// One output row from one logical row: 120 x 32-bit stores.
inline void expandRow(const uint8_t* src, const uint32_t* lut2x, uint16_t* dstRow) {
  uint32_t* d = reinterpret_cast<uint32_t*>(dstRow);
  for (int x = 0; x < IN_W; x += 4) {
    const uint32_t a = lut2x[src[x + 0]];
    const uint32_t b = lut2x[src[x + 1]];
    const uint32_t c = lut2x[src[x + 2]];
    const uint32_t e = lut2x[src[x + 3]];
    d[x + 0] = a;
    d[x + 1] = b;
    d[x + 2] = c;
    d[x + 3] = e;
  }
}

}  // namespace

void JET_HOT expand2x(const uint8_t* px, const uint32_t* lut2x, uint16_t* out) {
  for (int y = 0; y < cfg::LH; ++y) {
    uint16_t* row0 = out + (2 * y) * OUT_W;
    expandRow(px + y * IN_W, lut2x, row0);
    const int r1 = 2 * y + 1;
    if (r1 < OUT_H) memcpy(row0 + OUT_W, row0, (size_t)ROW_BYTES);
  }
}

void expand2xRows(const uint8_t* px, const uint32_t* lut2x, uint16_t* out, int outRow0,
                  int outRow1) {
  if (outRow0 < 0) outRow0 = 0;
  if (outRow1 > OUT_H) outRow1 = OUT_H;
  for (int r = outRow0; r < outRow1; ++r) {
    uint16_t* dst = out + (r - outRow0) * OUT_W;
    if ((r & 1) != 0 && r > outRow0) {
      memcpy(dst, dst - OUT_W, (size_t)ROW_BYTES);  // odd row: copy of the even row above
    } else {
      expandRow(px + (r >> 1) * IN_W, lut2x, dst);
    }
  }
}

}  // namespace kj
