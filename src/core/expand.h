// expand.h - 2x nearest-neighbour palette expand of the 120x68 index frame to the 240x135
// big-endian RGB565 panel frame. Used by the device present path AND the simulator PNG path,
// so screenshots show exactly what the panel shows.
// OWNER: group A (expand.cpp).
#pragma once
#include <stdint.h>

namespace kj {

// Full frame: out = 240*135 uint16 (64,800 B), 4-byte aligned. Logical row y -> output rows
// 2y and 2y+1, except logical row 67 which emits only output row 134.
// Per row: 120 x uint32 stores of lut2x[px] then memcpy of the 480 B row. Definition JET_HOT.
void expand2x(const uint8_t* px, const uint32_t* lut2x, uint16_t* out);

// Band variant for the DMA-allocation fallback path: writes output rows [outRow0, outRow1)
// into `out` (which holds (outRow1-outRow0)*240 pixels).
void expand2xRows(const uint8_t* px, const uint32_t* lut2x, uint16_t* out, int outRow0,
                  int outRow1);

}  // namespace kj
