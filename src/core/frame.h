// frame.h - the logical frame buffer (8-bit palette indices + 8-bit depth) and depth quantiser.
// OWNER: group B. SHARED CONTRACT.
//
// px[y*LW + x]    : palette index, 0 = pal::EMPTY ("not yet written"). Cleared every frame.
// depth[y*LW + x] : quantised camera depth dq(zc), 255 = DEPTH_FAR (nothing). Cleared every frame.
// Depth test rule used by EVERY drawer: a fragment with depth d is visible iff d <= depth[p].
// Opaque geometry (terrain, clouds, rings, jet voxels, flame) writes depth; translucent/dithered
// effects (trails, smoke, spray, glow halos, HUD) test but do not write.
#pragma once
#include <stdint.h>
#include "config.h"

namespace kj {

constexpr int LW = cfg::LW, LH = cfg::LH, LPIX = cfg::LPIX;
constexpr uint8_t DEPTH_FAR = 255;

struct FrameBuf {
  uint8_t* px;     // LPIX bytes
  uint8_t* depth;  // LPIX bytes
};

// zc = distance from the camera along the view direction (u) = dist * den.
// 1 u resolution up to 160 u (jet sits at ~88 +- 30), then 8 u steps; 254 ~ 910 u+.
inline uint8_t depthQ(float zc) {
  if (zc <= 0.0f) return 0;
  if (zc < 160.0f) return (uint8_t)zc;
  int q = 160 + (int)((zc - 160.0f) * 0.125f);
  return (uint8_t)(q > 254 ? 254 : q);
}

inline bool inFrame(int x, int y) { return (unsigned)x < (unsigned)LW && (unsigned)y < (unsigned)LH; }

}  // namespace kj
