// draw.h - shared 2D/3D drawing primitives on the logical FrameBuf (depth-tested).
// OWNER: group C (draw.cpp). SHARED CONTRACT (used by C, D for rings/HUD helpers).
// alpha16: 0..16 Bayer coverage (16 = solid). Depth test: visible iff dq <= depth[p].
#pragma once
#include <stdint.h>
#include "types.h"
#include "frame.h"
#include "camera.h"

namespace kj {

// Single pixel, depth-tested. writeDepth: also store dq.
void drawPoint(FrameBuf& fb, int x, int y, uint8_t dq, uint8_t color, bool writeDepth);
// Bresenham line with linearly interpolated depth; thick 1 or 2 (2 = also x+1/y+1 neighbour).
// Definition is JET_HOT.
void drawLine(FrameBuf& fb, int x0, int y0, uint8_t dq0, int x1, int y1, uint8_t dq1,
              uint8_t color, int thick, int alpha16, bool writeDepth);
// World-space line: projects both ends (culls if either end is behind den < 0.15).
// Returns the number of pixels attempted.
int drawLine3D(FrameBuf& fb, const View& v, const Vec3& a, const Vec3& b, uint8_t color, int thick,
               int alpha16, bool writeDepth);
// Filled disc (r in px, r <= 0.5 draws one px), Bayer coverage alpha16, constant depth dq.
void drawDisc(FrameBuf& fb, int cx, int cy, float r, uint8_t dq, uint8_t color, int alpha16,
              bool writeDepth);
// Dithered glow ring between r0 and r1 (density falls off outward), no depth write.
void drawGlow(FrameBuf& fb, int cx, int cy, float r0, float r1, uint8_t dq, uint8_t color,
              int alpha16Inner);
// Screen-space rectangles without depth test (HUD / menus).
void drawRect(FrameBuf& fb, int x, int y, int w, int h, uint8_t color);
void drawRectDither(FrameBuf& fb, int x, int y, int w, int h, uint8_t color, int alpha16);

}  // namespace kj
