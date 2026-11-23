// draw.cpp - shared depth-tested drawing primitives on the logical 120x68 frame.
// OWNER: group C. Depth rule (frame.h): a fragment with depth d is visible iff d <= depth[p].
#include "draw.h"

#include <math.h>
#include "compat.h"
#include "mathx.h"

namespace kj {

namespace {

// DRAM copy of BAYER4 (the constexpr header table would be read from flash rodata).
uint8_t s_bayer[16] = {0, 8, 2, 10, 12, 4, 14, 6, 3, 11, 1, 9, 15, 7, 13, 5};

inline bool passAlpha(int x, int y, int a16) {
  return a16 >= 16 || s_bayer[((y & 3) << 2) | (x & 3)] < a16;
}

inline void plotRaw(FrameBuf& fb, int x, int y, uint8_t dq, uint8_t color, int a16, bool wd) {
  if ((unsigned)x >= (unsigned)LW || (unsigned)y >= (unsigned)LH) return;
  if (!passAlpha(x, y, a16)) return;
  int p = y * LW + x;
  if (dq <= fb.depth[p]) {
    fb.px[p] = color;
    if (wd) fb.depth[p] = dq;
  }
}

// Liang-Barsky clip of the segment to [xmin,xmax]x[ymin,ymax]; t0/t1 returned in [0,1].
bool clipSeg(float x0, float y0, float x1, float y1, float xmin, float ymin, float xmax, float ymax,
             float& t0, float& t1) {
  t0 = 0.0f; t1 = 1.0f;
  float dx = x1 - x0, dy = y1 - y0;
  const float p[4] = {-dx, dx, -dy, dy};
  const float q[4] = {x0 - xmin, xmax - x0, y0 - ymin, ymax - y0};
  for (int i = 0; i < 4; ++i) {
    if (p[i] == 0.0f) {
      if (q[i] < 0.0f) return false;
    } else {
      float r = q[i] / p[i];
      if (p[i] < 0.0f) { if (r > t1) return false; if (r > t0) t0 = r; }
      else { if (r < t0) return false; if (r < t1) t1 = r; }
    }
  }
  return true;
}

inline int roundi(float v) { return (int)floorf(v + 0.5f); }

}  // namespace

// Core line routine (float endpoints so drawLine3D never overflows int on huge projections).
static JET_HOT int lineF(FrameBuf& fb, float fx0, float fy0, float d0, float fx1, float fy1,
                         float d1, uint8_t color, int thick, int a16, bool wd) {
  if (a16 <= 0) return 0;
  float t0 = 0.0f, t1 = 1.0f;
  // Clip to the frame grown by 1 px (thick lines draw a +1 neighbour). Both ends inside: the
  // clip is the identity (t0 = 0, t1 = 1 exactly), so skip its four float divides.
  const bool inside = fx0 >= -1.0f && fx0 <= (float)LW && fx1 >= -1.0f && fx1 <= (float)LW &&
                      fy0 >= -1.0f && fy0 <= (float)LH && fy1 >= -1.0f && fy1 <= (float)LH;
  if (!inside && !clipSeg(fx0, fy0, fx1, fy1, -1.0f, -1.0f, (float)LW, (float)LH, t0, t1)) return 0;
  float cx0 = fx0 + (fx1 - fx0) * t0, cy0 = fy0 + (fy1 - fy0) * t0;
  float cx1 = fx0 + (fx1 - fx0) * t1, cy1 = fy0 + (fy1 - fy0) * t1;
  float cd0 = d0 + (d1 - d0) * t0, cd1 = d0 + (d1 - d0) * t1;
  int x0 = roundi(cx0), y0 = roundi(cy0), x1 = roundi(cx1), y1 = roundi(cy1);
  int dx = x1 - x0, dy = y1 - y0;
  int adx = dx < 0 ? -dx : dx, ady = dy < 0 ? -dy : dy;
  int sx = dx < 0 ? -1 : 1, sy = dy < 0 ? -1 : 1;
  int n = adx > ady ? adx : ady;
  // Depth in 16.16 fixed point, stepped linearly.
  int32_t dqf = (int32_t)(cd0 * 65536.0f);
  int32_t dqs = n ? (int32_t)((cd1 - cd0) * 65536.0f) / n : 0;
  bool xMajor = adx >= ady;
  int err = xMajor ? adx / 2 : ady / 2;
  int x = x0, y = y0;
  uint8_t* px = fb.px;
  uint8_t* depth = fb.depth;
  for (int i = 0; i <= n; ++i) {
    int dqi = dqf >> 16;
    uint8_t dq = (uint8_t)(dqi < 0 ? 0 : (dqi > 255 ? 255 : dqi));
    if ((unsigned)x < (unsigned)LW && (unsigned)y < (unsigned)LH && passAlpha(x, y, a16)) {
      int p = y * LW + x;
      if (dq <= depth[p]) { px[p] = color; if (wd) depth[p] = dq; }
    }
    if (thick >= 2) {
      int nx = xMajor ? x : x + 1, ny = xMajor ? y + 1 : y;
      if ((unsigned)nx < (unsigned)LW && (unsigned)ny < (unsigned)LH && passAlpha(nx, ny, a16)) {
        int p = ny * LW + nx;
        if (dq <= depth[p]) { px[p] = color; if (wd) depth[p] = dq; }
      }
    }
    dqf += dqs;
    if (xMajor) {
      x += sx; err -= ady;
      if (err < 0) { y += sy; err += adx; }
    } else {
      y += sy; err -= adx;
      if (err < 0) { x += sx; err += ady; }
    }
  }
  return (n + 1) * (thick >= 2 ? 2 : 1);
}

void drawPoint(FrameBuf& fb, int x, int y, uint8_t dq, uint8_t color, bool writeDepth) {
  plotRaw(fb, x, y, dq, color, 16, writeDepth);
}

JET_HOT void drawLine(FrameBuf& fb, int x0, int y0, uint8_t dq0, int x1, int y1, uint8_t dq1,
                      uint8_t color, int thick, int alpha16, bool writeDepth) {
  lineF(fb, (float)x0, (float)y0, (float)dq0, (float)x1, (float)y1, (float)dq1, color, thick,
        alpha16, writeDepth);
}

int drawLine3D(FrameBuf& fb, const View& v, const Vec3& a, const Vec3& b, uint8_t color, int thick,
               int alpha16, bool writeDepth) {
  ScreenPt pa, pb;
  if (!project(v, a, pa, cfg::CULL_DEN) || !project(v, b, pb, cfg::CULL_DEN)) return 0;
  return lineF(fb, pa.x, pa.y, (float)pa.dq, pb.x, pb.y, (float)pb.dq, color, thick, alpha16,
               writeDepth);
}

void drawDisc(FrameBuf& fb, int cx, int cy, float r, uint8_t dq, uint8_t color, int alpha16,
              bool writeDepth) {
  if (alpha16 <= 0) return;
  if (r <= 0.5f) { plotRaw(fb, cx, cy, dq, color, alpha16, writeDepth); return; }
  int R = (int)ceilf(r);
  float lim = r * r + 0.5f * r;   // r=1 -> plus, r=1.5 -> 3x3, r=2 -> round 5x5
  int y0 = cy - R < 0 ? 0 : cy - R, y1 = cy + R > LH - 1 ? LH - 1 : cy + R;
  int x0 = cx - R < 0 ? 0 : cx - R, x1 = cx + R > LW - 1 ? LW - 1 : cx + R;
  for (int y = y0; y <= y1; ++y) {
    int dy = y - cy;
    float dy2 = (float)(dy * dy);
    for (int x = x0; x <= x1; ++x) {
      int dx = x - cx;
      if ((float)(dx * dx) + dy2 > lim) continue;
      if (!passAlpha(x, y, alpha16)) continue;
      int p = y * LW + x;
      if (dq <= fb.depth[p]) {
        fb.px[p] = color;
        if (writeDepth) fb.depth[p] = dq;
      }
    }
  }
}

void drawGlow(FrameBuf& fb, int cx, int cy, float r0, float r1, uint8_t dq, uint8_t color,
              int alpha16Inner) {
  if (alpha16Inner <= 0 || r1 <= r0) return;
  int R = (int)r1;
  if ((float)R < r1) ++R;   // ceil (r1 > r0 >= 0 in every caller; r1 > 0 here)
  const float r0s = r0 * r0, r1s = r1 * r1, inv = 1.0f / (r1 - r0);
  int y0 = cy - R < 0 ? 0 : cy - R, y1 = cy + R > LH - 1 ? LH - 1 : cy + R;
  int x0 = cx - R < 0 ? 0 : cx - R, x1 = cx + R > LW - 1 ? LW - 1 : cx + R;
  // d2 is an integer, so the coverage only depends on d2: evaluate the sqrtf once per distinct
  // d2 (<= 64 entries for every glow in the game) instead of once per pixel.
  constexpr int TAB = 64;
  const bool useTab = r0 >= 0.0f && r1s < (float)TAB;
  int8_t aOf[TAB];
  if (useTab) {
    for (int d = 0; d < TAB; ++d) {
      const float d2 = (float)d;
      int a = 0;
      if (d2 > r0s && d2 <= r1s) {
        const float t = (sqrtf(d2) - r0) * inv;     // 0 at r0 .. 1 at r1
        a = (int)((float)alpha16Inner * (1.0f - t) + 0.5f);
      }
      aOf[d] = (int8_t)(a < 0 ? 0 : (a > 16 ? 16 : a));
      if (d2 > r1s) {
        for (int e = d + 1; e < TAB; ++e) aOf[e] = 0;
        break;
      }
    }
  }
  for (int y = y0; y <= y1; ++y) {
    int dy = y - cy;
    const uint8_t* brow = s_bayer + ((y & 3) << 2);
    for (int x = x0; x <= x1; ++x) {
      int dx = x - cx;
      const int d2i = dx * dx + dy * dy;
      int a;
      if (useTab) {
        if (d2i >= TAB) continue;
        a = aOf[d2i];
      } else {
        float d2 = (float)d2i;
        if (d2 <= r0s || d2 > r1s) continue;
        float t = (sqrtf(d2) - r0) * inv;             // 0 at r0 .. 1 at r1
        a = (int)((float)alpha16Inner * (1.0f - t) + 0.5f);
      }
      if (a <= 0 || brow[x & 3] >= a) continue;
      int p = y * LW + x;
      if (dq <= fb.depth[p]) fb.px[p] = color;
    }
  }
}

void drawRect(FrameBuf& fb, int x, int y, int w, int h, uint8_t color) {
  int xa = x < 0 ? 0 : x, ya = y < 0 ? 0 : y;
  int xb = x + w > LW ? LW : x + w, yb = y + h > LH ? LH : y + h;
  for (int yy = ya; yy < yb; ++yy) {
    uint8_t* row = fb.px + yy * LW;
    for (int xx = xa; xx < xb; ++xx) row[xx] = color;
  }
}

void drawRectDither(FrameBuf& fb, int x, int y, int w, int h, uint8_t color, int alpha16) {
  if (alpha16 <= 0) return;
  int xa = x < 0 ? 0 : x, ya = y < 0 ? 0 : y;
  int xb = x + w > LW ? LW : x + w, yb = y + h > LH ? LH : y + h;
  for (int yy = ya; yy < yb; ++yy) {
    uint8_t* row = fb.px + yy * LW;
    for (int xx = xa; xx < xb; ++xx)
      if (passAlpha(xx, yy, alpha16)) row[xx] = color;
  }
}

}  // namespace kj
