// fastmath_d.h - private helpers of group D (game, flight, camera, course, fx, hud, synth).
// OWNER: group D. Not a shared contract: only group-D .cpp files include it.
//
// Everything here is BIT-IDENTICAL to the mathx.h / world.h originals for every input; the
// fast paths only skip the slow library calls (fmodf, floorf, expf) on the ESP32-S3, where
// they are out-of-line soft routines that also thrash the 16 KB instruction cache.
#pragma once
#include <stdint.h>
#include <math.h>
#include "compat.h"
#include "config.h"
#include "mathx.h"
#include "world.h"
#include "frame.h"

// Per-frame game code of group D runs once per frame from flash and misses the 16 KB I-cache on
// almost every line. JET_D_HOT puts the hottest such functions in IRAM (internal RAM):
//   JET_D_IRAM=2 (default): Synth::render, TiltController::update, ChaseCamera::update
//                           (JET_D_HOT, ~8.4 KB) + Flight::step (JET_D_HOT2, ~5.9 KB)
//   JET_D_IRAM=1: without Flight::step (it runs twice per frame; the 2nd call hits the cache)
//   JET_D_IRAM=0: everything in flash (gives the ~14.3 KB of IRAM back to the heap)
#ifndef JET_D_IRAM
#define JET_D_IRAM 2
#endif
#if JET_D_IRAM >= 1
#define JET_D_HOT JET_HOT
#else
#define JET_D_HOT
#endif
#if JET_D_IRAM >= 2
#define JET_D_HOT2 JET_HOT
#else
#define JET_D_HOT2
#endif

namespace kj {
namespace fm {

// Rare slow paths kept out of line, so the inlined fast paths stay small (IRAM / I-cache).
JET_NOINLINE inline float wrapPosSlow(float p) { return kj::wrapPos(p); }
JET_NOINLINE inline float wrapPiSlow(float a) { return kj::wrapPi(a); }
JET_NOINLINE inline float floorSlow(float v) { return floorf(v); }
JET_NOINLINE inline float easeSlow(float k, float dt) { return easeFactor(k, dt); }

// floor(v) as int; exact for |v| < 2^31 (same result as (int)floorf(v)).
inline int floorI(float v) {
  int i = (int)v;
  return (float)i > v ? i - 1 : i;
}
// floorf(v) for |v| < 2^31 (identical result; larger values are already integers).
inline float floorF(float v) {
  if (!(v > -2.0e9f && v < 2.0e9f)) return floorSlow(v);
  return (float)floorI(v);
}

// == wrapPos(p) (mathx.h). In-range and one-period-off inputs avoid fmodf; the results are
// exact (Sterbenz) and equal to the fmodf path, including the p + W rounding for p < 0.
inline float wrapPos(float p) {
  const float W = cfg::WORLD_U;
  if (p >= 0.0f && p < W) return p;
  if (p >= W && p < 2.0f * W) return p - W;
  if (p < 0.0f && p > -W) return p + W;
  return wrapPosSlow(p);
}
inline Vec3 wrapPos3(const Vec3& p) { return v3(wrapPos(p.x), wrapPos(p.y), p.z); }

// == wrapPi(a) (mathx.h) without fmodf for a in [-pi, pi) (+- one turn falls back).
inline float wrapPi(float a) {
  const float t = a + PI;
  if (t >= 0.0f && t < TAU) return t - PI;
  return wrapPiSlow(a);
}

// == World::groundHeight / isWater, but one cell lookup serves both.
inline uint8_t cellAt(const World& w, float x, float y) {
  return w.cell(floorI(x * (1.0f / cfg::CELL_U)), floorI(y * (1.0f / cfg::CELL_U)));
}
inline float cellHeight(uint8_t c) { return (float)cfg::HEIGHT_U[c >> 4]; }
// Land height, or -1 over water (flight.cpp / course.cpp landHeight()).
inline float landHeight(const World& w, float x, float y) {
  const uint8_t c = cellAt(w, x, y);
  return cellIsWater(c) ? -1.0f : cellHeight(c);
}
inline float groundHeight(const World& w, float x, float y) { return cellHeight(cellAt(w, x, y)); }
// Land height at the wrapped (x, y), -1 over water. Out of line: probe loops call it often, so
// one small cached copy beats inlining the cell lookup at every call site.
JET_NOINLINE inline float landHeightWrapped(const World& w, float x, float y) {
  return landHeight(w, wrapPos(x), wrapPos(y));
}
// == World::maxGroundAround (3x3 probe at +-r).
JET_NOINLINE inline float maxGroundAround(const World& w, float x, float y, float r) {
  float m = 0.0f;
  for (int j = -1; j <= 1; ++j) {
    for (int i = -1; i <= 1; ++i) {
      const float h = groundHeight(w, x + (float)i * r, y + (float)j * r);
      if (h > m) m = h;
    }
  }
  return m;
}
// == World::insideCloud.
inline bool insideCloud(const World& w, float x, float y, float z) {
  const int k = w.cloudCell(floorI((x - w.cloudOffX) * (1.0f / cfg::CLOUD_CELL_U)),
                            floorI((y - w.cloudOffY) * (1.0f / cfg::CLOUD_CELL_U)));
  if (k <= 0) return false;
  return z >= cfg::CLOUD_BASE && z <= cfg::CLOUD_BASE + cfg::CLOUD_LAYER_U * (float)k;
}

// easeFactor(k, dt) memoised on dt: the flight sub-step always uses the same dt, so the expf
// runs once instead of every step. Same expression -> identical value.
struct EaseMemo {
  float k, dt, f;
  constexpr explicit EaseMemo(float kk) : k(kk), dt(-1.0f), f(0.0f) {}
  float get(float d) {
    if (d != dt) {
      dt = d;
      f = easeSlow(k, d);
    }
    return f;
  }
};

// True when the integer end points lie in drawLine's clip box [-1, LW] x [-1, LH]: then the
// line routine's Liang-Barsky clip is a no-op (t0 = 0, t1 = 1 exactly) and lineNoClip below
// plots exactly the same pixels.
inline bool lineInBox(int x0, int y0, int x1, int y1) {
  return x0 >= -1 && x0 <= LW && x1 >= -1 && x1 <= LW && y0 >= -1 && y0 <= LH && y1 >= -1 &&
         y1 <= LH;
}

// == drawLine(fb, x0, y0, dq0, x1, y1, dq1, color, thick, a16, writeDepth) for end points that
// pass lineInBox(): the same Bresenham walk, 16.16 depth ramp and plot rules as draw.cpp's
// lineF, minus the clip (four float divides per line in lineF). Returns the pixels attempted
// (drawLine3D's count). Out-of-box lines must keep using drawLine / drawLine3D.
JET_NOINLINE inline int lineNoClip(FrameBuf& fb, int x0, int y0, int dq0, int x1, int y1, int dq1,
                                   uint8_t color, int thick, int a16, bool wd) {
  if (a16 <= 0) return 0;
  const int dx = x1 - x0, dy = y1 - y0;
  const int adx = dx < 0 ? -dx : dx, ady = dy < 0 ? -dy : dy;
  const int sx = dx < 0 ? -1 : 1, sy = dy < 0 ? -1 : 1;
  const int n = adx > ady ? adx : ady;
  int32_t dqf = (int32_t)dq0 * 65536;
  const int32_t dqs = n ? ((int32_t)(dq1 - dq0) * 65536) / n : 0;
  const bool xMajor = adx >= ady;
  int err = xMajor ? adx / 2 : ady / 2;
  int x = x0, y = y0;
  uint8_t* px = fb.px;
  uint8_t* depth = fb.depth;
  for (int i = 0; i <= n; ++i) {
    const int dqi = dqf >> 16;
    const uint8_t dq = (uint8_t)(dqi < 0 ? 0 : (dqi > 255 ? 255 : dqi));
    if ((unsigned)x < (unsigned)LW && (unsigned)y < (unsigned)LH && (a16 >= 16 || bayer(x, y) < a16)) {
      const int p = y * LW + x;
      if (dq <= depth[p]) { px[p] = color; if (wd) depth[p] = dq; }
    }
    if (thick >= 2) {
      const int nx = xMajor ? x : x + 1, ny = xMajor ? y + 1 : y;
      if ((unsigned)nx < (unsigned)LW && (unsigned)ny < (unsigned)LH &&
          (a16 >= 16 || bayer(nx, ny) < a16)) {
        const int p = ny * LW + nx;
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

// drawGlow (draw.h) for one fixed (r0, r1, alpha16Inner): the per-offset coverage (one sqrtf
// and one divide per pixel in drawGlow) is computed once by build(); draw() then plots exactly
// the same pixels. No depth write (like drawGlow).
struct GlowStamp {
  static constexpr int MAXR = 4;
  static constexpr int N = 2 * MAXR + 1;
  int R = -1;
  uint8_t a[N * N] = {};
  void build(float r0, float r1, int alpha16Inner) {
    R = 0;
    for (int i = 0; i < N * N; ++i) a[i] = 0;
    if (alpha16Inner <= 0 || r1 <= r0) return;
    R = (int)ceilf(r1);
    if (R > MAXR) R = MAXR;
    const float r0s = r0 * r0, r1s = r1 * r1, inv = 1.0f / (r1 - r0);
    for (int dy = -R; dy <= R; ++dy) {
      for (int dx = -R; dx <= R; ++dx) {
        const float d2 = (float)(dx * dx + dy * dy);
        if (d2 <= r0s || d2 > r1s) continue;
        const float t = (sqrtf(d2) - r0) * inv;
        const int v = (int)((float)alpha16Inner * (1.0f - t) + 0.5f);
        a[(dy + MAXR) * N + dx + MAXR] = (uint8_t)(v > 0 ? v : 0);
      }
    }
  }
  bool ready() const { return R >= 0; }
  void draw(FrameBuf& fb, int cx, int cy, uint8_t dq, uint8_t color) const {
    const int y0 = cy - R < 0 ? 0 : cy - R, y1 = cy + R > LH - 1 ? LH - 1 : cy + R;
    const int x0 = cx - R < 0 ? 0 : cx - R, x1 = cx + R > LW - 1 ? LW - 1 : cx + R;
    for (int y = y0; y <= y1; ++y) {
      const uint8_t* ar = a + (y - cy + MAXR) * N + MAXR - cx;
      const int row = y * LW;
      for (int x = x0; x <= x1; ++x) {
        const int av = ar[x];
        if (av <= 0 || bayer(x, y) >= av) continue;
        const int p = row + x;
        if (dq <= fb.depth[p]) fb.px[p] = color;
      }
    }
  }
};

}  // namespace fm
}  // namespace kj
