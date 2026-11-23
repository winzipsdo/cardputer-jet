// mathx.h - small inline math helpers, hashing, Bayer dither, deterministic RNG.
// OWNER: group D. SHARED CONTRACT (header-only; no .cpp).
// Rule: float only (sinf/cosf/expf/sqrtf/atan2f, literals with f suffix). No double.
#pragma once
#include <stdint.h>
#include <math.h>
#include "config.h"
#include "types.h"

namespace kj {

constexpr float PI = 3.14159265f;
constexpr float TAU = 6.28318531f;
constexpr float DEG = 0.0174532925f;   // degrees -> rad
constexpr float RAD2DEG = 57.2957795f;

inline float clampf(float v, float lo, float hi) { return v < lo ? lo : (v > hi ? hi : v); }
inline int clampi(int v, int lo, int hi) { return v < lo ? lo : (v > hi ? hi : v); }
inline float lerpf(float a, float b, float t) { return a + (b - a) * t; }
inline float signf(float v) { return v < 0.0f ? -1.0f : 1.0f; }
inline float sat(float v) { return clampf(v, 0.0f, 1.0f); }

// First-order easing: x += (target - x) * (1 - exp(-k dt)).
inline float easeFactor(float k, float dt) { return 1.0f - expf(-k * dt); }
inline void easeTo(float& x, float target, float k, float dt) { x += (target - x) * easeFactor(k, dt); }
inline float easeInOutQuad(float t) {
  t = sat(t);
  return t < 0.5f ? 2.0f * t * t : 1.0f - 2.0f * (1.0f - t) * (1.0f - t);
}
inline float smooth01(float t) { t = sat(t); return t * t * (3.0f - 2.0f * t); }

// Angle wrap to [-pi, pi).
inline float wrapPi(float a) {
  a = fmodf(a + PI, TAU);
  if (a < 0.0f) a += TAU;
  return a - PI;
}
// World position wrap to [0, WORLD_U) and world delta wrap to [-WORLD_U/2, WORLD_U/2).
inline float wrapPos(float p) {
  p = fmodf(p, cfg::WORLD_U);
  return p < 0.0f ? p + cfg::WORLD_U : p;
}
inline float wrapd(float d) {
  const float W = cfg::WORLD_U, H = cfg::WORLD_U * 0.5f;
  if (d >= H || d < -H) d -= W * floorf((d + H) / W);
  return d;
}
inline Vec3 wrapPos3(const Vec3& p) { return v3(wrapPos(p.x), wrapPos(p.y), p.z); }
inline Vec3 wrapDelta(const Vec3& a, const Vec3& b) {  // a - b, wrapped in x/y
  return v3(wrapd(a.x - b.x), wrapd(a.y - b.y), a.z - b.z);
}

inline Vec3 forwardOf(float psi) { return v3(sinf(psi), -cosf(psi), 0.0f); }
inline Vec3 rightOf(float psi) { return v3(cosf(psi), sinf(psi), 0.0f); }

// Integer hashes (identical on PC and device; used by world gen -> world CRC parity).
inline uint32_t hash32(uint32_t x) {
  x ^= x >> 16; x *= 0x7feb352dU; x ^= x >> 15; x *= 0x846ca68bU; x ^= x >> 16;
  return x;
}
inline uint32_t hash2i(int32_t x, int32_t y) {
  return hash32((uint32_t)x * 0x9E3779B1U ^ hash32((uint32_t)y + 0x632BE5ABU));
}
inline uint32_t hash3i(int32_t x, int32_t y, int32_t z) {
  return hash32(hash2i(x, y) ^ ((uint32_t)z * 0x85EBCA77U));
}

// 4x4 Bayer matrix, values 0..15. alpha16 in 0..16: pixel passes when BAYER4 < alpha16.
constexpr uint8_t BAYER4[16] = {0, 8, 2, 10, 12, 4, 14, 6, 3, 11, 1, 9, 15, 7, 13, 5};
inline int bayer(int x, int y) { return BAYER4[((y & 3) << 2) | (x & 3)]; }
inline bool bayerPass(int x, int y, int alpha16) { return bayer(x, y) < alpha16; }
inline int alpha16(float a) { return clampi((int)(a * 16.0f + 0.5f), 0, 16); }

// Deterministic xorshift32 RNG (never seed with 0).
struct Rng {
  uint32_t s = 0x1234567u;
  explicit Rng(uint32_t seed = 0x1234567u) : s(seed ? seed : 0x1234567u) {}
  uint32_t next() { s ^= s << 13; s ^= s >> 17; s ^= s << 5; return s; }
  float uniform() { return (float)(next() >> 8) * (1.0f / 16777216.0f); }  // [0,1)
  float range(float a, float b) { return a + (b - a) * uniform(); }
  int irange(int lo, int hiInclusive) { return lo + (int)(next() % (uint32_t)(hiInclusive - lo + 1)); }
  bool chance(float p) { return uniform() < p; }
};

}  // namespace kj
