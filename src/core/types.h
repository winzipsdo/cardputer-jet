// types.h - common value types and enums shared by every group.
// OWNER: group D (game) with sign-off from all groups. SHARED CONTRACT.
//
// World conventions (all groups):
//   * 1 u (world unit) = 1 jet voxel. Jet is 34 u long, 26 u span.
//   * z is up. Sea surface is z = 0.
//   * Heading psi: forward(psi) = (sin psi, -cos psi, 0); right(psi) = (cos psi, sin psi, 0).
//     psi = 0 points to -y ("north"); positive psi turns right (clockwise seen from above).
//   * NOTE: with x east, y south, z up this frame is LEFT-handed (like a screen map with z
//     toward the viewer). Never derive right/up vectors with cross() in world space; use
//     rightOf()/forwardOf() and the jet-axis formulas in render_jet.h. (Device-frame IMU
//     math in tilt.cpp is a separate, right-handed frame - cross() is fine there.)
//   * The world wraps every cfg::WORLD_U (8192 u) in x and y. Stored positions are kept in
//     [0, WORLD_U) with wrapPos(); every position DIFFERENCE must go through wrapd().
#pragma once
#include <stdint.h>
#include <math.h>

namespace kj {

struct Vec3 {
  float x, y, z;
};
inline Vec3 v3(float x, float y, float z) { Vec3 r = {x, y, z}; return r; }
inline Vec3 operator+(const Vec3& a, const Vec3& b) { return v3(a.x + b.x, a.y + b.y, a.z + b.z); }
inline Vec3 operator-(const Vec3& a, const Vec3& b) { return v3(a.x - b.x, a.y - b.y, a.z - b.z); }
inline Vec3 operator*(const Vec3& a, float s) { return v3(a.x * s, a.y * s, a.z * s); }
inline Vec3 operator*(float s, const Vec3& a) { return v3(a.x * s, a.y * s, a.z * s); }
inline Vec3& operator+=(Vec3& a, const Vec3& b) { a.x += b.x; a.y += b.y; a.z += b.z; return a; }
inline float dot(const Vec3& a, const Vec3& b) { return a.x * b.x + a.y * b.y + a.z * b.z; }
inline Vec3 cross(const Vec3& a, const Vec3& b) {
  return v3(a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x);
}
inline float length(const Vec3& a) { return sqrtf(dot(a, a)); }
inline Vec3 normalize(const Vec3& a) {
  float l = length(a);
  return l > 1e-9f ? a * (1.0f / l) : v3(0.0f, 0.0f, 1.0f);
}

struct Rect {
  int16_t x0, y0, x1, y1;  // inclusive-exclusive: [x0,x1) x [y0,y1); empty if x1<=x0
};

enum class Tod : uint8_t { Night = 0, Sunset = 1, Day = 2, Count = 3 };
enum class Assist : uint8_t { Easy = 0, Normal = 1, Pro = 2 };
enum class Sens : uint8_t { Low = 0, Med = 1, High = 2 };

}  // namespace kj
