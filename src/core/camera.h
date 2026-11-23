// camera.h - level pinhole chase camera with lens shift (no camera pitch or roll).
// OWNER: group D (camera.cpp). SHARED CONTRACT (View + project() are used by B, C, D).
//
// Projection of a world point P (all groups MUST use project() or the identical formula):
//   d = wrapDelta(P, focus);  across = d.x*cpsi + d.y*spsi ... see project() below
//   across = dx*cos(psi) + dy*sin(psi)            (= dot(d, right(psi)))
//   depth  = dx*sin(psi) - dy*cos(psi)            (= dot(d, forward(psi)), + = away from camera)
//   den    = 1 + q*depth   (cull if den < minDen)   m = S/den   (px per u at that depth)
//   sx     = Cx + m*across
//   sy     = Cy - m*((z - alt) + T*depth)
//   zc     = dist*den  (camera-space depth, for depthQ)
// The eye sits dist behind and T*dist above the focus; horizon row = Cy - f*T, f = S*dist.
#pragma once
#include <stdint.h>
#include "types.h"
#include "frame.h"

namespace kj {

class World;
struct JetState;

struct View {
  float fx, fy, alt;     // focus point (x,y wrapped)
  float psi, cpsi, spsi; // view heading and its cos/sin
  float dist;            // eye distance behind the focus (u)
  float q;               // 1/dist
  float S;               // px per u at the focus
  float f;               // focal length px = S*dist
  float T;               // look-down slope (includes terrain/cloud fix)
  float Cx, Cy;          // focus screen position (logical px, includes shake)
  float horizon;         // Cy - f*T (row of the horizon, may be off-screen)
  float eyeX, eyeY, eyeZ;
  float viewDist;        // march far limit (zc)
  float fogStart;        // zc where fog begins
  bool camBelowClouds;   // eyeZ < CLOUD_BASE
};

struct ScreenPt {
  float x, y;   // logical px
  float m;      // px per u at that depth
  float zc;     // camera depth (u)
  uint8_t dq;   // depthQ(zc)
};

inline bool project(const View& v, const Vec3& p, ScreenPt& out, float minDen = 0.15f) {
  float dx = p.x - v.fx, dy = p.y - v.fy;
  const float W = 8192.0f, H = 4096.0f;   // == cfg::WORLD_U (wrapd inlined for speed)
  if (dx >= H) dx -= W; else if (dx < -H) dx += W;
  if (dy >= H) dy -= W; else if (dy < -H) dy += W;
  float across = dx * v.cpsi + dy * v.spsi;
  float depth = dx * v.spsi - dy * v.cpsi;
  float den = 1.0f + v.q * depth;
  if (den < minDen) return false;
  float m = v.S / den;
  out.x = v.Cx + m * across;
  out.y = v.Cy - m * ((p.z - v.alt) + v.T * depth);
  out.m = m;
  out.zc = v.dist * den;
  out.dq = depthQ(out.zc);
  return true;
}

class ChaseCamera {
 public:
  void reset(const JetState& s, const World& w, int viewPreset);
  // dt is real (unscaled) time. shake in 0..1 (game owns decay). boost from JetState.
  void update(const JetState& s, const World& w, float dt, int viewPreset, float shake);
  void setCrashOrbit(bool on, const Vec3& wreck);
  const View& view() const { return v_; }

 private:
  View v_ = {};
  float tilt_ = 0.21f;      // eased base angle (rad)
  float tfix_ = 0.0f;       // eased terrain/cloud correction added to T
  float distEase_ = 88.0f;  // eased distance incl. boost pull
  float f_ = 88.0f, fVel_ = 0.0f;  // FOV spring
  float shakeT_ = 0.0f;
  bool orbit_ = false;
  Vec3 wreck_ = {0.0f, 0.0f, 0.0f};
  Vec3 prevPos_ = {0.0f, 0.0f, 0.0f};  // jet pivot at the previous update (follow prediction)
};

}  // namespace kj
