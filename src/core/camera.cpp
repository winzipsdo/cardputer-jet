// camera.cpp - level lens-shift chase camera (DESIGN.md section 5). OWNER: group D.
#include "camera.h"
#include "config.h"
#include "mathx.h"
#include "flight.h"
#include "world.h"
#include "fastmath_d.h"

namespace kj {

namespace {

// T correction (added to tan(tilt)) that keeps the eye above the terrain floor and out of the
// cloud slab. Returns the needed correction and the floor height.
float neededFix(const World& w, float eyeX, float eyeY, float alt, float T0, float dist,
                float jetZ, float& floorZ) {
  const float eyeZ = alt + T0 * dist;
  floorZ = fm::maxGroundAround(w, eyeX, eyeY, cfg::CAM_FLOOR_PROBE) + cfg::CAM_FLOOR_MARGIN;
  float need = 0.0f;
  const float lo = cfg::CLOUD_BASE - cfg::CAM_CLOUD_MARGIN;
  const float hi = cfg::CLOUD_TOP + cfg::CAM_CLOUD_MARGIN;
  if (eyeZ > lo && eyeZ < hi) {
    if (jetZ < cfg::CLOUD_BASE) need = (lo - eyeZ) / dist;
    else need = (hi - eyeZ) / dist;
  }
  if (eyeZ + need * dist < floorZ) need = (floorZ - eyeZ) / dist;
  return need;
}

}  // namespace

void ChaseCamera::reset(const JetState& s, const World& w, int viewPreset) {
  const int p = clampi(viewPreset, 0, 2);
  v_ = View{};
  v_.fx = wrapPos(s.pos.x);
  v_.fy = wrapPos(s.pos.y);
  v_.alt = s.pos.z;
  v_.psi = wrapPi(s.yaw);
  distEase_ = cfg::CAM_DIST[p] * (1.0f - cfg::CAM_BOOST_PULL * s.boost);
  f_ = cfg::CAM_F0 * (1.0f - cfg::CAM_FOV_PUNCH * s.boost);
  fVel_ = 0.0f;
  tilt_ = clampf(cfg::CAM_TILT[p] - cfg::CAM_TILT_CLIMB_K * sinf(s.pitch), -cfg::CAM_T_CLAMP,
                 cfg::CAM_T_CLAMP);
  shakeT_ = 0.0f;
  orbit_ = false;
  prevPos_ = s.pos;
  // Start with the terrain/cloud correction already applied (no eased pop-in).
  const float T0 = tanf(tilt_);
  const float ex = wrapPos(v_.fx - sinf(v_.psi) * distEase_);
  const float ey = wrapPos(v_.fy + cosf(v_.psi) * distEase_);
  float floorZ = 0.0f;
  tfix_ = neededFix(w, ex, ey, v_.alt, T0, distEase_, s.pos.z, floorZ);
  update(s, w, 0.0f, p, 0.0f);
}

void ChaseCamera::setCrashOrbit(bool on, const Vec3& wreck) {
  orbit_ = on;
  wreck_ = wreck;
}

JET_D_HOT void ChaseCamera::update(const JetState& s, const World& w, float dt, int viewPreset,
                         float shake) {
  const int p = clampi(viewPreset, 0, 2);
  if (dt < 0.0f) dt = 0.0f;
  const float boost = orbit_ ? 0.0f : s.boost;
  const Vec3 prevPos = prevPos_;
  prevPos_ = s.pos;

  // ---------------------------------------------------------------- follow
  if (!orbit_) {
    const float k = easeFactor(cfg::CAM_FOCUS_EASE, dt);
    // Predict with the jet's ACTUAL displacement this frame, not s.vel: in states where the
    // flight is not stepped (countdown, pause, help, recal) s.vel stays at cruise while the
    // jet is frozen, and a velocity feed would park the focus ~vel*(1-k)*dt/k ahead of it.
    Vec3 d = wrapDelta(s.pos, prevPos);
    if (fabsf(d.x) + fabsf(d.y) + fabsf(d.z) > 60.0f) d = v3(0.0f, 0.0f, 0.0f);  // teleport
    float px = v_.fx + d.x, py = v_.fy + d.y;
    px += wrapd(s.pos.x - px) * k;
    py += wrapd(s.pos.y - py) * k;
    v_.fx = fm::wrapPos(px);
    v_.fy = fm::wrapPos(py);
    v_.alt += d.z * cfg::CAM_ALT_FEED;
    easeTo(v_.alt, s.pos.z, cfg::CAM_ALT_EASE, dt);
    v_.psi = fm::wrapPi(v_.psi + fm::wrapPi(s.yaw - v_.psi) * easeFactor(cfg::CAM_HEADING_EASE, dt));
    easeTo(distEase_, cfg::CAM_DIST[p] * (1.0f - cfg::CAM_BOOST_PULL * s.boost),
           cfg::CAM_BOOST_EASE, dt);
  } else {
    v_.fx = fm::wrapPos(wreck_.x);
    v_.fy = fm::wrapPos(wreck_.y);
    easeTo(v_.alt, wreck_.z, cfg::CAM_ALT_EASE, dt);
    v_.psi = fm::wrapPi(v_.psi + cfg::CRASH_ORBIT_RATE * dt);
    easeTo(distEase_, cfg::CRASH_ORBIT_DIST, 2.0f, dt);
  }

  // ---------------------------------------------------------------- FOV spring
  {
    const float target = cfg::CAM_F0 * (1.0f - cfg::CAM_FOV_PUNCH * boost);
    const float wn = cfg::CAM_FOV_SPRING_W, z = cfg::CAM_FOV_SPRING_Z;
    int n = (int)(dt * 240.0f) + 1;
    if (n > 16) n = 16;
    const float h = dt / (float)n;
    for (int i = 0; i < n; ++i) {
      float acc = wn * wn * (target - f_) - 2.0f * z * wn * fVel_;
      fVel_ += acc * h;
      f_ += fVel_ * h;
    }
    f_ = clampf(f_, cfg::CAM_F0 * 0.6f, cfg::CAM_F0 * 1.3f);
  }

  // ---------------------------------------------------------------- look-down slope
  const float pitch = orbit_ ? 0.0f : s.pitch;
  const float tiltTarget = clampf(cfg::CAM_TILT[p] - cfg::CAM_TILT_CLIMB_K * sinf(pitch),
                                  -cfg::CAM_T_CLAMP, cfg::CAM_T_CLAMP);
  easeTo(tilt_, tiltTarget, cfg::CAM_T_EASE, dt);
  const float T0 = tanf(tilt_);
  const float dist = distEase_ > 1.0f ? distEase_ : 1.0f;
  const float sp = sinf(v_.psi), cpv = cosf(v_.psi);

  // ---------------------------------------------------------------- terrain / cloud avoidance
  {
    const float ex = fm::wrapPos(v_.fx - sp * dist), ey = fm::wrapPos(v_.fy + cpv * dist);
    const float jetZ = orbit_ ? wreck_.z : s.pos.z;
    float floorZ = 0.0f;
    const float need = neededFix(w, ex, ey, v_.alt, T0, dist, jetZ, floorZ);
    easeTo(tfix_, need, cfg::CAM_TFIX_EASE, dt);
    // never let the eye sink into terrain while the correction is still easing in
    if (v_.alt + (T0 + tfix_) * dist < floorZ - 1.0f) tfix_ = (floorZ - 1.0f - v_.alt) / dist - T0;
  }
  float T = T0 + tfix_;
  T = clampf(T, -2.0f, 2.0f);

  // ---------------------------------------------------------------- shake (stochastic rounding
  // so the small continuous rumble still produces occasional 1 px jitter)
  shakeT_ += dt;
  if (shakeT_ > 1024.0f) shakeT_ -= 1024.0f;  // float precision on endless attract loops
  float skim = 0.0f;
  if (!orbit_ && s.overWater) skim = sat(1.0f - s.heightAboveSurface / cfg::SKIM_BAND);
  const float sh = clampf(shake + 0.06f * boost + 0.08f * skim, 0.0f, 1.0f);
  float dx = 0.0f, dy = 0.0f;
  if (sh > 0.0f) {
    float d1 = shakeT_ * 61.803f, d2 = shakeT_ * 47.213f;
    d1 -= fm::floorF(d1);
    d2 -= fm::floorF(d2);
    dx = fm::floorF(cfg::SHAKE_X_PX * sh * sinf(91.0f * shakeT_) + d1);
    dy = fm::floorF(cfg::SHAKE_Y_PX * sh * cosf(77.0f * shakeT_) + d2);
  }

  // ---------------------------------------------------------------- derived view
  v_.dist = dist;
  v_.q = 1.0f / dist;
  v_.f = f_;
  v_.S = f_ / dist;
  v_.T = T;
  v_.cpsi = cpv;
  v_.spsi = sp;
  v_.Cx = (float)cfg::LW * 0.5f + dx;
  v_.Cy = (float)cfg::LH * (cfg::CAM_CY_FRAC - cfg::CAM_CY_BOOST * boost) + dy;
  v_.horizon = v_.Cy - v_.f * v_.T;
  v_.eyeX = fm::wrapPos(v_.fx - sp * dist);
  v_.eyeY = fm::wrapPos(v_.fy + cpv * dist);
  v_.eyeZ = v_.alt + T * dist;
  const float a = v_.alt > 0.0f ? v_.alt : 0.0f;
  float vd = cfg::VIEW_DIST_BASE + cfg::VIEW_DIST_ALT * a;
  v_.viewDist = vd < cfg::VIEW_DIST_MAX ? vd : cfg::VIEW_DIST_MAX;
  v_.fogStart = cfg::FOG_START_BASE + cfg::FOG_START_ALT * a;
  v_.camBelowClouds = v_.eyeZ < cfg::CLOUD_BASE;
}

}  // namespace kj
