// fx.cpp - particles, wingtip trails, wake V, speed lines, in-cloud mist.
// OWNER: group C. Everything is Bayer-dithered and depth-tested against the frame (which
// already holds the world and the jet); nothing here writes depth. No allocation.
#include "fx.h"

#include <math.h>
#include "draw.h"
#include "flight.h"
#include "mathx.h"
#include "palette.h"
#include "render_jet.h"
#include "world.h"
#include "fastmath_d.h"

namespace kj {

namespace {

enum PartKind : uint8_t { K_SMOKE = 0, K_SPRAY, K_FOAM, K_VAPOUR, K_FIRE, K_DEBRIS, K_SPARK };

constexpr float DEAD_T = -1.0e9f;       // wake mark / trail sentinel time
constexpr float SPEED_UNSET = -1.0e6f;  // speed line needs a (re)spawn

inline uint32_t xs32(uint32_t& s) {
  s ^= s << 13; s ^= s >> 17; s ^= s << 5;
  return s;
}
inline float frand(uint32_t& s) { return (float)(xs32(s) >> 8) * (1.0f / 16777216.0f); }
inline float frange(uint32_t& s, float a, float b) { return a + (b - a) * frand(s); }
inline float fsign(uint32_t& s) { return (xs32(s) & 0x100u) ? 1.0f : -1.0f; }
inline int roundi(float v) { return fm::floorI(v + 0.5f); }

inline bool alive(const Particle& p) { return p.life > 0.0f && p.age < p.life; }

Particle& allocPart(Particle* parts, int& next) {
  for (int i = 0; i < cfg::PARTICLES; ++i) {
    int k = (next + i) % cfg::PARTICLES;
    if (!alive(parts[k])) {
      next = (k + 1) % cfg::PARTICLES;
      return parts[k];
    }
  }
  Particle& p = parts[next];   // pool full: recycle round-robin (roughly the oldest)
  next = (next + 1) % cfg::PARTICLES;
  return p;
}

void initPart(Particle& p, const Vec3& pos, const Vec3& vel, float life, float size, uint8_t kind,
              uint8_t color, uint8_t alpha8) {
  p.p = fm::wrapPos3(pos);
  p.v = vel;
  p.age = 0.0f;
  p.life = life;
  p.size = size;
  p.kind = kind;
  p.color = color;
  p.pad[0] = alpha8;
  p.pad[1] = 0;
}

// Two unit vectors perpendicular to d (any handedness; only used to scatter particles).
void basisOf(const Vec3& d, Vec3& e1, Vec3& e2) {
  Vec3 n = normalize(d);
  Vec3 ref = (n.z > 0.9f || n.z < -0.9f) ? v3(1.0f, 0.0f, 0.0f) : v3(0.0f, 0.0f, 1.0f);
  e1 = normalize(cross(n, ref));
  e2 = normalize(cross(n, e1));
}

inline float dragK(float k, float dt) { return 1.0f / (1.0f + k * dt); }

// Filled, Bayer-dithered ellipse with constant depth (tolerance tol), no depth write.
// erode 0..1 thins the coverage toward the rim. skipFlame: leave flame / jet pixels alone.
int fillEllipse(FrameBuf& fb, float cx, float cy, float rx, float ry, uint8_t dq, int tol,
                uint8_t color, int a16, float erode, int bx, int by, bool skipJet) {
  if (a16 <= 0) return 0;
  if (rx < 0.5f) rx = 0.5f;
  if (ry < 0.5f) ry = 0.5f;
  // floor / ceil without libm calls (ceil(v) == -floor(-v)); |values| stay far below 2^31
  if (!(cx > -1.0e6f && cx < 1.0e6f && cy > -1.0e6f && cy < 1.0e6f && rx < 1.0e6f && ry < 1.0e6f))
    return 0;
  int x0 = fm::floorI(cx - rx), x1 = -fm::floorI(-(cx + rx));
  int y0 = fm::floorI(cy - ry), y1 = -fm::floorI(-(cy + ry));
  if (x1 < 0 || y1 < 0 || x0 >= LW || y0 >= LH) return 0;
  if (x0 < 0) x0 = 0;
  if (y0 < 0) y0 = 0;
  if (x1 > LW - 1) x1 = LW - 1;
  if (y1 > LH - 1) y1 = LH - 1;
  float irx = 1.0f / rx, iry = 1.0f / ry;
  int lim = (int)dq - tol;
  int n = 0;
  for (int y = y0; y <= y1; ++y) {
    float ny = ((float)y - cy) * iry;
    float ny2 = ny * ny;
    if (ny2 > 1.0f) continue;
    for (int x = x0; x <= x1; ++x) {
      float nx = ((float)x - cx) * irx;
      float rho = nx * nx + ny2;
      if (rho > 1.0f) continue;
      int a = erode > 0.0f ? (int)((float)a16 * (1.0f - erode * rho) + 0.5f) : a16;
      if (bayer(x + bx, y + by) >= a) continue;
      int q = y * LW + x;
      if (lim > (int)fb.depth[q]) continue;
      if (skipJet) {
        uint8_t c = fb.px[q];
        if (c >= pal::JET_FIRST && c < pal::FLAME0 + 6) continue;
      }
      fb.px[q] = color;
      ++n;
    }
  }
  return n;
}

// u^0.7 for the wake spread, u in [0, 1]: 256-segment table (built once with powf) + linear
// interpolation; |error| < 0.006 near 0 and < 1e-5 elsewhere (powf was ~1 call per wake mark).
float g_pow07[257];
bool g_pow07Init = false;
inline float pow07(float u) {
  if (!g_pow07Init) {
    for (int i = 0; i <= 256; ++i) g_pow07[i] = powf((float)i * (1.0f / 256.0f), 0.7f);
    g_pow07Init = true;
  }
  if (!(u > 0.0f)) return 0.0f;
  if (u >= 1.0f) return powf(u, 0.7f);
  const float f = u * 256.0f;
  const int i = (int)f;
  return g_pow07[i] + (g_pow07[i + 1] - g_pow07[i]) * (f - (float)i);
}

// 1 / sqrt(x) for x > 0 (bit seed + 3 Newton steps, relative error ~1e-7).
inline float rsqrtF(float x) {
  union { float f; uint32_t u; } b;
  b.f = x;
  b.u = 0x5F3759DFu - (b.u >> 1);
  float y = b.f;
  const float h = 0.5f * x;
  y = y * (1.5f - h * y * y);
  y = y * (1.5f - h * y * y);
  y = y * (1.5f - h * y * y);
  return y;
}

inline bool inLineBox(const ScreenPt& p) {
  return p.x >= -1.0f && p.x <= (float)LW && p.y >= -1.0f && p.y <= (float)LH;
}

// == drawLine3D(fb, v, a, b, ...) given the projections of a and b (see the trail comment).
int trailSegment(FrameBuf& fb, const View& v, const Vec3& a, const ScreenPt& sa, const Vec3& b,
                 const ScreenPt& sb, uint8_t col, int thick, int a16) {
  if (!inLineBox(sa) || !inLineBox(sb)) return drawLine3D(fb, v, a, b, col, thick, a16, false);
  if (a16 <= 0) return 0;
  // lineF's unclipped end is a + (b - a) * 1, which can differ from b in the last bit
  const int x0 = roundi(sa.x), y0 = roundi(sa.y);
  const int x1 = roundi(sa.x + (sb.x - sa.x)), y1 = roundi(sa.y + (sb.y - sa.y));
  return fm::lineNoClip(fb, x0, y0, sa.dq, x1, y1, sb.dq, col, thick, a16, false);
}

// == drawLine3D(fb, v, a, b, ...) (projects each end once; see trailSegment).
int line3D(FrameBuf& fb, const View& v, const Vec3& a, const Vec3& b, uint8_t col, int thick,
           int a16) {
  ScreenPt sa, sb;
  if (!project(v, a, sa, cfg::CULL_DEN) || !project(v, b, sb, cfg::CULL_DEN)) return 0;
  return trailSegment(fb, v, a, sa, b, sb, col, thick, a16);
}

}  // namespace

void Fx::reset() {
  for (int i = 0; i < cfg::PARTICLES; ++i) {
    parts_[i] = Particle();
    parts_[i].life = 0.0f;
  }
  nextPart_ = 0;
  for (int k = 0; k < 2; ++k) {
    for (int i = 0; i < cfg::TRAIL_POINTS; ++i) {
      trail_[k][i].p = v3(0.0f, 0.0f, 0.0f);
      trail_[k][i].t = DEAD_T;
    }
    trailHead_[k] = 0;
  }
  trailDist_ = 0.0f;
  for (int i = 0; i < cfg::WAKE_MARKS; ++i) {
    wake_marks_[i] = WakeMark();
    wake_marks_[i].t = DEAD_T;
  }
  wakeHead_ = 0;
  wakeDist_ = 0.0f;
  for (int i = 0; i < cfg::SPEEDLINES; ++i) speed_[i] = v3(0.0f, 0.0f, SPEED_UNSET);
  smokeT_ = sprayT_ = vapourT_ = 0.0f;
  wake_ = 0.0f;
  g_ = 0.0f;
  speedVis_ = 0.0f;
  vel_ = v3(0.0f, 0.0f, 0.0f);
  now_ = 0.0f;
  rng_ = 0x2545F491u;
}

void Fx::update(const JetState& s, const JetPose& pose, const World& w, float dt, bool inCloud) {
  if (dt < 0.0f) dt = 0.0f;
  now_ += dt;
  if (now_ >= 4096.0f) {  // float precision on endless attract loops: rebase stored times
    now_ -= 2048.0f;
    for (int k = 0; k < 2; ++k)
      for (int i = 0; i < cfg::TRAIL_POINTS; ++i)
        if (trail_[k][i].t > 0.0f) {
          trail_[k][i].t -= 2048.0f;
          if (!(trail_[k][i].t > 0.0f)) trail_[k][i].t = DEAD_T;  // older than 2048 s
        }
    for (int i = 0; i < cfg::WAKE_MARKS; ++i)
      if (wake_marks_[i].t > DEAD_T * 0.5f) {
        wake_marks_[i].t -= 2048.0f;
        if (!(wake_marks_[i].t > 0.0f)) wake_marks_[i].t = DEAD_T;
      }
  }
  vel_ = s.vel;
  g_ = s.g;
  const JetAxes ax = jetAxes(pose);
  const JetAnchors an = jetAnchors(pose, ax);
  const bool emit = pose.visible || s.invuln > 0.0f;   // hidden only by the blink: keep emitting
  const bool flameOn = emit && pose.assemble >= 1.0f;
  const float travel = s.speed * dt;
  const Vec3 fwdFlat = forwardOf(s.yaw), rightFlat = rightOf(s.yaw);

  // ---- particles: integrate and age (drag factors once per update, not per particle)
  const float dSmoke = dragK(3.0f, dt), dSpray = dragK(0.8f, dt), dFoam = dragK(2.0f, dt);
  const float dFire = dragK(2.5f, dt), dDebris = dragK(0.3f, dt), dSpark = dragK(1.5f, dt);
  for (int i = 0; i < cfg::PARTICLES; ++i) {
    Particle& p = parts_[i];
    if (!alive(p)) continue;
    p.age += dt;
    switch (p.kind) {
      case K_SMOKE: p.v = p.v * dSmoke; p.v.z += 4.0f * dt; break;
      case K_SPRAY: p.v = p.v * dSpray; p.v.z -= 50.0f * dt; break;
      case K_FOAM: p.v = p.v * dFoam; p.v.z = 0.0f; break;
      case K_VAPOUR: p.v = p.v * dSmoke; break;  // same k = 3
      case K_FIRE: p.v = p.v * dFire; p.v.z += 12.0f * dt; break;
      case K_DEBRIS: p.v = p.v * dDebris; p.v.z -= 60.0f * dt; break;
      case K_SPARK: p.v = p.v * dSpark; p.v.z -= 15.0f * dt; break;
      default: break;
    }
    p.p = fm::wrapPos3(p.p + p.v * dt);
    if (p.kind == K_SPRAY && p.p.z < 0.0f && p.v.z < 0.0f) p.age = p.life;   // back in the sea
    if (p.kind == K_DEBRIS) {
      float gh = fm::groundHeight(w, p.p.x, p.p.y);
      if (p.p.z < gh) { p.p.z = gh; p.v = v3(0.0f, 0.0f, 0.0f); }
    }
    if (p.kind == K_FIRE && p.p.z < 0.0f) p.p.z = 0.0f;
  }

  // ---- wingtip trails (only while pulling g; a break marker splits segments)
  const bool trailLive = trail_[0][trailHead_[0]].t > 0.0f;
  const bool spawnTrail = emit && (g_ > 0.3f || (trailLive && g_ > 0.22f));   // hysteresis
  if (spawnTrail) {
    trailDist_ += travel;
    bool commit = trailDist_ >= cfg::TRAIL_SPACING_U;
    for (int k = 0; k < 2; ++k) {
      const Vec3& tip = k ? an.tipR : an.tipL;
      TrailPoint& h = trail_[k][trailHead_[k]];
      if (h.t <= 0.0f || commit) {
        trailHead_[k] = (trailHead_[k] + 1) % cfg::TRAIL_POINTS;
      }
      TrailPoint& live = trail_[k][trailHead_[k]];   // the live head follows the tip
      live.p = tip;
      live.t = now_;
    }
    if (commit) trailDist_ = 0.0f;
  } else {
    for (int k = 0; k < 2; ++k) {
      TrailPoint& h = trail_[k][trailHead_[k]];
      if (h.t > 0.0f) {
        trailHead_[k] = (trailHead_[k] + 1) % cfg::TRAIL_POINTS;
        trail_[k][trailHead_[k]].t = DEAD_T;
      }
    }
    trailDist_ = 0.0f;
  }

  // ---- wake strength, foam marks, spray
  float wk = 0.0f;
  if (emit && s.overWater) {
    float h = s.heightAboveSurface;
    wk = sat((cfg::WAKE_START_H - h) / (cfg::WAKE_START_H - cfg::WAKE_FULL_H)) *
         sat(s.speed / cfg::WAKE_SPEED_FULL);
  }
  wake_ = wk;
  wakeDist_ += travel;
  if (wakeDist_ >= cfg::WAKE_SPACING_U) {
    wakeDist_ = 0.0f;
    if (wk > 0.03f) {
      WakeMark& m = wake_marks_[wakeHead_];
      wakeHead_ = (wakeHead_ + 1) % cfg::WAKE_MARKS;
      Vec3 c = s.pos - fwdFlat * cfg::WAKE_BEHIND_U;
      m.p = v3(fm::wrapPos(c.x), fm::wrapPos(c.y), 0.0f);
      m.ax = rightFlat.x;
      m.ay = rightFlat.y;
      m.t = now_;
      m.k = wk;
    }
  }
  if (wk > 0.25f) {
    float rate = (cfg::SPRAY_RATE_BASE + cfg::SPRAY_RATE_K * wk) * sat((wk - 0.25f) / 0.35f);
    sprayT_ += rate * dt;
    int guard = 0;
    while (sprayT_ >= 1.0f && guard++ < 8) {
      sprayT_ -= 1.0f;
      float side = fsign(rng_);
      Vec3 pos = v3(s.pos.x, s.pos.y, 0.3f) - fwdFlat * frange(rng_, 2.0f, 12.0f) +
                 rightFlat * (side * frange(rng_, 1.5f, 7.0f));
      Vec3 vel = rightFlat * (side * frange(rng_, 6.0f, 22.0f)) +
                 v3(0.0f, 0.0f, frange(rng_, 8.0f, 24.0f) * (0.5f + wk)) + s.vel * frange(rng_, 0.1f, 0.35f);
      initPart(allocPart(parts_, nextPart_), pos, vel, frange(rng_, 0.5f, 1.1f),
               frange(rng_, 0.8f, 1.6f), K_SPRAY, pal::SPRAY, 255);
    }
  } else {
    sprayT_ = 0.0f;
  }

  // ---- exhaust smoke from 78 % of the flame length
  if (flameOn) {
    float b = sat(s.boost);
    float period = lerpf(cfg::SMOKE_PERIOD, cfg::SMOKE_PERIOD_BOOST, b);
    smokeT_ += dt;
    int guard = 0;
    while (smokeT_ >= period && guard++ < 4) {
      smokeT_ -= period;
      Vec3 pos = an.nozzle + wrapDelta(an.flameTip, an.nozzle) * 0.78f +
                 ax.right * frange(rng_, -0.8f, 0.8f) + ax.up * frange(rng_, -0.8f, 0.8f);
      Vec3 vel = s.vel * -0.3f + v3(frange(rng_, -2.0f, 2.0f), frange(rng_, -2.0f, 2.0f), 1.0f);
      uint8_t a8 = (uint8_t)(255.0f * sat(0.46f + 0.36f * b));
      initPart(allocPart(parts_, nextPart_), pos, vel, cfg::SMOKE_LIFE * frange(rng_, 0.75f, 1.25f),
               frange(rng_, 1.3f, 2.5f), K_SMOKE, pal::SMOKE0, a8);
    }
    if (smokeT_ > period) smokeT_ = 0.0f;
  } else {
    smokeT_ = 0.0f;
  }

  // ---- vapour streaming off the wingtips inside clouds
  if (emit && inCloud) {
    vapourT_ += dt;
    int guard = 0;
    while (vapourT_ >= 0.04f && guard++ < 4) {
      vapourT_ -= 0.04f;
      for (int k = 0; k < 2; ++k) {
        const Vec3& tip = k ? an.tipR : an.tipL;
        Vec3 vel = s.vel * 0.55f + v3(frange(rng_, -3.0f, 3.0f), frange(rng_, -3.0f, 3.0f), frange(rng_, -2.0f, 2.0f));
        initPart(allocPart(parts_, nextPart_), tip, vel, frange(rng_, 0.35f, 0.5f),
                 frange(rng_, 1.2f, 2.0f), K_VAPOUR, pal::TRAIL0, 200);
      }
    }
  } else {
    vapourT_ = 0.0f;
  }

  // ---- speed lines: world-space streaks we fly past under boost
  easeTo(speedVis_, sat((s.boost - 0.15f) / 0.6f), 4.0f, dt);
  for (int i = 0; i < cfg::SPEEDLINES; ++i) {
    Vec3& sl = speed_[i];
    if (speedVis_ < 0.02f) { sl.z = SPEED_UNSET; continue; }
    bool unset = sl.z <= SPEED_UNSET * 0.5f;
    bool respawn = unset;
    if (!unset) {
      Vec3 d = wrapDelta(sl, s.pos);
      float along = dot(d, fwdFlat), lat = dot(d, rightFlat);
      respawn = along < -95.0f || along > cfg::SPEEDLINE_AHEAD + 80.0f || lat > 140.0f || lat < -140.0f;
    }
    if (respawn) {
      float ahead = unset ? frange(rng_, -60.0f, cfg::SPEEDLINE_AHEAD) : frange(rng_, 0.6f * cfg::SPEEDLINE_AHEAD, cfg::SPEEDLINE_AHEAD);
      float lat = fsign(rng_) * frange(rng_, 12.0f, 70.0f);
      float vert = frange(rng_, -14.0f, 36.0f);
      Vec3 np = s.pos + fwdFlat * ahead + rightFlat * lat + v3(0.0f, 0.0f, vert);
      if (np.z < 2.0f) np.z = 2.0f + frand(rng_) * 6.0f;
      sl = fm::wrapPos3(np);
    }
  }
}

void Fx::burst(Burst b, const Vec3& pos, const Vec3& dir, float strength) {
  Vec3 e1, e2;
  basisOf(dir, e1, e2);
  Vec3 n = normalize(dir);
  switch (b) {
    case Burst::RingPass:
    case Burst::Perfect: {
      float R = strength > 1.0f ? strength : 14.0f;
      for (int i = 0; i < 16; ++i) {
        float a = (float)i * (TAU / 16.0f) + frange(rng_, -0.1f, 0.1f);
        Vec3 radial = e1 * cosf(a) + e2 * sinf(a);
        Vec3 vel = radial * frange(rng_, 12.0f, 30.0f) + n * frange(rng_, -6.0f, 6.0f);
        initPart(allocPart(parts_, nextPart_), pos + radial * R, vel, frange(rng_, 0.5f, 0.85f),
                 frange(rng_, 0.8f, 1.4f), K_SPARK, (i & 1) ? pal::RING_GOLD : pal::RING_GLOW, 255);
      }
      if (b == Burst::Perfect) {
        for (int i = 0; i < 12; ++i) {
          float a = frange(rng_, 0.0f, TAU);
          Vec3 radial = e1 * cosf(a) + e2 * sinf(a);
          Vec3 vel = radial * frange(rng_, 25.0f, 45.0f) + n * frange(rng_, -10.0f, 10.0f);
          initPart(allocPart(parts_, nextPart_), pos + radial * (R * frange(rng_, 0.2f, 0.9f)), vel,
                   frange(rng_, 0.6f, 1.0f), frange(rng_, 1.0f, 1.8f), K_SPARK, pal::SPRAY, 255);
        }
      }
      break;
    }
    case Burst::CloudEntry: {
      for (int i = 0; i < 12; ++i) {
        float a = (float)i * (TAU / 12.0f);
        Vec3 radial = e1 * cosf(a) + e2 * sinf(a);
        Vec3 vel = radial * frange(rng_, 15.0f, 25.0f) + vel_ * 0.5f;
        initPart(allocPart(parts_, nextPart_), pos + radial * frange(rng_, 6.0f, 10.0f), vel,
                 frange(rng_, 0.6f, 0.9f), frange(rng_, 2.0f, 4.0f), K_VAPOUR, pal::TRAIL0, 220);
      }
      break;
    }
    case Burst::SkimTouch: {
      Vec3 fw = normalize(v3(vel_.x, vel_.y, 0.0f));
      Vec3 rt = v3(-fw.y, fw.x, 0.0f);
      for (int i = 0; i < 20; ++i) {
        float side = fsign(rng_);
        Vec3 p0 = v3(pos.x, pos.y, 0.3f) + rt * (side * frange(rng_, 0.0f, 6.0f)) - fw * frange(rng_, 0.0f, 8.0f);
        Vec3 vel = rt * (side * frange(rng_, 8.0f, 30.0f)) + v3(0.0f, 0.0f, frange(rng_, 15.0f, 40.0f)) +
                   vel_ * frange(rng_, 0.1f, 0.4f);
        initPart(allocPart(parts_, nextPart_), p0, vel, frange(rng_, 0.6f, 1.2f), frange(rng_, 1.0f, 2.0f),
                 K_SPRAY, pal::SPRAY, 255);
      }
      break;
    }
    case Burst::Crash: {
      for (int i = 0; i < 24; ++i) {
        Vec3 d = normalize(v3(frange(rng_, -1.0f, 1.0f), frange(rng_, -1.0f, 1.0f), frange(rng_, -0.2f, 1.0f)));
        initPart(allocPart(parts_, nextPart_), pos + d * frange(rng_, 0.0f, 5.0f),
                 d * frange(rng_, 8.0f, 30.0f) + v3(0.0f, 0.0f, 8.0f) + vel_ * 0.15f,
                 frange(rng_, 0.7f, 1.5f), frange(rng_, 4.0f, 8.0f), K_FIRE, pal::FLAME0, 255);
      }
      for (int i = 0; i < 14; ++i) {
        Vec3 d = normalize(v3(frange(rng_, -1.0f, 1.0f), frange(rng_, -1.0f, 1.0f), frange(rng_, 0.2f, 1.0f)));
        static const uint8_t DEB[4] = {(uint8_t)(pal::JET_BODY0 + 5), (uint8_t)(pal::JET_BODY0 + 3),
                                       (uint8_t)(pal::JET_ACCENT0 + 2), (uint8_t)(pal::JET_BODY0 + 1)};
        initPart(allocPart(parts_, nextPart_), pos, d * frange(rng_, 20.0f, 60.0f) + v3(0.0f, 0.0f, 25.0f) + vel_ * 0.25f,
                 frange(rng_, 1.5f, 2.2f), frange(rng_, 0.8f, 1.6f), K_DEBRIS, DEB[i & 3], 255);
      }
      for (int i = 0; i < 10; ++i) {
        Vec3 vel = v3(frange(rng_, -6.0f, 6.0f), frange(rng_, -6.0f, 6.0f), frange(rng_, 6.0f, 14.0f));
        initPart(allocPart(parts_, nextPart_), pos + v3(frange(rng_, -4.0f, 4.0f), frange(rng_, -4.0f, 4.0f), 2.0f),
                 vel, frange(rng_, 1.6f, 2.4f), frange(rng_, 3.0f, 6.0f), K_SMOKE, pal::SMOKE0, 230);
      }
      break;
    }
    case Burst::CrashSea: {
      for (int i = 0; i < 40; ++i) {
        float a = frange(rng_, 0.0f, TAU), r = frange(rng_, 0.0f, 3.0f);
        Vec3 off = v3(cosf(a) * r, sinf(a) * r, 0.5f);
        Vec3 vel = v3(cosf(a) * frange(rng_, 0.0f, 12.0f), sinf(a) * frange(rng_, 0.0f, 12.0f),
                      frange(rng_, 30.0f, 80.0f));
        initPart(allocPart(parts_, nextPart_), v3(pos.x, pos.y, 0.0f) + off, vel, frange(rng_, 1.0f, 1.8f),
                 frange(rng_, 2.0f, 4.0f), K_SPRAY, pal::SPRAY, 255);
      }
      for (int i = 0; i < 16; ++i) {
        float a = (float)i * (TAU / 16.0f);
        Vec3 radial = v3(cosf(a), sinf(a), 0.0f);
        initPart(allocPart(parts_, nextPart_), v3(pos.x, pos.y, 0.3f) + radial * 3.0f,
                 radial * frange(rng_, 25.0f, 35.0f), frange(rng_, 0.8f, 1.2f), frange(rng_, 2.0f, 3.0f),
                 K_FOAM, pal::FOAM, 255);
      }
      break;
    }
    case Burst::Ignite: {
      for (int i = 0; i < 6; ++i) {
        Vec3 vel = n * frange(rng_, 20.0f, 40.0f) + e1 * frange(rng_, -6.0f, 6.0f) + e2 * frange(rng_, -6.0f, 6.0f) +
                   vel_ * 0.6f;
        initPart(allocPart(parts_, nextPart_), pos, vel, frange(rng_, 0.15f, 0.28f), frange(rng_, 1.5f, 2.5f),
                 K_FIRE, pal::FLAME0, 255);
      }
      for (int i = 0; i < 8; ++i) {
        float a = (float)i * (TAU / 8.0f);
        Vec3 radial = e1 * cosf(a) + e2 * sinf(a);
        initPart(allocPart(parts_, nextPart_), pos + radial * 2.0f, radial * frange(rng_, 14.0f, 20.0f) + vel_ * 0.7f,
                 frange(rng_, 0.3f, 0.45f), frange(rng_, 1.2f, 2.0f), K_VAPOUR, pal::TRAIL0, 220);
      }
      break;
    }
  }
}

void Fx::draw(FrameBuf& fb, const View& v, Tod tod, float time, RenderStats& st) {
  const bool night = tod == Tod::Night;
  const int tw = (int)(time * 12.0f);   // shimmer phase for dithered effects

  // ---- wake V on the sea (oldest first so the young marks sit on top)
  const float eyeZ = v.eyeZ > 1.0f ? v.eyeZ : 1.0f;
  for (int j = 0; j < cfg::WAKE_MARKS; ++j) {
    const WakeMark& m = wake_marks_[(wakeHead_ + j) % cfg::WAKE_MARKS];
    if (m.t <= DEAD_T * 0.5f) continue;
    float age = now_ - m.t;
    if (age < 0.0f || age >= cfg::WAKE_LIFE_S) continue;
    float u = age / cfg::WAKE_LIFE_S;
    float fade = m.k * (1.0f - u) * (1.0f - 0.35f * u);
    int a16 = alpha16(fade * (night ? 0.75f : 0.65f));
    if (a16 <= 0) continue;
    uint8_t col = u < 0.18f ? pal::WAKE_A : pal::WAKE_B;
    float off = 3.0f + 27.0f * pow07(u);
    float rad = 1.7f + 3.4f * u;
    Vec3 across = v3(m.ax, m.ay, 0.0f);
    for (int side = -1; side <= 1; ++side) {
      float r = rad;
      if (side == 0) {
        if (u >= 0.45f) continue;
        r = 2.6f - 2.0f * u;
      }
      ScreenPt sp;
      if (!project(v, m.p + across * (off * (float)side), sp, cfg::CULL_DEN)) continue;
      float squash = eyeZ * rsqrtF(eyeZ * eyeZ + sp.zc * sp.zc);
      if (squash < 0.22f) squash = 0.22f;
      float rx = r * sp.m;
      fillEllipse(fb, sp.x, sp.y, rx, rx * squash, sp.dq, 3, side == 0 ? pal::WAKE_A : col, a16,
                  0.5f + 0.5f * u,
                  night ? tw : 0, night ? (tw >> 2) : 0, true);
    }
  }

  // ---- wingtip trails: newest to oldest; break markers split segments, age ends the walk.
  // Each point is projected once. A segment whose two ends both lie in the line clip box
  // [-1, LW] x [-1, LH] is drawn with drawLine on the rounded ends, which plots exactly what
  // drawLine3D would (no clipping happens, and it rounds the same way) without projecting
  // both ends again; any other segment still goes through drawLine3D.
  const float life = cfg::TRAIL_LIFE_S * (0.3f + 0.7f * g_);
  const float invLife = 1.0f / life;
  for (int k = 0; k < 2; ++k) {
    bool havePrev = false;
    Vec3 prevP = v3(0.0f, 0.0f, 0.0f);
    ScreenPt prevS = {0.0f, 0.0f, 1.0f, 0.0f, 0};
    int idx = trailHead_[k];
    for (int j = 0; j < cfg::TRAIL_POINTS; ++j) {
      const TrailPoint& tp = trail_[k][idx];
      idx = idx == 0 ? cfg::TRAIL_POINTS - 1 : idx - 1;
      if (tp.t <= 0.0f) { havePrev = false; continue; }   // break / never written
      float age = now_ - tp.t;
      if (age > life) break;
      ScreenPt cur;
      if (!project(v, tp.p, cur, cfg::CULL_DEN)) { havePrev = false; continue; }
      if (havePrev) {
        float a = age * invLife;
        float alpha = a < 0.5f ? 1.0f : 2.0f * (1.0f - a);
        uint8_t col = a < 0.3f ? pal::TRAIL0 : (a < 0.65f ? (uint8_t)(pal::TRAIL0 + 1) : (uint8_t)(pal::TRAIL0 + 2));
        int thick = prevS.m >= cfg::JET_SPLAT_BIG_M ? 2 : 1;
        st.linePx += (uint32_t)trailSegment(fb, v, prevP, prevS, tp.p, cur, col, thick, alpha16(alpha));
      }
      prevP = tp.p;
      prevS = cur;
      havePrev = true;
    }
  }

  // ---- speed lines
  if (speedVis_ > 0.02f) {
    Vec3 dir = normalize(vel_);
    float spd = length(vel_);
    float len = 4.0f + 0.08f * spd;
    int a16 = alpha16(speedVis_ * 0.85f);
    for (int i = 0; i < cfg::SPEEDLINES; ++i) {
      const Vec3& sl = speed_[i];
      if (sl.z <= SPEED_UNSET * 0.5f) continue;
      st.linePx += (uint32_t)line3D(fb, v, sl, sl - dir * len, pal::SPEEDLINE, 1, a16);
    }
  }

  // ---- particles
  for (int i = 0; i < cfg::PARTICLES; ++i) {
    const Particle& p = parts_[i];
    if (!alive(p)) continue;
    ScreenPt sp;
    if (!project(v, p.p, sp, cfg::CULL_DEN)) continue;
    if (sp.x < -12.0f || sp.x > (float)LW + 12.0f || sp.y < -12.0f || sp.y > (float)LH + 12.0f) continue;
    float u = p.age / p.life;
    float size = p.size;
    float alpha = 1.0f;
    uint8_t col = p.color;
    bool skipJet = false;
    switch (p.kind) {
      case K_SMOKE: {
        size *= 1.0f + 2.4f * u;
        float fadeIn = sat(p.age / 0.08f);
        float a8 = (float)p.pad[0] * (1.0f / 255.0f);
        float w = 1.0f - u;
        alpha = a8 * w * w * fadeIn * (night ? 0.55f : 1.0f);
        col = (uint8_t)(pal::SMOKE0 + (u < 0.45f ? 0 : (u < 0.8f ? 1 : 2)));
        skipJet = true;
        break;
      }
      case K_SPRAY:
        size *= 1.0f - 0.5f * u;
        alpha = 1.0f - 0.4f * u * u;
        col = u < 0.5f ? pal::SPRAY : pal::FOAM;
        break;
      case K_FOAM:
        size *= 1.0f + 0.8f * u;
        alpha = 0.9f - 0.5f * u;
        col = u < 0.5f ? pal::FOAM : pal::WAKE_B;
        skipJet = true;
        break;
      case K_VAPOUR:
        size *= (1.0f + 0.6f * u) * (1.0f - 0.5f * u * u);
        alpha = (0.85f - 0.45f * u) * ((float)p.pad[0] * (1.0f / 255.0f));
        col = u < 0.5f ? pal::TRAIL0 : (uint8_t)(pal::TRAIL0 + 1);
        break;
      case K_FIRE:
        // Fireball swells then collapses into a small dark ember; no thin dither meshes.
        size *= (u < 0.35f ? 1.0f + 1.6f * u : 1.56f * (1.0f - (u - 0.35f) * 1.1f));
        alpha = u < 0.7f ? 1.0f : 0.75f;
        if (u < 0.10f) col = pal::FLAME0;
        else if (u < 0.22f) col = (uint8_t)(pal::FLAME0 + 1);
        else if (u < 0.38f) col = (uint8_t)(pal::FLAME0 + 2);
        else if (u < 0.55f) col = (uint8_t)(pal::FLAME0 + 3);
        else if (u < 0.72f) col = (uint8_t)(pal::FLAME0 + 4);
        else col = (uint8_t)(pal::SMOKE0 + 1);
        break;
      case K_DEBRIS:
        alpha = u < 0.8f ? 1.0f : (1.0f - u) * 5.0f;
        break;
      case K_SPARK:
        alpha = 1.0f - u;
        if (u < 0.3f) col = pal::SPARK;
        break;
      default:
        break;
    }
    int a16 = alpha16(alpha);
    if (a16 <= 0) continue;
    ++st.particles;
    float rpx = 0.5f * size * sp.m;
    int x = roundi(sp.x), y = roundi(sp.y);
    if (rpx < 0.75f || p.kind == K_DEBRIS) {
      if (!inFrame(x, y) || bayer(x, y) >= a16) continue;
      int q = y * LW + x;
      if (sp.dq > fb.depth[q]) continue;
      if (skipJet && fb.px[q] >= pal::JET_FIRST && fb.px[q] < pal::FLAME0 + 6) continue;
      fb.px[q] = col;
      if (p.kind == K_DEBRIS && sp.m > 1.5f && inFrame(x + 1, y) && sp.dq <= fb.depth[q + 1])
        fb.px[q + 1] = col;
    } else {
      fillEllipse(fb, sp.x, sp.y, rpx, rpx, sp.dq, 0, col, a16, 0.0f, tw & 3, 0, skipJet);
    }
  }
}

void Fx::drawMist(FrameBuf& fb, float amount, uint32_t frame) {
  int a16 = alpha16(amount);
  if (a16 <= 0) return;
  int aJet = a16 / 2;   // keep the hero readable inside the cloud
  int ox = (int)((frame >> 1) & 3u), oy = (int)((frame >> 3) & 3u);
  for (int y = 0; y < LH; ++y) {
    uint8_t* row = fb.px + y * LW;
    // the Bayer row repeats every 4 px: pass masks for the world and the jet coverage
    bool passW[4], passJ[4];
    for (int i = 0; i < 4; ++i) {
      const int b = bayer(i + ox, y + oy);
      passW[i] = b < a16;
      passJ[i] = b < aJet;
    }
    for (int x = 0; x < LW; ++x) {
      const uint8_t c = row[x];
      const bool jet = c >= pal::JET_FIRST && c < pal::FLAME0 + 6;
      if ((jet ? passJ : passW)[x & 3]) row[x] = pal::MIST;
    }
  }
}

}  // namespace kj
