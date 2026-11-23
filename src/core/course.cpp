// course.cpp - ring gates: chained placement, pass/miss detection, magnet assist, drawing and
// the off-screen guide (DESIGN.md section 11.2). OWNER: group D. Uses draw.h (group C).
#include "course.h"
#include "mathx.h"
#include "flight.h"
#include "world.h"
#include "draw.h"
#include "palette.h"
#include "fastmath_d.h"

namespace kj {

namespace {

constexpr int RIM_POINTS = 20;

// cos/sin of the rim angles i * TAU / RIM_POINTS (same expressions as before, computed once:
// drawing used to call cosf + sinf 20 times per ring per frame).
float g_rimC[RIM_POINTS], g_rimS[RIM_POINTS];
bool g_rimInit = false;
fm::GlowStamp g_farGlow;  // == drawGlow(.., 1.0f, 3.0f, .., 6) of the far ring beacon

void initDrawTables() {
  if (g_rimInit) return;
  for (int i = 0; i < RIM_POINTS; ++i) {
    float a = (float)i * (TAU / (float)RIM_POINTS);
    g_rimC[i] = cosf(a);
    g_rimS[i] = sinf(a);
  }
  g_farGlow.build(1.0f, 3.0f, 6);
  g_rimInit = true;
}

inline uint32_t rngNext(uint32_t& s) {
  s ^= s << 13;
  s ^= s >> 17;
  s ^= s << 5;
  return s;
}
inline float rngUniform(uint32_t& s) { return (float)(rngNext(s) >> 8) * (1.0f / 16777216.0f); }
inline float rngRange(uint32_t& s, float a, float b) { return a + (b - a) * rngUniform(s); }

inline float landH(const World& w, float x, float y) { return fm::landHeightWrapped(w, x, y); }

// cos/sin of the 8 probe directions k * pi/4 (and k * pi/4 + 0.2) used by the ring placement
// (same expressions as the per-call cosf/sinf they replace).
float g_oct8C[8], g_oct8S[8], g_oct8bC[8], g_oct8bS[8];
bool g_oct8Init = false;
void initOct8() {
  if (g_oct8Init) return;
  for (int k = 0; k < 8; ++k) {
    float a = (float)k * (PI * 0.25f);
    g_oct8C[k] = cosf(a);
    g_oct8S[k] = sinf(a);
    float b = (float)k * (PI * 0.25f) + 0.2f;
    g_oct8bC[k] = cosf(b);
    g_oct8bS[k] = sinf(b);
  }
  g_oct8Init = true;
}

float ringRadius(int index) {
  float d = sat((float)(index - 1) / (float)cfg::RING_DIFF_RINGS);
  return lerpf(cfg::RING_R_START, cfg::RING_R_END, d);
}

// LOW ring site: open water within 40 u of the centre, some land 40..90 u away.
bool lowSiteOk(const World& w, float cx, float cy) {
  initOct8();
  for (int k = 0; k < 8; ++k) {
    const float ca = g_oct8C[k], sa = g_oct8S[k];
    if (landH(w, cx + ca * 20.0f, cy + sa * 20.0f) >= 0.0f) return false;
    if (landH(w, cx + ca * 38.0f, cy + sa * 38.0f) >= 0.0f) return false;
  }
  if (landH(w, cx, cy) >= 0.0f) return false;
  for (int k = 0; k < 8; ++k) {
    const float ca = g_oct8bC[k], sa = g_oct8bS[k];
    for (int j = 0; j < 3; ++j) {
      float r = 55.0f + 17.5f * (float)j;
      if (landH(w, cx + ca * r, cy + sa * r) >= 0.0f) return true;
    }
  }
  return false;
}

// Raise needed so that every rim point over land clears the ground by RING_RIM_CLEAR.
float rimRaise(const World& w, const Vec3& c, float yaw, float R) {
  Vec3 rt = rightOf(yaw);
  float raise = 0.0f;
  initOct8();
  for (int k = 0; k < 8; ++k) {
    const float ca = g_oct8C[k];
    float px = c.x + rt.x * R * ca, py = c.y + rt.y * R * ca;
    float pz = c.z + R * g_oct8S[k];
    float h = landH(w, px, py);
    if (h < 0.0f) continue;
    float need = h + cfg::RING_RIM_CLEAR - pz;
    if (need > raise) raise = need;
  }
  // the centre itself (jet passes there)
  float hc = landH(w, c.x, c.y);
  if (hc >= 0.0f && hc + cfg::RING_RIM_CLEAR + R * 0.5f - c.z > raise)
    raise = hc + cfg::RING_RIM_CLEAR + R * 0.5f - c.z;
  return raise;
}

// Straight path from a to b clear of terrain by `clear` u?
bool pathClear(const World& w, const Vec3& a, const Vec3& b, float clear) {
  Vec3 d = wrapDelta(b, a);
  for (int k = 1; k <= 8; ++k) {
    float t = (float)k / 9.0f;
    float h = landH(w, a.x + d.x * t, a.y + d.y * t);
    if (h < 0.0f) continue;
    float z = a.z + d.z * t;
    if (h + clear > z) return false;
  }
  return true;
}

float surfaceAt(const World& w, float x, float y) {
  float h = landH(w, x, y);
  return h < 0.0f ? 0.0f : h;
}

// Place ring number `index` (1-based) after a ring/jet at `from` heading `fromYaw`.
Ring makeRing(const World& w, const Vec3& from, float fromYaw, int index, uint32_t& rng,
              int8_t& lastSign) {
  const float d = sat((float)(index - 1) / (float)cfg::RING_DIFF_RINGS);
  const float dist = lerpf(cfg::RING_DIST_START, cfg::RING_DIST_END, d);
  const float tmin = lerpf(cfg::RING_TURN_MIN0, cfg::RING_TURN_MIN1, d);
  const float tmax = lerpf(cfg::RING_TURN_MAX0, cfg::RING_TURN_MAX1, d);
  const float R = ringRadius(index);

  // turn direction alternates with p = 0.65
  int8_t sign = rngUniform(rng) < 0.65f ? (int8_t)-lastSign : lastSign;
  lastSign = sign;

  // altitude class
  float pc = rngUniform(rng);
  RingKind kind = RingKind::Mid;
  if (pc < cfg::RING_P_LOW) kind = RingKind::Low;
  else if (pc < cfg::RING_P_LOW + cfg::RING_P_MID || index < cfg::RING_HIGH_FROM) kind = RingKind::Mid;
  else kind = RingKind::High;

  Ring ring = {};
  ring.radius = R;
  ring.live = true;
  ring.passAnim = 0.0f;

  for (int attempt = 0; attempt < 6; ++attempt) {
    float dh = (float)sign * rngRange(rng, tmin, tmax);
    if (attempt >= 3) dh = -dh;  // try the other side after a few failures
    float yaw = fm::wrapPi(fromYaw + dh);
    Vec3 c = {0.0f, 0.0f, 0.0f};
    if (kind == RingKind::Low) {
      // try 4 heading candidates, keep one with land nearby but none inside
      bool found = false;
      Vec3 cand = {0.0f, 0.0f, 0.0f};
      float candYaw = yaw;
      for (int k = 0; k < 4 && !found; ++k) {
        float y2 = k == 0 ? yaw : fm::wrapPi(fromYaw + (float)sign * rngRange(rng, tmin, tmax));
        Vec3 f2 = forwardOf(y2);
        Vec3 c2 = v3(fm::wrapPos(from.x + f2.x * dist), fm::wrapPos(from.y + f2.y * dist), 0.0f);
        if (k == 0) { cand = c2; candYaw = y2; }
        if (lowSiteOk(w, c2.x, c2.y)) { cand = c2; candYaw = y2; found = true; }
      }
      yaw = candYaw;
      c = cand;
      c.z = surfaceAt(w, c.x, c.y) + rngRange(rng, cfg::RING_ALT_LOW_MIN, cfg::RING_ALT_LOW_MAX);
      // keep the whole rim above the water (a 10..20 u centre with R 14..20 would sink the
      // lower arc into the sea): bottom of the rim at least 3 u above the surface
      if (c.z < surfaceAt(w, c.x, c.y) + R + 3.0f) c.z = surfaceAt(w, c.x, c.y) + R + 3.0f;
    } else {
      Vec3 f = forwardOf(yaw);
      c = v3(fm::wrapPos(from.x + f.x * dist), fm::wrapPos(from.y + f.y * dist), 0.0f);
      if (kind == RingKind::Mid)
        c.z = surfaceAt(w, c.x, c.y) + rngRange(rng, cfg::RING_ALT_MID_MIN, cfg::RING_ALT_MID_MAX);
      else
        c.z = rngRange(rng, cfg::RING_ALT_HIGH_MIN, cfg::RING_ALT_HIGH_MAX);
    }
    float raise = rimRaise(w, c, yaw, R);
    if (raise > 0.0f) c.z += raise;
    bool ok = raise < 30.0f && pathClear(w, from, c, 10.0f) &&
              c.z < cfg::CEILING - R - 15.0f;
    if (ok) {
      ring.c = c;
      ring.yaw = yaw;
      ring.kind = kind;
      return ring;
    }
  }

  // Fallback: a MID ring over water (or high enough to clear everything on the way).
  float bestYaw = fm::wrapPi(fromYaw + (float)sign * tmin);
  Vec3 bestC = {0.0f, 0.0f, 0.0f};
  bool water = false;
  for (int k = 0; k < 8 && !water; ++k) {
    float yaw = fm::wrapPi(fromYaw + (float)((k & 1) ? -1 : 1) * (tmin + 0.15f * (float)(k >> 1)));
    Vec3 f = forwardOf(yaw);
    Vec3 c = v3(fm::wrapPos(from.x + f.x * dist), fm::wrapPos(from.y + f.y * dist), 0.0f);
    if (k == 0 || landH(w, c.x, c.y) < 0.0f) {
      bestYaw = yaw;
      bestC = c;
      water = landH(w, c.x, c.y) < 0.0f;
    }
  }
  float maxG = 0.0f;
  Vec3 dd = wrapDelta(bestC, from);
  for (int k = 0; k <= 8; ++k) {
    float t = (float)k / 8.0f;
    float h = surfaceAt(w, from.x + dd.x * t, from.y + dd.y * t);
    if (h > maxG) maxG = h;
  }
  bestC.z = maxG + rngRange(rng, cfg::RING_ALT_MID_MIN, cfg::RING_ALT_MID_MAX);
  float raise = rimRaise(w, bestC, bestYaw, R);
  if (raise > 0.0f) bestC.z += raise;
  ring.c = bestC;
  ring.yaw = bestYaw;
  ring.kind = RingKind::Mid;
  return ring;
}

// Draw one ring as RIM_POINTS projected segments. Returns pixels attempted.
int drawRingLines(FrameBuf& fb, const View& v, const Ring& r, float scale, uint8_t colTopLeft,
                  uint8_t colBottomRight, int alpha, bool writeDepth, int thickOverride) {
  ScreenPt sc;
  if (!project(v, r.c, sc, cfg::CULL_DEN)) return 0;
  const float R = r.radius * scale;
  int thick = thickOverride > 0 ? thickOverride : (sc.m * R > 10.0f ? 2 : 1);
  Vec3 rt = rightOf(r.yaw);
  ScreenPt pts[RIM_POINTS];
  bool ok[RIM_POINTS];
  bool any = false;
  for (int i = 0; i < RIM_POINTS; ++i) {
    const float ca = g_rimC[i], sa = g_rimS[i];
    Vec3 p = v3(r.c.x + rt.x * R * ca, r.c.y + rt.y * R * ca, r.c.z + R * sa);
    ok[i] = project(v, p, pts[i], cfg::CULL_DEN);
    any = any || ok[i];
  }
  if (!any) return 0;
  int count = 0;
  for (int i = 0; i < RIM_POINTS; ++i) {
    int j = (i + 1) % RIM_POINTS;
    if (!ok[i] || !ok[j]) continue;
    float x0 = pts[i].x, y0 = pts[i].y, x1 = pts[j].x, y1 = pts[j].y;
    // reject absurd segments (very close to the camera plane)
    if (fabsf(x0) > 2000.0f || fabsf(y0) > 2000.0f || fabsf(x1) > 2000.0f || fabsf(y1) > 2000.0f)
      continue;
    float mx = 0.5f * (x0 + x1) - sc.x, my = 0.5f * (y0 + y1) - sc.y;
    uint8_t col = (mx + my) < 0.0f ? colTopLeft : colBottomRight;  // top-left half lit
    int ix0 = fm::floorI(x0), iy0 = fm::floorI(y0), ix1 = fm::floorI(x1), iy1 = fm::floorI(y1);
    if (fm::lineInBox(ix0, iy0, ix1, iy1))  // same pixels as drawLine, without its clip divides
      fm::lineNoClip(fb, ix0, iy0, pts[i].dq, ix1, iy1, pts[j].dq, col, thick, alpha, writeDepth);
    else
      drawLine(fb, ix0, iy0, pts[i].dq, ix1, iy1, pts[j].dq, col, thick, alpha, writeDepth);
    int dx = ix1 - ix0, dy = iy1 - iy0;
    count += (dx < 0 ? -dx : dx) > (dy < 0 ? -dy : dy) ? (dx < 0 ? -dx : dx) + 1 : (dy < 0 ? -dy : dy) + 1;
  }
  return count;
}

}  // namespace

void Course::reset(const JetState& s, const World& w, uint32_t seed) {
  seed_ = seed ? seed : 0x5EEDu;
  rngNext(seed_);
  cur_ = 0;
  passed_ = 0;
  spawned_ = 0;
  sinceProgress_ = 0.0f;
  lastDist_ = 0.0f;
  lastTurnSign_ = (seed_ & 1) ? (int8_t)1 : (int8_t)-1;
  ghost_ = Ring{};
  Vec3 from = s.pos;
  float yaw = s.yaw;
  for (int i = 0; i < cfg::LIVE_RINGS; ++i) {
    ++spawned_;
    rings_[i] = makeRing(w, from, yaw, (int)spawned_, seed_, lastTurnSign_);
    from = rings_[i].c;
    yaw = rings_[i].yaw;
  }
  Vec3 d = wrapDelta(rings_[0].c, s.pos);
  lastDist_ = length(d);
}

bool Course::current(Ring& out) const {
  out = rings_[cur_];
  return out.live;
}

RingEvent Course::update(const Vec3& prevPos, const JetState& s, const World& w, float dt) {
  RingEvent ev = {};
  ev.result = RingResult::None;

  if (ghost_.live) {
    ghost_.passAnim += dt;
    if (ghost_.passAnim > 0.3f) ghost_.live = false;
  }
  Ring& r = rings_[cur_];
  if (!r.live) return ev;
  sinceProgress_ += dt;

  const Vec3 n = forwardOf(r.yaw);
  const Vec3 rel0 = wrapDelta(prevPos, r.c);
  const Vec3 rel1 = wrapDelta(s.pos, r.c);
  const float d0 = dot(rel0, n), d1 = dot(rel1, n);
  const float dist = length(rel1);
  lastDist_ = dist;

  bool crossed = (d0 < 0.0f) != (d1 < 0.0f) && fabsf(d0 - d1) > 1e-6f &&
                 length(wrapDelta(s.pos, prevPos)) < 200.0f;  // ignore teleports
  RingResult res = RingResult::None;
  float lateral = 0.0f;
  if (crossed) {
    float t = d0 / (d0 - d1);
    Vec3 p = rel0 + (rel1 - rel0) * t;
    float along = dot(p, n);
    float l2 = dot(p, p) - along * along;
    lateral = sqrtf(l2 > 0.0f ? l2 : 0.0f);
    if (lateral <= cfg::RING_PERFECT_R) res = RingResult::Perfect;
    else if (lateral <= r.radius) res = RingResult::Through;
    else res = RingResult::Missed;
  } else if (dist > cfg::RING_MISS_DIST || sinceProgress_ > cfg::RING_MISS_TIME) {
    res = RingResult::Missed;
    lateral = dist;
  }
  if (res == RingResult::None) return ev;

  ev.result = res;
  ev.lateral = lateral;
  ev.index = (uint16_t)(passed_ + 1);
  ev.pos = r.c;
  ev.normal = n;
  ev.kind = r.kind;
  sinceProgress_ = 0.0f;

  if (res != RingResult::Missed) {
    ghost_ = r;
    ghost_.passAnim = 0.0001f;
    ++passed_;
    // replace the passed ring with a new one after the last live ring
    const Ring& last = rings_[(cur_ + cfg::LIVE_RINGS - 1) % cfg::LIVE_RINGS];
    ++spawned_;
    Ring nr = makeRing(w, last.c, last.yaw, (int)spawned_, seed_, lastTurnSign_);
    rings_[cur_] = nr;
    cur_ = (cur_ + 1) % cfg::LIVE_RINGS;
  } else {
    // re-place the whole chain relative to the jet's current pose
    spawned_ = passed_;
    cur_ = 0;
    Vec3 from = s.pos;
    float yaw = s.yaw;
    for (int i = 0; i < cfg::LIVE_RINGS; ++i) {
      ++spawned_;
      rings_[i] = makeRing(w, from, yaw, (int)spawned_, seed_, lastTurnSign_);
      from = rings_[i].c;
      yaw = rings_[i].yaw;
    }
  }
  return ev;
}

void Course::assist(const JetState& s, Assist a, float playerTurn, float playerClimb,
                    float& turnAdd, float& climbAdd) const {
  turnAdd = 0.0f;
  climbAdd = 0.0f;
  const Ring& r = rings_[cur_];
  if (!r.live) return;
  const float gain = cfg::ASSIST_GAIN[(int)a <= 2 ? (int)a : 1];
  if (gain <= 0.0f) return;
  Vec3 d = wrapDelta(r.c, s.pos);
  if (length(d) > cfg::MAGNET_RANGE) return;
  float bearing = atan2f(d.x, -d.y);  // heading that points at the ring
  float err = fm::wrapPi(bearing - s.yaw);
  if (fabsf(err) > cfg::MAGNET_CONE) return;
  turnAdd = clampf(cfg::MAGNET_TURN_K * err, -cfg::MAGNET_TURN_MAX, cfg::MAGNET_TURN_MAX) * gain *
            (1.0f - sat(fabsf(playerTurn)));
  climbAdd = clampf(cfg::MAGNET_CLIMB_K * d.z, -cfg::MAGNET_CLIMB_MAX, cfg::MAGNET_CLIMB_MAX) *
             gain * (1.0f - sat(fabsf(playerClimb)));
}

void Course::draw(FrameBuf& fb, const View& v, Tod tod, float time, RenderStats& st) const {
  const Ring& cur = rings_[cur_];
  const Ring& next = rings_[(cur_ + 1) % cfg::LIVE_RINGS];
  const Ring& far = rings_[(cur_ + 2) % cfg::LIVE_RINGS];
  const bool night = tod == Tod::Night;
  initDrawTables();

  // far ring: beacon dot + 6-dot light column down to the surface (chasing upward)
  if (far.live) {
    ScreenPt sp;
    if (project(v, far.c, sp, cfg::CULL_DEN)) {
      int x = fm::floorI(sp.x), y = fm::floorI(sp.y);
      if (night) g_farGlow.draw(fb, x, y, sp.dq, pal::RING_GLOW);
      drawPoint(fb, x, y, sp.dq, pal::BEACON, false);
      drawPoint(fb, x + 1, y, sp.dq, pal::BEACON, false);
      int phase = (int)(time * 8.0f) % 6;
      for (int k = 1; k <= 6; ++k) {
        Vec3 p = v3(far.c.x, far.c.y, far.c.z * (1.0f - (float)k / 7.0f));
        ScreenPt q;
        if (!project(v, p, q, cfg::CULL_DEN)) continue;
        uint8_t col = (6 - k) == phase ? pal::RING_GLOW : pal::BEACON;
        drawPoint(fb, fm::floorI(q.x), fm::floorI(q.y), q.dq, col, false);
        st.linePx += 1;
      }
    }
  }

  // next ring: cyan, 60 % dither
  if (next.live)
    st.linePx += (uint32_t)drawRingLines(fb, v, next, 1.0f, pal::RING_CYAN, pal::RING_CYAN,
                                         alpha16(0.6f), true, 0);

  // current ring: amber, pulsing at 3 Hz, night halo
  if (cur.live) {
    float pulse = 0.5f + 0.5f * sinf(TAU * 3.0f * time);
    if (night)
      st.linePx += (uint32_t)drawRingLines(fb, v, cur, 1.14f, pal::RING_GLOW, pal::RING_GLOW,
                                           3 + (int)(4.0f * pulse), false, 1);
    uint8_t lit = pulse > 0.8f ? pal::RING_GLOW : pal::RING_GOLD;
    st.linePx += (uint32_t)drawRingLines(fb, v, cur, 1.0f + 0.03f * pulse, lit,
                                         pal::RING_GOLD_DARK, 16, true, 0);
  }

  // pass animation: scale 1 -> 1.6 over 0.3 s, fading
  if (ghost_.live) {
    float p = sat(ghost_.passAnim / 0.3f);
    int a = (int)(16.0f * (1.0f - p) + 0.5f);
    if (a > 0)
      st.linePx += (uint32_t)drawRingLines(fb, v, ghost_, 1.0f + 0.6f * p, pal::RING_GLOW,
                                           pal::RING_GOLD, a, false, 0);
  }
}

bool Course::guide(const View& v, int& edgeX, int& edgeY, int& vertical) const {
  const Ring& r = rings_[cur_];
  if (!r.live) return false;
  ScreenPt sp;
  bool ok = project(v, r.c, sp, cfg::CULL_DEN);
  if (ok && sp.x >= 0.0f && sp.x < (float)LW && sp.y >= 0.0f && sp.y < (float)LH) return false;

  const float cx = v.Cx, cy = clampf(v.Cy, 12.0f, (float)LH - 14.0f);
  float dx, dy;
  if (ok) {
    dx = sp.x - cx;
    dy = sp.y - cy;
  } else {
    Vec3 d = wrapDelta(r.c, v3(v.fx, v.fy, v.alt));
    float across = d.x * v.cpsi + d.y * v.spsi;
    dx = across >= 0.0f ? 1.0f : -1.0f;
    dy = 0.0f;
  }
  if (fabsf(dx) < 1e-4f && fabsf(dy) < 1e-4f) dx = 1.0f;
  // clip the ray from the screen centre to an inner box (clear of the HUD corners)
  const float x0 = 3.0f, x1 = (float)LW - 4.0f, y0 = 10.0f, y1 = (float)LH - 13.0f;
  float t = 1e9f;
  if (dx > 0.0f) t = fminf(t, (x1 - cx) / dx);
  if (dx < 0.0f) t = fminf(t, (x0 - cx) / dx);
  if (dy > 0.0f) t = fminf(t, (y1 - cy) / dy);
  if (dy < 0.0f) t = fminf(t, (y0 - cy) / dy);
  if (t > 1e8f || t < 0.0f) t = 0.0f;
  edgeX = clampi((int)(cx + dx * t), (int)x0, (int)x1);
  edgeY = clampi((int)(cy + dy * t), (int)y0, (int)y1);
  float dz = r.c.z - v.alt;
  vertical = dz > 30.0f ? 1 : (dz < -30.0f ? -1 : 0);
  return true;
}

}  // namespace kj
