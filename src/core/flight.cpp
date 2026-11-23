// flight.cpp - arcade flight model, crash rules, skim touch, EASY floor, close calls, GPWS.
// DESIGN.md section 10. OWNER: group D. Called in fixed cfg::SIM_STEP sub-steps.
#include "flight.h"
#include "config.h"
#include "mathx.h"
#include "world.h"
#include "fastmath_d.h"

namespace kj {

namespace {

// Ease factors of the fixed sub-step: expf once per distinct dt (bit-identical, see fm::EaseMemo).
fm::EaseMemo e_boostUp(cfg::BOOST_UP), e_boostDown(cfg::BOOST_DOWN), e_yaw(cfg::YAW_EASE),
    e_bank(cfg::BANK_EASE), e_climb(cfg::CLIMB_EASE), e_energy(cfg::ENERGY_EASE),
    e_g(cfg::G_EASE), e_floorUp(cfg::EASY_FLOOR_UP), e_floorDown(cfg::EASY_FLOOR_DOWN),
    e_easyNose(8.0f), e_noseUp(10.0f);

inline void easeM(float& x, float target, fm::EaseMemo& m, float dt) { x += (target - x) * m.get(dt); }

}  // namespace

void Flight::reset(const Vec3& pos, float yaw) {
  s_ = JetState{};
  s_.pos = wrapPos3(pos);
  s_.yaw = wrapPi(yaw);
  s_.speed = cfg::V_CRUISE;
  s_.energy = 1.0f;
  s_.boostEnergy = 1.0f;
  Vec3 f = forwardOf(s_.yaw);
  s_.vel = f * cfg::V_CRUISE;
  s_.heightAboveSurface = pos.z;
  s_.overWater = true;
  easyFloor_ = 0.0f;
  closeCallCd_ = 0.0f;
  wasInCloud_ = false;
  prevBoostCmd_ = 0.0f;
  boostDry_ = false;
  diveHold_ = 0.0f;
  touching_ = false;
}

float Flight::predictedGap(const World& w, float sec) const {
  float best = 1e9f;
  for (int i = 1; i <= 4; ++i) {
    float t = sec * (float)i * 0.25f;
    float x = fm::wrapPos(s_.pos.x + s_.vel.x * t);
    float y = fm::wrapPos(s_.pos.y + s_.vel.y * t);
    float z = s_.pos.z + s_.vel.z * t;
    float gap = z - fm::groundHeight(w, x, y);
    if (gap < best) best = gap;
  }
  return best;
}

JET_D_HOT2 CrashCause Flight::step(const FlightInput& in, const World& w, Assist assist, float dt,
                        FlightEvents& ev) {
  JetState& s = s_;
  if (!(dt > 0.0f)) return CrashCause::None;
  s.time += dt;
  if (s.invuln > 0.0f) s.invuln = s.invuln > dt ? s.invuln - dt : 0.0f;
  const float turnIn = clampf(in.turn, -1.0f, 1.0f);
  const float climbIn = clampf(in.climb, -1.0f, 1.0f);

  // ---------------------------------------------------------------- boost
  bool engaged = prevBoostCmd_ > 0.5f;
  if (in.boost) {
    // After running the tank dry, a NEW press is needed: holding the key must not re-ignite
    // every time the tank regenerates to BOOST_MIN_ENGAGE (sputter cycle).
    if (!engaged && !boostDry_ && s.boostEnergy >= cfg::BOOST_MIN_ENGAGE) engaged = true;
  } else {
    engaged = false;
    boostDry_ = false;
  }
  if (s.boostEnergy <= 0.0f) {
    if (engaged) boostDry_ = true;
    engaged = false;
  }
  if (engaged && prevBoostCmd_ < 0.5f) ev.boostIgnite = true;
  prevBoostCmd_ = engaged ? 1.0f : 0.0f;
  {
    float bt = engaged ? 1.0f : 0.0f;
    easeM(s.boost, bt, bt > s.boost ? e_boostUp : e_boostDown, dt);
    bool skimming = s.overWater && s.heightAboveSurface < cfg::SKIM_BAND;
    if (s.boost > 0.5f)
      s.boostEnergy -= cfg::BOOST_DRAIN * dt;
    else
      s.boostEnergy += (skimming ? cfg::BOOST_REGEN_SKIM : cfg::BOOST_REGEN) * dt;
    s.boostEnergy = clampf(s.boostEnergy, 0.0f, 1.0f);
  }

  // ---------------------------------------------------------------- loop (M6 stretch)
  if (in.loopReq && !s.looping && !s.rolling()) {
    float r = s.speed * cfg::LOOP_TIME / TAU;
    if (s.pos.z > 20.0f && s.pos.z + 2.2f * r < cfg::CEILING - 10.0f) {
      s.looping = true;
      s.loopT = 0.0f;
    }
  }

  // ---------------------------------------------------------------- barrel roll
  if (in.rollReq != 0) {
    if (!s.rolling() && s.rollCooldown <= 0.0f && !s.looping) {
      s.rollDir = in.rollReq < 0 ? (int8_t)-1 : (int8_t)1;
      s.rollT = 0.0f;
      ev.rollStart = true;
    } else {
      s.rollQueued = in.rollReq < 0 ? (int8_t)-1 : (int8_t)1;  // one buffered roll
    }
  }
  if (s.rollCooldown > 0.0f) s.rollCooldown -= dt;
  if (!s.rolling() && s.rollQueued != 0 && s.rollCooldown <= 0.0f && !s.looping) {
    s.rollDir = s.rollQueued;
    s.rollQueued = 0;
    s.rollT = 0.0f;
    ev.rollStart = true;
  }
  float dashV = 0.0f;
  if (s.rolling()) {
    s.rollT += dt / cfg::ROLL_TIME;
    if (s.rollT >= 1.0f) {
      s.rollT = 0.0f;
      s.rollDir = 0;
      s.rollAngle = 0.0f;
      s.rollCooldown = cfg::ROLL_COOLDOWN;
    } else {
      s.rollAngle = (float)s.rollDir * TAU * easeInOutQuad(s.rollT);
      // lateral dash: integral over the roll = ROLL_DASH_U
      dashV = (float)s.rollDir * cfg::ROLL_DASH_U * PI / (2.0f * cfg::ROLL_TIME) *
              sinf(PI * s.rollT);
    }
  }

  // ---------------------------------------------------------------- turning
  const float maxYaw = lerpf(cfg::MAX_YAW, cfg::MAX_YAW_BOOST, s.boost);
  const float want = s.looping ? 0.0f : turnIn * maxYaw;
  easeM(s.yawRate, want, e_yaw, dt);
  s.yaw = fm::wrapPi(s.yaw + s.yawRate * dt);
  const float yawN = s.yawRate / cfg::MAX_YAW;
  easeM(s.bank, clampf(yawN, -1.0f, 1.0f) * cfg::BANK_MAX, e_bank, dt);

  // ---------------------------------------------------------------- climbing
  if (s.looping) {
    s.loopT += dt / cfg::LOOP_TIME;
    if (s.loopT >= 1.0f) {
      s.loopT = 0.0f;
      s.looping = false;
      s.pitch = 0.0f;
      s.pitchRate = 0.0f;
      ev.loopDone = true;
    } else {
      // slow in, faster over the top, slow out
      float np = TAU * smooth01(s.loopT);
      s.pitchRate = (np - s.pitch) / dt;
      s.pitch = np;
    }
  } else {
    if (s.pitch > PI) s.pitch -= TAU;  // safety after an aborted loop
    float ask = climbIn > 0.0f ? climbIn * cfg::CLIMB_MAX : climbIn * cfg::DIVE_MAX;
    ask += cfg::TURN_NOSE_UP * fabsf(yawN);
    float d = (ask - s.pitch) * e_climb.get(dt);
    d = clampf(d, -cfg::CLIMB_RATE_DOWN * dt, cfg::CLIMB_RATE_UP * dt);
    s.pitch += d;
    s.pitchRate = d / dt;
  }
  diveHold_ = climbIn <= -0.95f ? diveHold_ + dt : 0.0f;

  // ---------------------------------------------------------------- speed and energy
  const float spch = sinf(s.pitch);  // also used by the motion and probes below
  {
    const float sp = spch;
    float target = 1.0f - cfg::CLIMB_BLEED * sp * (s.pitch > 0.0f ? 1.0f - s.boost : 1.0f);
    target = clampf(target, 0.8f, cfg::ENERGY_MAX);
    easeM(s.energy, target, e_energy, dt);
    s.energy = clampf(s.energy, 0.8f, cfg::ENERGY_MAX);
    s.speed = cfg::V_CRUISE * (1.0f + (cfg::BOOST_MUL - 1.0f) * s.boost) * s.energy;
  }

  // ---------------------------------------------------------------- motion
  const float syaw = sinf(s.yaw), cyaw = cosf(s.yaw);
  const Vec3 f = v3(syaw, -cyaw, 0.0f), r = v3(cyaw, syaw, 0.0f);  // forwardOf / rightOf
  const float cp = cosf(s.pitch);
  float dz = s.speed * spch * dt;
  if (dz > 0.0f && !s.looping) {
    float gap = cfg::CEILING - s.pos.z;
    if (gap < cfg::CEIL_CUSHION) dz *= (gap > 0.0f ? gap : 0.0f) / cfg::CEIL_CUSHION;
  }
  if (fabsf(climbIn) < 0.05f && !s.looping)
    dz += cfg::BOB_AMP * cosf(cfg::BOB_FREQ * s.time) * dt;
  const float hs = s.speed * cp * dt;
  const float dx = f.x * hs + r.x * dashV * dt;
  const float dy = f.y * hs + r.y * dashV * dt;
  s.pos.x = fm::wrapPos(s.pos.x + dx);
  s.pos.y = fm::wrapPos(s.pos.y + dy);
  s.pos.z += dz;
  if (s.pos.z > cfg::CEILING) s.pos.z = cfg::CEILING;
  s.vel = v3(dx / dt, dy / dt, dz / dt);

  // ---------------------------------------------------------------- g gauge
  {
    float gt = fabsf(s.yawRate) / cfg::MAX_YAW + (s.rolling() ? 0.7f : 0.0f) + 0.9f * s.boost +
               fabsf(s.pitchRate) / cfg::CLIMB_RATE_UP;
    easeM(s.g, clampf(gt, 0.0f, 1.0f), e_g, dt);
  }

  // ---------------------------------------------------------------- surface queries
  const uint8_t cell0 = fm::cellAt(w, s.pos.x, s.pos.y);
  bool water = cellIsWater(cell0);
  float gnd = water ? 0.0f : fm::cellHeight(cell0);

  // ---------------------------------------------------------------- EASY terrain floor
  const bool easy = assist == Assist::Easy;
  const bool deliberateDive = s.pitch < -0.6f && diveHold_ >= 0.4f;
  if (easy) {
    float maxG = -1.0f;
    for (int i = 0; i < 5; ++i) {
      float t = 0.16f * (float)i;
      float ax = s.pos.x + f.x * s.speed * t, ay = s.pos.y + f.y * s.speed * t;
      for (int j = -1; j <= 1; ++j) {
        float h = fm::landHeightWrapped(w, ax + r.x * 7.0f * (float)j, ay + r.y * 7.0f * (float)j);
        if (h > maxG) maxG = h;
      }
    }
    float target = maxG > 0.0f ? maxG + cfg::EASY_FLOOR_CLEAR : 0.0f;
    easeM(easyFloor_, target, target > easyFloor_ ? e_floorUp : e_floorDown, dt);
    if (maxG > 0.0f && !deliberateDive && s.pos.z < easyFloor_) {
      s.pos.z = easyFloor_;
      if (s.vel.z < 0.0f) s.vel.z = 0.0f;
      if (s.pitch < 0.0f) s.pitch -= s.pitch * e_easyNose.get(dt);
    }
  } else {
    easyFloor_ = 0.0f;
  }

  // ---------------------------------------------------------------- crash rules
  CrashCause crash = CrashCause::None;
  const bool canCrash = s.invuln <= 0.0f && (!easy || deliberateDive);

  // terrain: pivot and nose probe (land only)
  if (!water && s.pos.z < gnd + cfg::TERRAIN_CLEAR) {
    if (canCrash) crash = CrashCause::Terrain;
    else s.pos.z = gnd + cfg::TERRAIN_CLEAR;  // forgiven: lift out of the ground
  }
  if (crash == CrashCause::None) {
    float nx = fm::wrapPos(s.pos.x + f.x * cfg::NOSE_PROBE), ny = fm::wrapPos(s.pos.y + f.y * cfg::NOSE_PROBE);
    float nz = s.pos.z + cfg::NOSE_PROBE * spch;
    const uint8_t nc = fm::cellAt(w, nx, ny);
    if (!cellIsWater(nc)) {
      float ng = fm::cellHeight(nc);
      if (nz < ng + cfg::NOSE_CLEAR) {
        if (canCrash) crash = CrashCause::Terrain;
        else if (s.pitch < 0.0f) s.pitch -= s.pitch * e_noseUp.get(dt);  // nose up
      }
    }
  }

  // sea: crash on a hard or steep impact, otherwise a skim touch
  if (crash == CrashCause::None && water && s.pos.z <= cfg::SKIM_Z) {
    bool hard = s.vel.z < cfg::SEA_CRASH_VZ || s.pitch < cfg::SEA_CRASH_PITCH;
    if (hard && canCrash) {
      crash = CrashCause::Sea;
    } else {
      s.pos.z = cfg::SKIM_Z;
      s.vel.z = 0.0f;
      if (s.pitch < 0.0f) s.pitch = 0.0f;
      if (s.pitchRate < 0.0f) s.pitchRate = 0.0f;
      if (!touching_) ev.skimTouch = true;
      touching_ = true;
    }
  } else if (s.pos.z > cfg::SKIM_Z + 0.6f) {
    touching_ = false;
  }

  s.overWater = water;
  s.heightAboveSurface = s.pos.z - gnd;

  if (crash != CrashCause::None) return crash;

  // ---------------------------------------------------------------- close call (5 probes)
  if (closeCallCd_ > 0.0f) closeCallCd_ -= dt;
  if (closeCallCd_ <= 0.0f && s.invuln <= 0.0f) {
    const float sb = sinf(s.bank), cb = cosf(s.bank);
    const float half = cfg::JET_SPAN_U * 0.5f;
    const float px[5] = {0.0f, f.x * 17.0f, -f.x * 15.0f, r.x * half * cb, -r.x * half * cb};
    const float py[5] = {0.0f, f.y * 17.0f, -f.y * 15.0f, r.y * half * cb, -r.y * half * cb};
    const float pz[5] = {0.0f, 17.0f * spch, -15.0f * spch, -half * sb, half * sb};
    for (int i = 0; i < 5; ++i) {
      float h = fm::landHeightWrapped(w, s.pos.x + px[i], s.pos.y + py[i]);
      if (h < 0.0f) continue;
      if (s.pos.z + pz[i] - h < cfg::CLOSE_CALL_DIST) {
        ev.closeCall = true;
        closeCallCd_ = cfg::CLOSE_CALL_COOLDOWN;
        break;
      }
    }
  }

  // ---------------------------------------------------------------- GPWS
  // Land: sinking fast toward terrain. Sea: only when the contact would actually be a crash
  // (with a 20 % margin); a gentle descent onto the water is a safe skim touch and must not
  // beep, or players learn to ignore the warning.
  {
    const bool sink = s.vel.z < -cfg::GPWS_SINK;
    const bool seaHard = s.vel.z < 0.0f && (s.vel.z < cfg::SEA_CRASH_VZ * 0.8f ||
                                            s.pitch < cfg::SEA_CRASH_PITCH * 0.8f);
    ev.pullUp = false;
    if (sink || seaHard) {
      float landGap = 1e9f, anyGap = 1e9f;
      for (int i = 1; i <= 4; ++i) {  // same probes as predictedGap()
        const float t = cfg::GPWS_LOOKAHEAD * (float)i * 0.25f;
        const float z = s.pos.z + s.vel.z * t;
        const float lh = fm::landHeightWrapped(w, s.pos.x + s.vel.x * t, s.pos.y + s.vel.y * t);
        const float gap = z - (lh > 0.0f ? lh : 0.0f);
        if (gap < anyGap) anyGap = gap;
        if (lh >= 0.0f && gap < landGap) landGap = gap;
      }
      ev.pullUp = (sink && landGap < cfg::GPWS_GAP) || (seaHard && anyGap < cfg::GPWS_GAP);
    }
  }

  // ---------------------------------------------------------------- clouds
  bool inCloud = fm::insideCloud(w, s.pos.x, s.pos.y, s.pos.z);
  if (inCloud && !wasInCloud_) ev.enteredCloud = true;
  if (!inCloud && wasInCloud_) ev.exitedCloud = true;
  wasInCloud_ = inCloud;
  ev.inCloud = inCloud;

  return CrashCause::None;
}

}  // namespace kj
