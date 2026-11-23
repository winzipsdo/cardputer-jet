// game.cpp - state machine, input mapping, scoring and juice, render order, persistence,
// telemetry and simulator hooks (DESIGN.md sections 8, 9, 11, 14, 15). OWNER: group D.
#include "game.h"
#include <stdarg.h>
#include <stdio.h>
#include <string.h>
#include "mathx.h"
#include "render_world.h"
#include "draw.h"
#include "fastmath_d.h"

namespace kj {

namespace {

// The logical frame lives in .bss (8,160 B each), never on the heap.
uint8_t s_px[cfg::LPIX];
uint8_t s_depth[cfg::LPIX];

const char* const STATE_NAMES[] = {"BOOT", "CALIBRATE", "TITLE", "COUNTDOWN",
                                   "PLAY", "PAUSE", "CRASH", "GAMEOVER"};
const char* const KIND_NAMES[] = {"THROUGH", "PERFECT", "MISSED"};
const char* const SENS_NAMES[] = {"LOW", "MED", "HIGH"};
const char* const ASSIST_NAMES[] = {"EASY", "NORMAL", "PRO"};
const char* const TOD_NAMES[] = {"NIGHT", "SUNSET", "DAY"};
const char* const MENU_LABELS[MENU_ITEMS] = {"RESUME", "TILT", "SENSITIVITY", "INVERT", "ASSIST",
                                             "TIME", "VOLUME", "BRIGHTNESS", "QUIT"};

constexpr float ASSEMBLE_S = 0.85f;   // = (0.05 * 11 + 0.3) s, render_jet's slice-drop span
constexpr float FLASH_DECAY = 4.0f;
constexpr float POPUP_LIFE = 0.8f;
constexpr float CENTRE_LIFE = 1.2f;
constexpr float BENCH_S = 20.0f;
// The core's own bench line is printed this long after the 20 s mark, so it never lands in the
// same serial burst as the platform's [BENCH] summary (main.cpp), which is printed at 20 s.
constexpr float BENCH_LOG_DELAY_S = 0.5f;

// Linear key ramp: toward +-1 at KEY_RAMP_UP, back toward 0 at KEY_RAMP_DOWN.
void rampKey(float& v, float target, float dt) {
  if (target == 0.0f || v * target < 0.0f) {
    float step = cfg::KEY_RAMP_DOWN * dt;
    if (fabsf(v) <= step) v = 0.0f;
    else v -= signf(v) * step;
  } else {
    v += signf(target) * cfg::KEY_RAMP_UP * dt;
    if (fabsf(v) > fabsf(target)) v = target;
  }
}

// Render-side lookup tables (built once, values identical to the per-pixel / per-call math).
fm::GlowStamp g_beaconGlow;     // == drawGlow(.., 1.0f, 2.5f, .., pal::BEACON, 4)
uint8_t g_isCloudPx[256];       // pal::isWorld(c) && pal::worldBase(c) >= pal::B_CLOUD_TOP
bool g_renderTables = false;
void initRenderTables() {
  if (g_renderTables) return;
  g_beaconGlow.build(1.0f, 2.5f, 4);
  for (int c = 0; c < 256; ++c)
    g_isCloudPx[c] = (pal::isWorld((uint8_t)c) && pal::worldBase((uint8_t)c) >= pal::B_CLOUD_TOP) ? 1 : 0;
  g_renderTables = true;
}

inline bool inFlightState(GameState s) {
  return s == GameState::Countdown || s == GameState::Play || s == GameState::Crash;
}

}  // namespace

// ============================================================================ logging
void Game::logf(const char* fmt, ...) const {
  if (!api_.log) return;
  char buf[200];
  va_list ap;
  va_start(ap, fmt);
  vsnprintf(buf, sizeof buf, fmt, ap);
  va_end(ap);
  api_.log(buf);
}

// ============================================================================ init
void Game::init(const PlatformApi& api, const Persist& p, uint32_t seed, bool imuPresent) {
  api_ = api;
  persist_ = p;
  if (persist_.version != PERSIST_VERSION) persist_ = defaultPersist();
  // sanitise everything that came from storage
  if (persist_.tod > 2) persist_.tod = 0;
  if (persist_.sens > 2) persist_.sens = (uint8_t)Sens::Med;
  if (persist_.assist > 2) persist_.assist = (uint8_t)Assist::Normal;
  if (persist_.view > 2) persist_.view = 0;
  if (persist_.bright < cfg::BRIGHT_MIN) persist_.bright = cfg::BRIGHT_MIN;
  if (persist_.rollSign != -1 && persist_.rollSign != 1) persist_.rollSign = cfg::ROLL_SIGN_DEFAULT;
  if (persist_.pitchSign != -1 && persist_.pitchSign != 1) persist_.pitchSign = cfg::PITCH_SIGN_DEFAULT;
  if (persist_.bestCombo > cfg::COMBO_MAX) persist_.bestCombo = cfg::COMBO_MAX;
  persist_.tiltOn = persist_.tiltOn ? 1 : 0;
  persist_.invert = persist_.invert ? 1 : 0;
  persist_.mute = persist_.mute ? 1 : 0;
  persist_.drift = persist_.drift ? 1 : 0;

  seed_ = seed ? seed : 1u;
  imuPresent_ = imuPresent;
  fb_.px = s_px;
  fb_.depth = s_depth;
  memset(s_px, pal::EMPTY, sizeof s_px);
  memset(s_depth, DEPTH_FAR, sizeof s_depth);

  prof_.init(api.micros);
  auto now = [this]() -> uint32_t { return api_.micros ? api_.micros() : 0u; };

  uint32_t t0 = now();
  pal_.init();
  pal_.setTod((Tod)persist_.tod);
  uint32_t t1 = now();
  world_.generate(seed_);
  worldRenderInit(seed_);
  skyInit(seed_);
  uint32_t t2 = now();
  bool jetOk = jet_.build();
  uint32_t t3 = now();
  if (!jetOk) logf("[ERR] jet_build");

  int islands = 0;
  if (world_.macro())
    for (int i = 0; i < cfg::MACRO_N * cfg::MACRO_N; ++i)
      if (world_.macro()[i] != cfg::MACRO_OCEAN) ++islands;
  logf("[INIT] palette=%.2f world=%.1f world_crc=%08X islands=%d beacons=%d jet=%.2f vox=%d stars=%d",
       (double)((float)(t1 - t0) * 0.001f), (double)((float)(t2 - t1) * 0.001f),
       (unsigned)world_.crc(), islands, world_.beaconCount(), (double)((float)(t3 - t2) * 0.001f),
       jet_.count(), cfg::STARS);

  tilt_.setNeutral(v3(persist_.nx, persist_.ny, persist_.nz));
  applyTiltConfig();
  synth_.init(seed_ ^ 0x9E3779B9u);
  synth_.setMute(persist_.mute != 0);
  fx_.reset();

  placeAttractJet();
  cam_.reset(flight_.s(), world_, persist_.view);
  prevPos_ = flight_.s().pos;

  hud_ = HudData{};
  hud_.combo = 1;
  for (int i = 0; i < MENU_ITEMS; ++i) hud_.menuLabels[i] = MENU_LABELS[i];
  state_ = GameState::Boot;
  stateT_ = 0.0f;
  clock_ = 0.0f;
  imuStream_ = JET_DEBUG != 0;
}

void Game::applyTiltConfig() {
  tilt_.configure((Sens)persist_.sens, persist_.invert != 0, persist_.drift != 0,
                  persist_.rollSign, persist_.pitchSign);
}

// ============================================================================ state changes
void Game::enterState(GameState s) {
  if (s != state_)
    logf("[GAME] state %s->%s", STATE_NAMES[(int)state_], STATE_NAMES[(int)s]);
  state_ = s;
  stateT_ = 0.0f;
  menuSel_ = 0;
}

void Game::placeAttractJet() {
  float x = 0.0f, y = 0.0f, yaw = 0.0f;
  world_.findSpawn(runCount_ * 7u + 3u, x, y, yaw);
  float z = cfg::ATTRACT_ALT;
  float g = world_.maxGroundAround(x, y, 40.0f);
  if (g + 20.0f > z) z = g + 20.0f;
  flight_.reset(v3(x, y, z), yaw);
  flight_.mut().invuln = 1.0f;
  attractT_ = 0.0f;
  assembleT_ = ASSEMBLE_S;
  simAcc_ = 0.0f;
  prevPos_ = flight_.s().pos;
}

void Game::startRun(Mode m, bool skipCountdown) {
  mode_ = m;
  score_ = 0;
  combo_ = 1;
  bestComboRun_ = 1;
  ringsRun_ = 0;
  crashes_ = 0;
  skimTouches_ = 0;
  newBest_ = false;
  timeLeft_ = m == Mode::RingRun ? cfg::TIMER_START : -1.0f;
  skimTick_ = 0.0f;
  timeScale_ = 1.0f;
  hitStop_ = 0.0f;
  slowmoT_ = 0.0f;
  simAcc_ = 0.0f;
  keyTurn_ = keyClimb_ = 0.0f;
  pendingRoll_ = 0;
  pendingLoop_ = false;
  helpOn_ = false;
  recal_ = false;
  hud_.centre[0] = 0;
  for (int i = 0; i < HUD_POPUPS; ++i) hud_.popups[i].text[0] = 0;
  ++runCount_;

  float x = 0.0f, y = 0.0f, yaw = 0.0f;
  world_.findSpawn(runCount_, x, y, yaw);
  float z = cfg::SPAWN_ALT;
  float g = world_.maxGroundAround(x, y, 30.0f);
  if (g + 15.0f > z) z = g + 15.0f;
  flight_.reset(v3(x, y, z), yaw);
  fx_.reset();
  course_.reset(flight_.s(), world_, hash32(seed_ ^ (runCount_ * 0x9E3779B1u)));
  cam_.setCrashOrbit(false, wreck_);
  cam_.reset(flight_.s(), world_, persist_.view);
  prevPos_ = flight_.s().pos;
  apRollT_ = 9.0f + 5.0f * (float)(hash32(seed_ + runCount_) & 0xFFFF) * (1.0f / 65535.0f);

  if (skipCountdown) {
    assembleT_ = ASSEMBLE_S;
    enterState(GameState::Play);
  } else {
    assembleT_ = 0.0f;
    countdownDigit_ = 3;
    enterState(GameState::Countdown);
    synth_.trigger(Sfx::Blip, 1.0f);
    // re-zero the tilt neutral if the player is already holding still
    if (imuPresent_ && persist_.tiltOn) {
      cdPrevN_ = tilt_.neutral();
      tilt_.beginCalibration(cfg::COUNTDOWN_S);
      bootCal_ = false;
      calSkipped_ = true;  // silent: a timeout here just keeps the current neutral
      countdownCal_ = true;  // pitch-only re-zero (see onCalibrationDone)
    }
  }
}

void Game::crash(CrashCause c) {
  const JetState& s = flight_.s();
  logf("[GAME] crash cause=%s alt=%.1f vz=%.1f pitch=%.2f pos=(%d,%d)",
       c == CrashCause::Sea ? "sea" : "terrain", (double)s.pos.z, (double)s.vel.z,
       (double)s.pitch, (int)s.pos.x, (int)s.pos.y);
  wreck_ = s.pos;
  if (c == CrashCause::Sea) wreck_.z = 0.0f;
  ++crashes_;
  combo_ = 1;
  flash_ = 1.0f;
  flashRgb_ = 0xFFF4E0u;
  shake_ = 1.0f;
  timeScale_ = cfg::CRASH_SLOWMO_SCALE;
  slowmoT_ = cfg::CRASH_SLOWMO_S;
  hitStop_ = 0.0f;
  fx_.burst(c == CrashCause::Sea ? Burst::CrashSea : Burst::Crash, s.pos, forwardOf(s.yaw), 1.0f);
  synth_.trigger(Sfx::Crash, 1.0f);
  cam_.setCrashOrbit(true, wreck_);
  if (mode_ == Mode::RingRun) {
    timeLeft_ -= cfg::TIMER_CRASH;
    if (timeLeft_ < 0.0f) timeLeft_ = 0.0f;
    addPopup("-5S", LW / 2, 40, pal::HUD_RED);
  }
  setCentre(c == CrashCause::Sea ? "SPLASH" : "CRASH");
  lastEv_ = FlightEvents{};
  enterState(GameState::Crash);
}

void Game::respawn() {
  const JetState& s = flight_.s();
  Ring r;
  Vec3 pos;
  float yaw = s.yaw;
  if (course_.current(r)) {
    Vec3 d = wrapDelta(r.c, wreck_);
    float hd = sqrtf(d.x * d.x + d.y * d.y);
    float dx, dy;
    if (hd < 1.0f) {
      Vec3 f = forwardOf(s.yaw);
      dx = f.x;
      dy = f.y;
    } else {
      dx = d.x / hd;
      dy = d.y / hd;
    }
    float back = hd + cfg::RESPAWN_BACK_U;
    if (back > 350.0f) back = 350.0f;
    pos = v3(wrapPos(r.c.x - dx * back), wrapPos(r.c.y - dy * back), cfg::RESPAWN_ALT);
    yaw = atan2f(dx, -dy);
  } else {
    Vec3 f = forwardOf(s.yaw);
    pos = v3(wrapPos(wreck_.x - f.x * cfg::RESPAWN_BACK_U), wrapPos(wreck_.y - f.y * cfg::RESPAWN_BACK_U),
             cfg::RESPAWN_ALT);
  }
  float g = world_.maxGroundAround(pos.x, pos.y, 24.0f);
  if (g + 18.0f > pos.z) pos.z = g + 18.0f;
  // The respawn line runs back over the wreck toward the ring, and the wreck is reached after
  // the invulnerability ends: clear every peak along the first 320 u of that line too.
  {
    const Vec3 f = forwardOf(yaw);
    float gp = 0.0f;
    for (int i = 1; i <= 13; ++i) {
      const float t = 24.0f * (float)i;
      const float qx = wrapPos(pos.x + f.x * t), qy = wrapPos(pos.y + f.y * t);
      gp = fmaxf(gp, world_.maxGroundAround(qx, qy, 12.0f));
    }
    if (gp + 15.0f > pos.z) pos.z = gp + 15.0f;
  }
  flight_.reset(pos, yaw);
  flight_.mut().invuln = cfg::INVULN_TIME;
  cam_.setCrashOrbit(false, wreck_);
  cam_.reset(flight_.s(), world_, persist_.view);
  assembleT_ = 0.0f;
  simAcc_ = 0.0f;
  pendingRoll_ = 0;
  pendingLoop_ = false;
  prevPos_ = flight_.s().pos;
  keyTurn_ = keyClimb_ = 0.0f;
  logf("[GAME] respawn pos=(%d,%d,%d) time=%.1f", (int)pos.x, (int)pos.y, (int)pos.z,
       (double)(timeLeft_ > 0.0f ? timeLeft_ : 0.0f));
  enterState(GameState::Play);
}

void Game::gameOver() {
  newBest_ = false;
  if (mode_ == Mode::RingRun && score_ > persist_.best) {
    persist_.best = score_;
    newBest_ = true;
  }
  if (bestComboRun_ > persist_.bestCombo && mode_ == Mode::Free) persist_.bestCombo = bestComboRun_;
  logf("[GAME] over score=%u best=%u rings=%u new=%d", (unsigned)score_, (unsigned)persist_.best,
       (unsigned)ringsRun_, newBest_ ? 1 : 0);
  hud_.lastScore = score_;
  hud_.lastRings = ringsRun_;
  hud_.newBest = newBest_;
  if (newBest_) {
    synth_.trigger(Sfx::Perfect, 1.335f);
    saveNow();  // game over with a new best: save right away
  } else {
    synth_.trigger(Sfx::Miss, 1.0f);
  }
  cam_.setCrashOrbit(false, wreck_);
  if (state_ == GameState::Crash) {
    placeAttractJet();
    cam_.reset(flight_.s(), world_, persist_.view);
  }
  timeScale_ = 1.0f;
  hud_.centre[0] = 0;
  enterState(GameState::GameOver);
}

// ============================================================================ persistence
void Game::requestSave() { saveTimer_ = cfg::PERSIST_DEBOUNCE_S; }

void Game::saveNow() {
  if (api_.save) api_.save(persist_);
  saveTimer_ = -1.0f;
}

// ============================================================================ hud helpers
void Game::addPopup(const char* text, int x, int y, uint8_t color) {
  int slot = -1;
  float oldest = -1.0f;
  for (int i = 0; i < HUD_POPUPS; ++i) {
    Popup& p = hud_.popups[i];
    if (!p.text[0] || p.age >= POPUP_LIFE) { slot = i; break; }
    if (p.age > oldest) { oldest = p.age; slot = i; }
  }
  if (slot < 0) slot = 0;
  Popup& p = hud_.popups[slot];
  snprintf(p.text, sizeof p.text, "%s", text);
  p.age = 0.0f;
  p.x = (int16_t)clampi(x, 14, LW - 14);
  p.y = (int16_t)clampi(y, 14, LH - 18);
  p.color = color;
}

void Game::setCentre(const char* text) {
  snprintf(hud_.centre, sizeof hud_.centre, "%s", text);
  hud_.centreAge = 0.0f;
}

// ============================================================================ tilt calibration
void Game::startRecal() {
  if (!imuPresent_) return;
  countdownCal_ = false;
  recalT_ = 0.0f;
  if (state_ == GameState::Title || state_ == GameState::GameOver) {
    tilt_.beginCalibration(cfg::TILT_CAL_TIMEOUT_S);
    bootCal_ = true;
    calSkipped_ = false;
    helpOn_ = false;
    enterState(GameState::Calibrate);
  } else if (state_ == GameState::Play || state_ == GameState::Pause) {
    tilt_.beginCalibration(cfg::TILT_RECAL_S + cfg::TILT_CAL_STILL_S);
    recal_ = true;
    bootCal_ = false;
    calSkipped_ = false;
  }
}

void Game::onCalibrationDone() {
  if (tilt_.calibTimedOut()) {
    if (!calSkipped_) logf("[CAL] timeout using_stored");
    if (bootCal_ && !calSkipped_) holdStillMsg_ = true;
  } else {
    if (countdownCal_) {
      // The silent countdown re-zero only adopts the new PITCH: a bank held while waiting
      // (e.g. pre-tilting toward the first ring) must not become a run-long turn bias, and
      // neutral drift only corrects pitch. Keep the previous roll component (device x).
      const Vec3 nNew = tilt_.neutral();
      Vec3 cand = v3(cdPrevN_.x, nNew.y, nNew.z);
      const float l = length(cand);
      tilt_.setNeutral(l > 0.5f ? cand : cdPrevN_);
    }
    Vec3 n = tilt_.neutral(), b = tilt_.gyroBias();
    persist_.nx = n.x;
    persist_.ny = n.y;
    persist_.nz = n.z;
    requestSave();
    logf("[CAL] ok n=(%.3f,%.3f,%.3f) bias=(%.1f,%.1f,%.1f) still_ms=%d", (double)n.x, (double)n.y,
         (double)n.z, (double)b.x, (double)b.y, (double)b.z,
         (int)(cfg::TILT_CAL_STILL_S * 1000.0f));
  }
  recal_ = false;
  countdownCal_ = false;
}

// ============================================================================ input
FlightInput Game::playerInput(float rdt) {
  FlightInput fi = {};
  const JetState& s = flight_.s();
  float kt = ((acts_ & A_RIGHT) ? 1.0f : 0.0f) - ((acts_ & A_LEFT) ? 1.0f : 0.0f);
  float kc = ((acts_ & A_UP) ? 1.0f : 0.0f) - ((acts_ & A_DOWN) ? 1.0f : 0.0f);
  if (persist_.invert) kc = -kc;
  rampKey(keyTurn_, kt, rdt);
  rampKey(keyClimb_, kc, rdt);
  float tt = 0.0f, tc = 0.0f;
  if (imuPresent_ && persist_.tiltOn) {
    TiltOut o = tilt_.out();
    if (o.valid) {
      tt = o.turn;
      tc = o.climb;
    }
  }
  float pt = clampf(tt + keyTurn_, -1.0f, 1.0f);
  float pc = clampf(tc + keyClimb_, -1.0f, 1.0f);
  float ta = 0.0f, ca = 0.0f;
  course_.assist(s, (Assist)persist_.assist, pt, pc, ta, ca);
  fi.turn = clampf(pt + ta, -1.0f, 1.0f);
  fi.climb = clampf(pc + ca, -1.0f, 1.0f);
  fi.boost = (acts_ & A_BOOST) != 0;
  if (actsPressed_ & A_ROLL_L) pendingRoll_ = -1;
  if (actsPressed_ & A_ROLL_R) pendingRoll_ = 1;
  if (actsPressed_ & A_ROLL_AUTO) {
    if (fabsf(fi.turn) > 0.15f) {
      pendingRoll_ = fi.turn > 0.0f ? (int8_t)1 : (int8_t)-1;
    } else {
      lastAutoRoll_ = (int8_t)-lastAutoRoll_;
      pendingRoll_ = lastAutoRoll_;
    }
  }
  if (actsPressed_ & A_LOOP) pendingLoop_ = true;
  return fi;
}

FlightInput Game::autopilotInput(float rdt, bool useRings) {
  FlightInput fi = {};
  const JetState& s = flight_.s();
  Ring r;
  bool have = useRings && course_.current(r);
  float targetYaw, targetZ, hd = 1000.0f;
  if (have) {
    Vec3 d = wrapDelta(r.c, s.pos);
    hd = sqrtf(d.x * d.x + d.y * d.y);
    targetYaw = atan2f(d.x, -d.y);
    targetZ = r.c.z;
  } else {
    // attract: toward the moon first, then a slow wander
    float wander = 0.0f;
    if (attractT_ > 8.0f) {
      float t = attractT_ - 8.0f;
      wander = 1.1f * sinf(0.05f * t) + 0.5f * sinf(0.13f * t + 1.0f);
    }
    targetYaw = cfg::LIGHT_AZ + wander;
    targetZ = cfg::ATTRACT_ALT;
  }
  float err = fm::wrapPi(targetYaw - s.yaw);
  fi.turn = clampf(1.6f * err, -1.0f, 1.0f);

  // terrain look-ahead along the nose (and a little to each side)
  const Vec3 f = forwardOf(s.yaw), rt = rightOf(s.yaw);
  float gAhead = 0.0f;
  for (int k = 1; k <= 5; ++k) {
    float dd = 25.0f * (float)k;
    for (int j = -1; j <= 1; ++j) {
      float x = fm::wrapPos(s.pos.x + f.x * dd + rt.x * 12.0f * (float)j);
      float y = fm::wrapPos(s.pos.y + f.y * dd + rt.y * 12.0f * (float)j);
      float g = fm::groundHeight(world_, x, y);
      if (g > gAhead) gAhead = g;
    }
  }
  const float minZ = gAhead + 14.0f;
  if (!have && targetZ < gAhead + 22.0f) targetZ = gAhead + 22.0f;
  float altErr = targetZ - s.pos.z;
  float pitchDes;
  if (have) pitchDes = atan2f(altErr, fmaxf(hd * 0.8f, 40.0f));
  else pitchDes = clampf(altErr * 0.02f, -0.3f, 0.4f);
  if (s.pos.z < minZ) pitchDes = fmaxf(pitchDes, 0.35f + 0.03f * (minZ - s.pos.z));
  if (flight_.predictedGap(world_, 1.0f) < 10.0f) pitchDes = fmaxf(pitchDes, 0.6f);
  if (s.pos.z < 30.0f) {  // never slam into the sea
    float lim = -asinf(clampf(9.0f / fmaxf(s.speed, 1.0f), 0.0f, 0.9f));
    pitchDes = fmaxf(pitchDes, lim);
  }
  pitchDes = clampf(pitchDes, -0.6f, 0.95f);
  pitchDes -= cfg::TURN_NOSE_UP * fabsf(s.yawRate / cfg::MAX_YAW);
  fi.climb = clampf(pitchDes >= 0.0f ? pitchDes / cfg::CLIMB_MAX : pitchDes / cfg::DIVE_MAX, -1.0f, 1.0f);
  if (fabsf(fi.climb) < 0.06f && fabsf(altErr) > 3.0f) fi.climb = altErr > 0.0f ? 0.06f : -0.06f;

  fi.boost = have && fabsf(err) < 0.12f && hd > 140.0f && s.boostEnergy > 0.3f && fabsf(altErr) < 50.0f;

  apRollT_ -= rdt;
  if (apRollT_ <= 0.0f && (!have || (hd > 160.0f && fabsf(err) < 0.35f)) && !s.rolling()) {
    lastAutoRoll_ = (int8_t)-lastAutoRoll_;
    pendingRoll_ = lastAutoRoll_;
    float u = (float)(hash32(seed_ ^ (frameNo_ * 0x85EBCA77u)) & 0xFFFF) * (1.0f / 65535.0f);
    apRollT_ = lerpf(cfg::ATTRACT_ROLL_MIN_S, cfg::ATTRACT_ROLL_MAX_S, u);
  }
  return fi;
}

CrashCause Game::runFlight(float sdt, FlightInput fi, FlightEvents& ev, Assist assist) {
  simAcc_ += sdt;
  int n = 0;
  while (simAcc_ >= cfg::SIM_STEP && n < cfg::SIM_MAX_SUBSTEPS) {
    simAcc_ -= cfg::SIM_STEP;
    ++n;
    fi.rollReq = pendingRoll_;
    fi.loopReq = pendingLoop_;
    pendingRoll_ = 0;
    pendingLoop_ = false;
    CrashCause c = flight_.step(fi, world_, assist, cfg::SIM_STEP, ev);
    if (c != CrashCause::None) {
      simAcc_ = 0.0f;
      return c;
    }
  }
  if (simAcc_ > cfg::SIM_STEP) simAcc_ = 0.0f;  // drop the backlog after a long hitch
  return CrashCause::None;
}

// ============================================================================ events and scoring
void Game::handleEvents(const FlightEvents& ev, bool scoring) {
  const JetState& s = flight_.s();
  if (ev.boostIgnite) {
    shake_ = fmaxf(shake_, 0.25f);
    JetAnchors an = jetAnchors(pose());
    fx_.burst(Burst::Ignite, an.nozzle, forwardOf(s.yaw) * -1.0f, 1.0f);
    synth_.trigger(Sfx::Ignite, 1.0f);
  }
  if (ev.skimTouch) {
    shake_ = fmaxf(shake_, 0.35f);
    fx_.burst(Burst::SkimTouch, s.pos, v3(0.0f, 0.0f, 1.0f), 1.0f);
    synth_.trigger(Sfx::Slap, 1.0f);
    ++skimTouches_;
  }
  if (ev.enteredCloud) {
    shake_ = fmaxf(shake_, 0.2f);
    fx_.burst(Burst::CloudEntry, s.pos, forwardOf(s.yaw), 1.0f);
    synth_.trigger(Sfx::Whoosh, 1.0f);
  }
  if (ev.rollStart) synth_.trigger(Sfx::Roll, 1.0f);
  if (!scoring) return;
  const uint32_t c = combo_;
  if (ev.closeCall) {
    score_ += (uint32_t)cfg::PTS_CLOSE * c;
    addPopup("CLOSE!", LW / 2, 34, pal::HUD_CYAN);
    shake_ = fmaxf(shake_, 0.12f);
  }
  if (ev.loopDone) {
    score_ += (uint32_t)cfg::PTS_LOOP * c;
    addPopup("LOOP!", LW / 2, 30, pal::HUD_GREEN);
    synth_.trigger(Sfx::Chime, 1.5f);
  }
}

void Game::handleRing(const RingEvent& e) {
  if (e.result == RingResult::None) return;
  const JetState& s = flight_.s();
  int px = LW / 2, py = 26;
  ScreenPt sp;
  if (project(cam_.view(), e.pos, sp, cfg::CULL_DEN)) {
    px = (int)sp.x;
    py = (int)sp.y - 8;
  }
  if (e.result == RingResult::Missed) {
    combo_ = 1;
    synth_.trigger(Sfx::Miss, 1.0f);
    addPopup("MISS", LW / 2, 26, pal::HUD_RED);
    logf("[GAME] ring n=%u kind=MISSED score=%u combo=1 time=%.1f alt=%.1f", (unsigned)e.index,
         (unsigned)score_, (double)(timeLeft_ > 0.0f ? timeLeft_ : 0.0f), (double)s.heightAboveSurface);
    return;
  }
  const bool perfect = e.result == RingResult::Perfect;
  const uint32_t c = combo_;
  uint32_t pts = (uint32_t)cfg::PTS_RING * c;
  int tags = 0;
  if (perfect) pts += (uint32_t)cfg::PTS_PERFECT * c;
  ++ringsRun_;

  char buf[12];
  snprintf(buf, sizeof buf, "+%u", (unsigned)pts);  // provisional; bonuses add below
  if (s.rolling()) {
    pts += (uint32_t)cfg::PTS_ROLL * c;
    addPopup("ROLL!", px, py + 7 + 6 * tags++, pal::HUD_GREEN);
  }
  // LOW rings sit at >= surface + R + 3 (course.cpp), so a centred pass through one is above
  // LOW_RING_ALT early on: award LOW! by ring kind, or for any genuinely low pass.
  if (e.kind == RingKind::Low || s.heightAboveSurface < cfg::LOW_RING_ALT) {
    pts += (uint32_t)cfg::PTS_LOW * c;
    addPopup("LOW!", px, py + 7 + 6 * tags++, pal::HUD_CYAN);
  }
  if (s.boost > 0.5f && tags < 3) {
    pts += (uint32_t)cfg::PTS_BOOST * c;
    addPopup("BOOST!", px, py + 7 + 6 * tags++, pal::HUD_AMBER);
  } else if (s.boost > 0.5f) {
    pts += (uint32_t)cfg::PTS_BOOST * c;
  }
  snprintf(buf, sizeof buf, "+%u", (unsigned)pts);
  addPopup(buf, px, py, pal::HUD_WHITE);
  score_ += pts;

  int nc = (int)combo_ + (perfect ? 2 : 1);
  combo_ = (uint8_t)(nc > cfg::COMBO_MAX ? cfg::COMBO_MAX : nc);
  if (combo_ > bestComboRun_) bestComboRun_ = combo_;
  hud_.comboPulse = 1.0f;

  if (mode_ == Mode::RingRun) {
    // The ring bonus shrinks with difficulty (spacing shrinks too), so late plain passes no
    // longer gain time at cruise: only PERFECT lines and boosting keep the timer up.
    const float d = sat((float)(e.index - 1) / (float)cfg::RING_DIFF_RINGS);
    timeLeft_ += cfg::TIMER_RING * lerpf(1.0f, 0.6f, d) + (perfect ? cfg::TIMER_PERFECT : 0.0f);
    if (timeLeft_ > cfg::TIMER_MAX) timeLeft_ = cfg::TIMER_MAX;
  }
  JetState& ms = flight_.mut();
  ms.boostEnergy = fminf(1.0f, ms.boostEnergy + cfg::BOOST_RING_BONUS);

  float R = lerpf(cfg::RING_R_START, cfg::RING_R_END,
                  sat((float)(e.index - 1) / (float)cfg::RING_DIFF_RINGS));
  shake_ = fmaxf(shake_, 0.15f);
  float pitch = powf(2.0f, (float)c / 12.0f);
  if (perfect) {
    hitStop_ = cfg::HITSTOP_PERFECT_S;
    flash_ = fmaxf(flash_, 0.35f);
    flashRgb_ = 0xE8F4FFu;
    synth_.trigger(Sfx::Perfect, pitch);
    fx_.burst(Burst::Perfect, e.pos, e.normal, R);
    setCentre("PERFECT");
  } else {
    synth_.trigger(Sfx::Chime, pitch);
    fx_.burst(Burst::RingPass, e.pos, e.normal, R);
  }
  logf("[GAME] ring n=%u kind=%s score=%u combo=%d time=%.1f alt=%.1f", (unsigned)e.index,
       KIND_NAMES[perfect ? 1 : 0], (unsigned)score_, (int)combo_,
       (double)(timeLeft_ > 0.0f ? timeLeft_ : 0.0f), (double)s.heightAboveSurface);
}

void Game::updateMist(float rdt, bool inCloud) {
  float target = 0.0f;
  if (inCloud) {
    float mid = 0.5f * (cfg::MIST_MIN + cfg::MIST_MAX);
    float flick = 0.08f * sinf(clock_ * 19.0f) * sinf(clock_ * 7.3f + 1.0f);
    target = clampf(mid + flick, cfg::MIST_MIN, cfg::MIST_MAX);
  }
  if (mist_ != target) easeTo(mist_, target, 6.0f, rdt);
  if (mist_ < 0.004f && !inCloud) mist_ = 0.0f;
}

// ============================================================================ per-state updates
void Game::updateAttract(float rdt) {
  attractT_ += rdt;
  if (attractT_ > 2056.0f) attractT_ -= 2048.0f;  // precision (wander phase jump is eased)
  flight_.mut().invuln = 1.0f;  // the attract jet never crashes
  FlightInput fi = autopilotInput(rdt, false);
  FlightEvents ev = {};
  runFlight(rdt, fi, ev, Assist::Normal);
  lastEv_ = ev;
  handleEvents(ev, false);
  fx_.update(flight_.s(), pose(), world_, rdt, ev.inCloud);
  updateMist(rdt, ev.inCloud);
}

void Game::updatePlay(float rdt) {
  if (actsPressed_ & A_PAUSE) {
    helpOn_ = false;
    pausedFromPlay_ = true;
    synth_.trigger(Sfx::Blip, 1.0f);
    enterState(GameState::Pause);
    return;
  }
  if (helpOn_) {
    if (actsPressed_ & (A_HELP | A_BACK | A_CONFIRM)) helpOn_ = false;
    return;
  }
  if (recal_) {  // frozen while the 'c' recalibration window runs; ENTER/BKSP end it
    if ((actsPressed_ & (A_CONFIRM | A_BACK)) && tilt_.calibrating()) {
      calSkipped_ = true;
      tilt_.beginCalibration(0.0f);  // ends at the next sample, keeps the current neutral
    }
    return;
  }

  FlightInput fi = autopilot_ ? autopilotInput(rdt, true) : playerInput(rdt);
  stickTurn_ = fi.turn;
  stickClimb_ = fi.climb;

  // time scale: perfect hit-stop and the tail of the crash slow motion
  float scale = 1.0f;
  if (hitStop_ > 0.0f) {
    hitStop_ -= rdt;
    scale = cfg::HITSTOP_PERFECT_SCALE;
  }
  if (timeScale_ != 1.0f) easeTo(timeScale_, 1.0f, cfg::TIMESCALE_EASE, rdt);
  const float sdt = rdt * scale * timeScale_;
  assembleT_ += rdt;

  const Vec3 startPos = flight_.s().pos;
  FlightEvents ev = {};
  CrashCause cc = runFlight(sdt, fi, ev, (Assist)persist_.assist);
  lastEv_ = ev;
  handleEvents(ev, true);
  if (cc != CrashCause::None) {
    crash(cc);
    return;
  }
  const JetState& s = flight_.s();

  // rings
  RingEvent re = course_.update(startPos, s, world_, sdt);
  handleRing(re);
  prevPos_ = s.pos;

  // GPWS beep every 0.16 s
  if (ev.pullUp) {
    pullUpBeep_ -= rdt;
    if (pullUpBeep_ <= 0.0f) {
      synth_.trigger(Sfx::PullUp, 1.0f);
      pullUpBeep_ = 0.16f;
    }
  } else {
    pullUpBeep_ = 0.0f;
  }

  // skim scoring
  if (s.overWater && s.heightAboveSurface < cfg::SKIM_BAND) {
    skimTick_ += sdt;
    while (skimTick_ >= cfg::SKIM_TICK_S) {
      skimTick_ -= cfg::SKIM_TICK_S;
      score_ += (uint32_t)cfg::PTS_SKIM_TICK * combo_;
    }
  } else {
    skimTick_ = 0.0f;
  }

  // Ring Run timer
  if (mode_ == Mode::RingRun) {
    float before = timeLeft_;
    timeLeft_ -= sdt;
    if (timeLeft_ < 5.0f && timeLeft_ > 0.0f && -fm::floorF(-before) != -fm::floorF(-timeLeft_))
      synth_.trigger(Sfx::Tick, 1.0f);
    if (timeLeft_ <= 0.0f) {
      timeLeft_ = 0.0f;
      fx_.update(s, pose(), world_, sdt, ev.inCloud);
      gameOver();
      return;
    }
  }

  fx_.update(s, pose(), world_, sdt, ev.inCloud);
  updateMist(rdt, ev.inCloud);
}

void Game::buildMenu() {
  for (int i = 0; i < MENU_ITEMS; ++i) {
    hud_.menuLabels[i] = MENU_LABELS[i];
    menuVal_[i][0] = 0;
  }
  snprintf(menuVal_[1], sizeof menuVal_[1], "%s", !imuPresent_ ? "NO IMU" : (persist_.tiltOn ? "ON" : "OFF"));
  snprintf(menuVal_[2], sizeof menuVal_[2], "%s", SENS_NAMES[persist_.sens % 3]);
  snprintf(menuVal_[3], sizeof menuVal_[3], "%s", persist_.invert ? "ON" : "OFF");
  snprintf(menuVal_[4], sizeof menuVal_[4], "%s", ASSIST_NAMES[persist_.assist % 3]);
  snprintf(menuVal_[5], sizeof menuVal_[5], "%s", TOD_NAMES[persist_.tod % 3]);
  if (persist_.mute) snprintf(menuVal_[6], sizeof menuVal_[6], "MUTE");
  else snprintf(menuVal_[6], sizeof menuVal_[6], "%d", (int)persist_.volume);
  snprintf(menuVal_[7], sizeof menuVal_[7], "%d", (int)persist_.bright);
  for (int i = 0; i < MENU_ITEMS; ++i) hud_.menuValues[i] = menuVal_[i];
}

void Game::menuAdjust(int item, int dir) {
  switch (item) {
    case 1: persist_.tiltOn = persist_.tiltOn ? 0 : 1; break;
    case 2: persist_.sens = (uint8_t)((persist_.sens + 3 + dir) % 3); break;
    case 3: persist_.invert = persist_.invert ? 0 : 1; break;
    case 4: persist_.assist = (uint8_t)((persist_.assist + 3 + dir) % 3); break;
    case 5: setTod((Tod)((persist_.tod + 3 + dir) % 3)); break;
    case 6: {
      int v = (int)persist_.volume + dir * (int)cfg::VOLUME_STEP;
      persist_.volume = (uint8_t)clampi(v, 0, 255);
      persist_.mute = 0;
      break;
    }
    case 7: {
      int v = (int)persist_.bright + dir * (int)cfg::BRIGHT_STEP;
      persist_.bright = (uint8_t)clampi(v, cfg::BRIGHT_MIN, 255);
      break;
    }
    default: return;
  }
  applyTiltConfig();
  synth_.setMute(persist_.mute != 0);
  requestSave();
  synth_.trigger(Sfx::Blip, dir > 0 ? 1.2f : 0.9f);
}

void Game::updatePause() {
  if (recal_) {  // calibration overlay; resumes in the pause menu when done
    if ((actsPressed_ & (A_CONFIRM | A_BACK)) && tilt_.calibrating()) {
      calSkipped_ = true;
      tilt_.beginCalibration(0.0f);
    }
    return;
  }
  if (helpOn_) {
    if (actsPressed_ & (A_HELP | A_BACK | A_CONFIRM | A_PAUSE)) helpOn_ = false;
    return;
  }
  if (actsPressed_ & A_PAUSE) {
    synth_.trigger(Sfx::Blip, 1.2f);
    enterState(GameState::Play);
    return;
  }
  bool quit = (actsPressed_ & A_BACK) != 0;
  if (actsPressed_ & A_UP) {
    menuSel_ = (int8_t)((menuSel_ + MENU_ITEMS - 1) % MENU_ITEMS);
    synth_.trigger(Sfx::Blip, 1.0f);
  }
  if (actsPressed_ & A_DOWN) {
    menuSel_ = (int8_t)((menuSel_ + 1) % MENU_ITEMS);
    synth_.trigger(Sfx::Blip, 1.0f);
  }
  if (actsPressed_ & A_LEFT) menuAdjust(menuSel_, -1);
  if (actsPressed_ & A_RIGHT) menuAdjust(menuSel_, 1);
  if (actsPressed_ & A_CONFIRM) {
    if (menuSel_ == 0) {
      synth_.trigger(Sfx::Blip, 1.2f);
      int8_t keep = menuSel_;
      enterState(GameState::Play);
      menuSel_ = keep;
      return;
    }
    if (menuSel_ == MENU_ITEMS - 1) quit = true;
    else menuAdjust(menuSel_, 1);
  }
  if (quit) {
    if (mode_ == Mode::Free && bestComboRun_ > persist_.bestCombo) {
      persist_.bestCombo = bestComboRun_;
      requestSave();
    }
    if (saveTimer_ >= 0.0f) saveNow();
    logf("[GAME] quit score=%u rings=%u", (unsigned)score_, (unsigned)ringsRun_);
    placeAttractJet();
    cam_.setCrashOrbit(false, wreck_);
    cam_.reset(flight_.s(), world_, persist_.view);
    fx_.reset();
    hud_.centre[0] = 0;
    enterState(GameState::Title);
  }
}

void Game::handleGlobalKeys() {
  const uint32_t a = actsPressed_;
  if (a & A_TOD) setTod((Tod)((persist_.tod + 1) % 3));
  if (a & A_VIEW) {
    persist_.view = (uint8_t)((persist_.view + 1) % 3);
    requestSave();
  }
  if (a & A_FPS) statsOn_ = !statsOn_;
  if (a & A_IMUDBG) imuStream_ = !imuStream_;
  if (a & A_MUTE) {
    persist_.mute = persist_.mute ? 0 : 1;
    synth_.setMute(persist_.mute != 0);
    requestSave();
  }
  if (a & (A_VOL_DN | A_VOL_UP)) {
    int v = (int)persist_.volume + ((a & A_VOL_UP) ? 1 : -1) * (int)cfg::VOLUME_STEP;
    persist_.volume = (uint8_t)clampi(v, 0, 255);
    persist_.mute = 0;
    synth_.setMute(false);
    synth_.trigger(Sfx::Blip, 1.0f);
    requestSave();
  }
  if (a & (A_BRI_DN | A_BRI_UP)) {
    int v = (int)persist_.bright + ((a & A_BRI_UP) ? 1 : -1) * (int)cfg::BRIGHT_STEP;
    persist_.bright = (uint8_t)clampi(v, cfg::BRIGHT_MIN, 255);
    requestSave();
  }
  if (a & A_TILT) {
    persist_.tiltOn = persist_.tiltOn ? 0 : 1;
    setCentre(!imuPresent_ ? "NO IMU" : (persist_.tiltOn ? "TILT ON" : "KEYS ONLY"));
    requestSave();
  }
  if (a & A_INVERT) {
    persist_.invert = persist_.invert ? 0 : 1;
    applyTiltConfig();
    setCentre(persist_.invert ? "INVERT ON" : "INVERT OFF");
    requestSave();
  }
  if (a & A_RECAL) startRecal();
  if ((a & A_HELP) && !helpOn_ && (state_ == GameState::Title || state_ == GameState::Play)) {
    helpOn_ = true;
    actsPressed_ &= ~(uint32_t)A_HELP;  // do not close it again this frame
  }
}

// ============================================================================ update
void Game::update(const InputFrame& in) {
  prof_.begin(ST_SIM);
  const float rdt = clampf(in.dt, 0.0f, cfg::DT_MAX);
  lastDt_ = rdt;
  clock_ += rdt;
  // Keep the float animation clock small (precision): 2048 s keeps the cloud drift continuous
  // (6 and 2.5 u/s * 2048 s are whole multiples of the 1024 u cloud period).
  if (clock_ >= 4096.0f) clock_ -= 2048.0f;
  stateT_ += rdt;
  if (recal_) recalT_ += rdt;
  batteryPct_ = in.batteryPct;

  acts_ = keysToActs(in.keysDown) | (in.btnA ? (uint32_t)A_PAUSE : 0u);
  actsPressed_ = keysToActs(in.keysPressed) | (in.btnAPressed ? (uint32_t)A_PAUSE : 0u);
  prevKeys_ = in.keysDown;
  if (in.serialCmd) command(in.serialCmd);

  // ---- IMU / tilt
  if (imuPresent_) {
    tilt_.update(in.imu, rdt);
    if (in.imu.fresh) {
      imuStaleT_ = 0.0f;
      staleLogged_ = false;
    } else {
      imuStaleT_ += rdt;
      if (imuStaleT_ > cfg::TILT_STALE_S && !staleLogged_) {
        logf("[ERR] imu_stale ms=%d", (int)(imuStaleT_ * 1000.0f));
        staleLogged_ = true;
      }
    }
    bool cal = tilt_.calibrating();
    if (calWas_ && !cal) onCalibrationDone();
    calWas_ = cal;
  }
  world_.setTime(clock_);

  // ---- decays (real time)
  // (the decays and eases below skip their expf when there is nothing to do; same results)
  if (shake_ != 0.0f) shake_ *= expf(-cfg::SHAKE_DECAY * rdt);
  if (flash_ != 0.0f) flash_ *= expf(-FLASH_DECAY * rdt);
  if (flash_ < 0.004f) flash_ = 0.0f;
  {
    // HUD-exempt palette dim behind menus / screens so their text stays legible
    float fadeT = 0.0f;
    if (state_ == GameState::Pause || helpOn_) fadeT = 0.55f;
    else if (state_ == GameState::GameOver) fadeT = 0.35f;
    else if (state_ == GameState::Calibrate || recal_) fadeT = 0.3f;
    if (fade_ != fadeT) easeTo(fade_, fadeT, 3.0f, rdt);
    if (fade_ < 0.002f) fade_ = 0.0f;
  }
  hud_.comboPulse = fmaxf(0.0f, hud_.comboPulse - 3.0f * rdt);
  for (int i = 0; i < HUD_POPUPS; ++i)
    if (hud_.popups[i].text[0]) {
      hud_.popups[i].age += rdt;
      if (hud_.popups[i].age > POPUP_LIFE) hud_.popups[i].text[0] = 0;
    }
  if (hud_.centre[0]) {
    hud_.centreAge += rdt;
    if (hud_.centreAge > (state_ == GameState::Title ? 2.0f : CENTRE_LIFE)) hud_.centre[0] = 0;
  }

  if (state_ != GameState::Boot) handleGlobalKeys();

  switch (state_) {
    case GameState::Boot: {
      fade_ = 1.0f;
      if (imuPresent_ && persist_.tiltOn) {
        tilt_.beginCalibration(cfg::TILT_CAL_TIMEOUT_S);
        countdownCal_ = false;
        calWas_ = true;
        bootCal_ = true;
        calSkipped_ = false;
        enterState(GameState::Calibrate);
      } else {
        enterState(GameState::Title);
      }
      // JET_BOOT_BENCH: main.cpp injects the serial 'b' on the first frames (one start only).
      break;
    }
    case GameState::Calibrate: {
      updateAttract(rdt);
      if ((actsPressed_ & (A_CONFIRM | A_BACK)) && tilt_.calibrating()) {
        calSkipped_ = true;
        tilt_.beginCalibration(0.0f);  // ends at the next sample, keeps the stored neutral
      }
      if (!tilt_.calibrating() && stateT_ > 0.05f) {
        if (holdStillMsg_) {
          setCentre("HOLD STILL");
          holdStillMsg_ = false;
        }
        enterState(GameState::Title);
      }
      if (!imuPresent_) enterState(GameState::Title);
      break;
    }
    case GameState::Title: {
      updateAttract(rdt);
      if (helpOn_) {
        if (actsPressed_ & (A_HELP | A_BACK | A_CONFIRM)) helpOn_ = false;
      } else if (actsPressed_ & (A_CONFIRM | A_MODE1)) {
        startRun(Mode::RingRun, false);
      } else if (actsPressed_ & A_MODE2) {
        startRun(Mode::Free, false);
      }
      break;
    }
    case GameState::Countdown: {
      assembleT_ += rdt;
      int digit = 3 - (int)(stateT_ / (cfg::COUNTDOWN_S / 3.0f));
      if (digit < 1) digit = 1;
      if (digit != countdownDigit_) {
        countdownDigit_ = (int8_t)digit;
        synth_.trigger(Sfx::Blip, 1.0f);
      }
      fx_.update(flight_.s(), pose(), world_, rdt, false);
      if (stateT_ >= cfg::COUNTDOWN_S) {
        countdownDigit_ = 0;
        synth_.trigger(Sfx::Go, 1.0f);
        setCentre("GO!");
        flight_.mut().invuln = 0.0f;
        simAcc_ = 0.0f;
        enterState(GameState::Play);
      }
      break;
    }
    case GameState::Play:
      updatePlay(rdt);
      break;
    case GameState::Pause:
      updatePause();
      break;
    case GameState::Crash: {
      float scale = 1.0f;
      if (slowmoT_ > 0.0f) {
        slowmoT_ -= rdt;
        scale = cfg::CRASH_SLOWMO_SCALE;
        timeScale_ = cfg::CRASH_SLOWMO_SCALE;
      } else {
        easeTo(timeScale_, 1.0f, cfg::TIMESCALE_EASE, rdt);
        scale = timeScale_;
      }
      fx_.update(flight_.s(), pose(), world_, rdt * scale, false);
      updateMist(rdt, false);
      if (stateT_ >= cfg::CRASH_SEQ_S) {
        if (mode_ == Mode::RingRun && timeLeft_ <= 0.0f) gameOver();
        else respawn();
      }
      break;
    }
    case GameState::GameOver: {
      updateAttract(rdt);
      if (stateT_ > 0.8f && (actsPressed_ & A_CONFIRM)) {
        hud_.newBest = false;
        startRun(mode_, false);  // ENTER RETRY: straight into the countdown
      } else if (stateT_ > 0.8f && (actsPressed_ & (A_BACK | A_PAUSE))) {
        hud_.newBest = false;
        enterState(GameState::Title);  // BKSP MENU
      }
      break;
    }
  }

  // ---- camera (real dt; shake owned here)
  cam_.update(flight_.s(), world_, rdt, persist_.view, shake_);

  // ---- audio parameters
  {
    const JetState& s = flight_.s();
    AudioParams ap;
    ap.speedRatio = s.speed / cfg::V_CRUISE;
    ap.boost = s.boost;
    float dive = -sinf(s.pitch);
    ap.wind = fminf(1.6f, ap.speedRatio * (1.0f + 0.6f * fmaxf(0.0f, dive)));
    ap.spray = fx_.wakeStrength();
    ap.inCloud = lastEv_.inCloud && state_ != GameState::Crash;
    // Menus (calibrate / title / game over) stay quiet: the engine only runs while flying.
    ap.engineOn = state_ == GameState::Countdown || state_ == GameState::Play;
    if (!ap.engineOn) ap.spray = 0.0f;   // the attract flyover's wake must not hiss either
    if (state_ == GameState::Countdown) ap.boost = 0.0f;
    synth_.setParams(ap);
    synth_.setMute(persist_.mute != 0);
  }

  // ---- debounced settings save (never mid-flight)
  if (saveTimer_ >= 0.0f) {
    saveTimer_ -= rdt;
    if (saveTimer_ <= 0.0f) {
      saveTimer_ = 0.0f;
      if (!inFlightState(state_)) saveNow();
    }
  }

  benchTick(rdt);
  prof_.end(ST_SIM);
}

// ============================================================================ bench
// 20 s autopilot Ring Run. The device platform (main.cpp) prints the authoritative [BENCH]
// summary with per-stage timings (and the "[BENCH] start" line); the core only flies the run
// and logs a short frame-rate line under its own tag, BENCH_LOG_DELAY_S later.
void Game::startBench() {
  // a restart while a run is in progress keeps the autopilot state from before the first start
  if (!(benchT_ >= 0.0f && benchT_ < BENCH_S)) benchPrevAuto_ = autopilot_;
  benchT_ = 0.0f;
  benchFrames_ = benchWinFrames_ = 0;
  benchWinT_ = 0.0f;
  benchFpsMin_ = 1e9f;
  benchAvg_ = 0.0f;
  autopilot_ = true;
  if (!inFlightState(state_)) startRun(Mode::RingRun, true);
}

void Game::benchTick(float rdt) {
  if (benchT_ < 0.0f) return;
  const bool running = benchT_ < BENCH_S;
  benchT_ += rdt;
  if (running) {
    ++benchFrames_;
    ++benchWinFrames_;
    benchWinT_ += rdt;
    if (benchWinT_ >= 1.0f) {
      const float fps = (float)benchWinFrames_ / benchWinT_;
      if (fps < benchFpsMin_) benchFpsMin_ = fps;
      benchWinT_ = 0.0f;
      benchWinFrames_ = 0;
    }
    if (benchT_ >= BENCH_S) {  // run over: hand the controls back, log a bit later
      benchAvg_ = (float)benchFrames_ / benchT_;
      autopilot_ = benchPrevAuto_;
    }
    return;
  }
  if (benchT_ >= BENCH_S + BENCH_LOG_DELAY_S) {
    logf("[BENCH] core frames=%u fps_min=%.1f fps_avg=%.1f", (unsigned)benchFrames_,
         (double)(benchFpsMin_ > 1e8f ? benchAvg_ : benchFpsMin_), (double)benchAvg_);
    benchT_ = -1.0f;
  }
}

// ============================================================================ render
JetPose Game::pose() const {
  const JetState& s = flight_.s();
  JetPose p;
  p.pos = s.pos;
  p.yaw = s.yaw;
  p.roll = s.totalRoll();
  p.pitch = s.pitch;
  p.boost = s.boost;
  p.yawRateN = clampf(s.yawRate / cfg::MAX_YAW, -1.0f, 1.0f);
  p.climbRateN = clampf(s.pitchRate / cfg::CLIMB_RATE_UP, -1.0f, 1.0f);
  p.time = clock_;
  p.assemble = sat(assembleT_ / ASSEMBLE_S);
  p.visible = state_ != GameState::Crash && state_ != GameState::Boot;
  if (p.visible && state_ == GameState::Play && s.invuln > 0.0f && assembleT_ >= ASSEMBLE_S)
    p.visible = (((int)(clock_ * 16.0f)) & 1) == 0;  // 8 Hz blink
  return p;
}

void Game::buildHud() {
  HudData& h = hud_;
  const JetState& s = flight_.s();
  h.time = clock_;
  h.showPlay = (state_ == GameState::Countdown || state_ == GameState::Play ||
                state_ == GameState::Crash) && !helpOn_;
  h.score = score_;
  h.combo = combo_;
  h.timeLeft = mode_ == Mode::RingRun ? timeLeft_ : -1.0f;
  h.ring = ringsRun_;
  h.speedKt = s.speed * 2.0f;
  h.alt = s.heightAboveSurface;
  h.boostEnergy = s.boostEnergy;
  h.skimming = state_ == GameState::Play && s.overWater && s.heightAboveSurface < cfg::SKIM_BAND;
  h.pullUp = state_ == GameState::Play && lastEv_.pullUp;
  TiltOut to = tilt_.out();
  if (!imuPresent_) h.tilt = TiltBadge::NoImu;
  else if (!persist_.tiltOn) h.tilt = TiltBadge::Keys;
  else if (!to.valid) h.tilt = TiltBadge::Guard;
  else if (!tilt_.calibrated()) h.tilt = TiltBadge::NeedsCal;
  else h.tilt = TiltBadge::TiltOk;
  h.batteryPct = batteryPct_;
  h.guide = false;
  if (state_ == GameState::Play) {
    int gx = 0, gy = 0, gv = 0;
    if (course_.guide(cam_.view(), gx, gy, gv)) {
      h.guide = true;
      h.guideX = (int16_t)gx;
      h.guideY = (int16_t)gy;
      h.guideVertical = (int8_t)gv;
    }
  }
  h.stats = statsOn_;
  if (statsOn_) {  // only the 'f' overlay shows them (64-bit sums -> float: not free)
    h.fps = prof_.fps();
    h.coreMs = prof_.coreMsAvg();
  }
  h.imuDebug = imuStream_ && imuPresent_;
  const int si = persist_.sens % 3;
  h.dotX = clampf(to.rollDeg / cfg::TILT_ROLL_FULL[si], -1.0f, 1.0f);
  h.dotY = clampf(-to.pitchDeg / cfg::TILT_PITCH_FULL[si], -1.0f, 1.0f);
  h.calibProgress = tilt_.calibProgress();
  h.calibTimedOut =
      tilt_.calibrating() && (recal_ ? recalT_ : stateT_) > 1.5f && h.calibProgress < 0.2f;
  h.best = persist_.best;
  h.bestCombo = state_ == GameState::GameOver ? bestComboRun_ : persist_.bestCombo;
  h.newBest = newBest_ && state_ == GameState::GameOver;
  h.countdown = countdownDigit_;
  h.menuSel = menuSel_;
  if (state_ == GameState::Pause) buildMenu();

  switch (state_) {
    case GameState::Boot: h.screen = HudScreen::None; break;
    case GameState::Calibrate: h.screen = HudScreen::Calibrate; break;
    case GameState::Title: h.screen = helpOn_ ? HudScreen::Help : HudScreen::Title; break;
    case GameState::Countdown: h.screen = HudScreen::Countdown; break;
    case GameState::Play: h.screen = helpOn_ ? HudScreen::Help : HudScreen::None; break;
    case GameState::Pause: h.screen = helpOn_ ? HudScreen::Help : HudScreen::Pause; break;
    case GameState::Crash: h.screen = HudScreen::None; break;
    case GameState::GameOver: h.screen = HudScreen::GameOver; break;
  }
  if (recal_ && tilt_.calibrating()) h.screen = HudScreen::Calibrate;
}

void Game::render() {
  RenderStats& st = prof_.stats();
  st.clear();

  prof_.begin(ST_PAL);
  PalFx pfx;
  pfx.flash = flash_;
  pfx.flashRgb = flashRgb_;
  pfx.mist = 0.3f * sat(mist_ / cfg::MIST_MIN);
  pfx.fade = fade_;
  pal_.setFx(pfx);
  pal_.finalize();
  memset(fb_.px, pal::EMPTY, (size_t)LPIX);
  memset(fb_.depth, DEPTH_FAR, (size_t)LPIX);
  prof_.end(ST_PAL);

  if (state_ == GameState::Boot) {
    prof_.begin(ST_HUD);
    memset(fb_.px, pal::HUD_SHADOW, (size_t)LPIX);
    hudTextCentered(fb_, LW / 2, LH / 2 - 3, "GENERATING ISLANDS", pal::HUD_CYAN);
    prof_.end(ST_HUD);
    ++frameNo_;
    return;
  }

  const View& v = cam_.view();
  const Tod tod = pal_.tod();
  initRenderTables();

  prof_.begin(ST_WORLD);
  int mx = -1000, my = 0;
  bool moon = skyMoonScreen(v, tod, mx, my);
  SeaFx sea;
  sea.time = clock_;
  sea.moonX = moon ? mx : -1000;
  sea.moonPath = moon && tod != Tod::Day;
  sea.cloudShadows = tod != Tod::Night;
  renderWorld(fb_, v, world_, tod, sea, st);
  prof_.end(ST_WORLD);

  prof_.begin(ST_SKY);
  renderSky(fb_, v, tod, clock_, lastDt_, st);
  prof_.end(ST_SKY);

  prof_.begin(ST_OBJ);
  if (state_ == GameState::Countdown || state_ == GameState::Play || state_ == GameState::Pause ||
      state_ == GameState::Crash)
    course_.draw(fb_, v, tod, clock_, st);
  {
    Beacon bc[16];
    int n = world_.beaconsNear(v.fx, v.fy, 700.0f, bc, 16);
    for (int i = 0; i < n; ++i) {
      int ph = ((int)(clock_ * 160.0f) + bc[i].phase) & 255;
      if (ph > 170) continue;  // slow blink
      ScreenPt sp;
      if (!project(v, v3(bc[i].x, bc[i].y, bc[i].z + 1.0f), sp, cfg::CULL_DEN)) continue;
      int x = fm::floorI(sp.x), y = fm::floorI(sp.y);
      if (!inFrame(x, y)) continue;
      if (tod == Tod::Night) g_beaconGlow.draw(fb_, x, y, sp.dq, pal::BEACON);
      drawPoint(fb_, x, y, sp.dq, pal::BEACON, false);
    }
  }
  prof_.end(ST_OBJ);

  prof_.begin(ST_JET);
  JetPose jp = pose();
  if (jp.visible) {
    // Clouds never hide the player's jet: while it flies in the cloud slab the camera sits
    // outside it, so cloud surfaces lie between the eye and the jet. Push cloud pixels to the
    // far plane (mist is drawn over the jet afterwards, so it still reads as "inside").
    // Only around the jet: last frame's jet + flame box grown by a margin (the jet barely moves
    // on screen between frames); the whole frame when there is no recent box (first frame,
    // respawn). Outside the box clouds keep their depth, so trails and the shadow far from the
    // jet are still hidden by clouds in front of them.
    // Decide from the eye->jet segment, not the jet altitude alone: in a steep climb out of the
    // slab the chase camera trails below and cloud tops sit between the eye and the jet.
    const float segLo = fminf(v.eyeZ, jp.pos.z), segHi = fmaxf(v.eyeZ, jp.pos.z);
    if (segLo < cfg::CLOUD_TOP + 6.0f && segHi > cfg::CLOUD_BASE - 6.0f) {
      int x0 = 0, y0 = 0, x1 = LW, y1 = LH;
      if (jetBoundsFrame_ + 1 == frameNo_ && jetBounds_.x1 > jetBounds_.x0) {
        constexpr int M = 8;
        x0 = jetBounds_.x0 - M < 0 ? 0 : jetBounds_.x0 - M;
        y0 = jetBounds_.y0 - M < 0 ? 0 : jetBounds_.y0 - M;
        x1 = jetBounds_.x1 + M > LW ? LW : jetBounds_.x1 + M;
        y1 = jetBounds_.y1 + M > LH ? LH : jetBounds_.y1 + M;
      }
      for (int y = y0; y < y1; ++y) {
        const uint8_t* px = fb_.px + y * LW;
        uint8_t* dp = fb_.depth + y * LW;
        for (int x = x0; x < x1; ++x)
          if (g_isCloudPx[px[x]]) dp[x] = DEPTH_FAR;
      }
    }
    renderJetShadow(fb_, v, jet_, jp, world_, pal_, tod, &st);
    renderJet(fb_, v, jet_, jp, tod, st, &jetBounds_);
    jetBoundsFrame_ = frameNo_;
  } else {
    jetBounds_ = Rect{0, 0, 0, 0};
  }
  prof_.end(ST_JET);

  prof_.begin(ST_FX);
  fx_.draw(fb_, v, tod, clock_, st);
  if (mist_ > 0.01f) fx_.drawMist(fb_, mist_, frameNo_);
  prof_.end(ST_FX);

  prof_.begin(ST_HUD);
  buildHud();
  drawHud(fb_, hud_);
  prof_.end(ST_HUD);
  ++frameNo_;
}

// ============================================================================ telemetry, commands
Telemetry Game::telemetry() const {
  const JetState& s = flight_.s();
  TiltOut to = tilt_.out();
  Telemetry t;
  t.state = state_;
  t.mode = mode_;
  t.score = score_;
  t.combo = combo_;
  t.timeLeft = mode_ == Mode::RingRun ? timeLeft_ : -1.0f;
  t.rings = ringsRun_;
  t.pos = s.pos;
  t.yaw = s.yaw;
  t.speed = s.speed;
  t.agl = s.heightAboveSurface;
  t.boost = s.boost;
  t.boostEnergy = s.boostEnergy;
  t.tiltRoll = to.rollDeg;
  t.tiltPitch = to.pitchDeg;
  t.turn = stickTurn_;
  t.climb = stickClimb_;
  t.tiltOn = persist_.tiltOn != 0;
  t.tiltValid = to.valid;
  t.calibrated = tilt_.calibrated();
  t.up = tilt_.up();
  t.neutral = tilt_.neutral();
  t.tod = pal_.tod();
  return t;
}

bool Game::command(char c) {
  switch (c) {
    case 'r':
      persist_.rollSign = (int8_t)-persist_.rollSign;
      applyTiltConfig();
      requestSave();
      logf("[GAME] roll_sign=%d", (int)persist_.rollSign);
      return true;
    case 'p':
      persist_.pitchSign = (int8_t)-persist_.pitchSign;
      applyTiltConfig();
      requestSave();
      logf("[GAME] pitch_sign=%d", (int)persist_.pitchSign);
      return true;
    case 'c':
      startRecal();
      return true;
    case 't':
      persist_.tiltOn = persist_.tiltOn ? 0 : 1;
      requestSave();
      logf("[GAME] tilt=%d", (int)persist_.tiltOn);
      return true;
    case 'n':
      setTod((Tod)((persist_.tod + 1) % 3));
      return true;
    case 'a':
      setAutopilot(!autopilot_);
      return true;
    case 'b':
      startBench();
      return true;
    case 'x':
      if (state_ == GameState::Play) crash(CrashCause::Terrain);
      else if (state_ == GameState::Crash) respawn();
      return true;
    case 'X':
      persist_.best = 0;
      persist_.bestCombo = 0;
      saveNow();
      logf("[GAME] best reset");
      return true;
    case 'f':
      statsOn_ = !statsOn_;
      return true;
    case 'i':
      imuStream_ = !imuStream_;
      return true;
    case 'm':
      if (api_.log) jet_.dumpAscii(api_.log);
      return true;
    default:
      return false;
  }
}

// ============================================================================ simulator hooks
void Game::debugStart(Mode m) {
  helpOn_ = false;
  recal_ = false;
  startRun(m, true);
}

void Game::debugTeleport(float x, float y, float z, float yawRad) {
  float boostE = flight_.s().boostEnergy;
  flight_.reset(v3(x, y, z), yawRad);
  flight_.mut().boostEnergy = boostE;
  simAcc_ = 0.0f;
  prevPos_ = flight_.s().pos;
  assembleT_ = ASSEMBLE_S;
  cam_.reset(flight_.s(), world_, persist_.view);
}

void Game::setTod(Tod t) {
  int ti = (int)t;
  if (ti < 0 || ti > 2) ti = 0;
  persist_.tod = (uint8_t)ti;
  pal_.setTod((Tod)ti);
  flash_ = fmaxf(flash_, 0.5f);
  flashRgb_ = 0xFFF4E0u;
  requestSave();
}

void Game::setAutopilot(bool on) {
  autopilot_ = on;
  logf("[GAME] autopilot=%d", on ? 1 : 0);
}

void Game::setView(int preset) {
  persist_.view = (uint8_t)clampi(preset, 0, 2);
}

bool Game::debugVar(const char* name, float& out) const {
  if (!name) return false;
  const JetState& s = flight_.s();
  const TiltOut to = tilt_.out();
  struct Entry { const char* n; float v; };
  Ring r = {};
  bool haveRing = course_.current(r);
  float ringDist = haveRing ? length(wrapDelta(r.c, s.pos)) : -1.0f;
  const Entry table[] = {
    {"alt", s.pos.z},
    {"agl", s.heightAboveSurface},
    {"speed", s.speed},
    {"score", (float)score_},
    {"combo", (float)combo_},
    {"crashed", (float)crashes_},
    {"crashes", (float)crashes_},
    {"rings", (float)ringsRun_},
    {"horizon", cam_.view().horizon},
    {"state", (float)(int)state_},
    {"boost", s.boost},
    {"boost_energy", s.boostEnergy},
    {"time_left", mode_ == Mode::RingRun ? timeLeft_ : -1.0f},
    {"jet_w", (float)(jetBounds_.x1 - jetBounds_.x0)},
    {"jet_h", (float)(jetBounds_.y1 - jetBounds_.y0)},
    {"yaw", s.yaw},
    {"yaw_rate", s.yawRate},
    {"bank", s.bank},
    {"pitch", s.pitch},
    {"vz", s.vel.z},
    {"energy", s.energy},
    {"g", s.g},
    {"x", s.pos.x},
    {"y", s.pos.y},
    {"invuln", s.invuln},
    {"rolling", s.rolling() ? 1.0f : 0.0f},
    {"over_water", s.overWater ? 1.0f : 0.0f},
    {"skim_touches", (float)skimTouches_},
    {"in_cloud", lastEv_.inCloud ? 1.0f : 0.0f},
    {"pull_up", lastEv_.pullUp ? 1.0f : 0.0f},
    {"turn", to.turn},             // tilt controller output (before keys / assist)
    {"climb", to.climb},
    {"tilt_turn", to.turn},
    {"tilt_climb", to.climb},
    {"tilt_roll", to.rollDeg},
    {"tilt_pitch", to.pitchDeg},
    {"tilt_valid", to.valid ? 1.0f : 0.0f},
    {"calibrated", tilt_.calibrated() ? 1.0f : 0.0f},
    {"calibrating", tilt_.calibrating() ? 1.0f : 0.0f},
    {"stick_turn", stickTurn_},
    {"stick_climb", stickClimb_},
    {"mode", (float)(int)mode_},
    {"best", (float)persist_.best},
    {"best_combo", (float)persist_.bestCombo},
    {"ring_dist", ringDist},
    {"ring_alt", haveRing ? r.c.z : -1.0f},
    {"mist", mist_},
    {"shake", shake_},
    {"fps", prof_.fps()},
  };
  for (const Entry& e : table) {
    if (strcmp(e.n, name) == 0) {
      out = e.v;
      return true;
    }
  }
  return false;
}

}  // namespace kj
