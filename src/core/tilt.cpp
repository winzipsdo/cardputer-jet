// tilt.cpp - IMU tilt steering (DESIGN.md section 8). OWNER: group D.
// Device frame (right-handed): X right (toward Enter), Y away from the user, Z out of the keys.
#include "tilt.h"
#include "config.h"
#include "mathx.h"
#include "fastmath_d.h"

namespace kj {

namespace {

// Soft deadzone + curve: u = sat((|a| - dz) / (full - dz)); out = sign * u * (LIN + (1 - LIN) u).
float shapeAxis(float deg, float dz, float full) {
  float a = fabsf(deg);
  float span = full - dz;
  if (span < 0.1f) span = 0.1f;
  float u = sat((a - dz) / span);
  float out = u * (cfg::TILT_CURVE_LIN + (1.0f - cfg::TILT_CURVE_LIN) * u);
  return deg < 0.0f ? -out : out;
}

// One-pole low-pass that lands exactly on 0 when the target is 0 (so "no input" is exact).
void lowPass(float& y, float target, float alpha) {
  y += (target - y) * alpha;
  if (target == 0.0f && fabsf(y) < 1e-4f) y = 0.0f;
}

bool finiteVec(const Vec3& v) {
  // NaN fails every comparison; huge values are rejected too.
  return fabsf(v.x) < 1e6f && fabsf(v.y) < 1e6f && fabsf(v.z) < 1e6f;
}

}  // namespace

void TiltController::configure(Sens sens, bool invert, bool drift, int8_t rollSign,
                               int8_t pitchSign) {
  sens_ = (int)sens <= 2 ? sens : Sens::Med;
  invert_ = invert;
  drift_ = drift;
  rollSign_ = rollSign < 0 ? (int8_t)-1 : (int8_t)1;
  pitchSign_ = pitchSign < 0 ? (int8_t)-1 : (int8_t)1;
}

void TiltController::beginCalibration(float windowSec) {
  calibrating_ = true;
  timedOut_ = false;
  calWindow_ = windowSec < 0.0f ? 0.0f : windowSec;
  calElapsed_ = 0.0f;
  stillTime_ = 0.0f;
  calSumA_ = v3(0.0f, 0.0f, 0.0f);
  calSumG_ = v3(0.0f, 0.0f, 0.0f);
  calN_ = 0;
}

float TiltController::calibProgress() const {
  if (calibrating_) return sat(stillTime_ / cfg::TILT_CAL_STILL_S);
  return calibrated_ ? 1.0f : 0.0f;
}

void TiltController::setNeutral(const Vec3& n) {
  if (!finiteVec(n)) return;
  float l = length(n);
  if (!(l > 0.5f) || l > 2.0f) return;  // reject garbage from storage
  n_ = n * (1.0f / l);
  nCal_ = n_;
}

JET_D_HOT void TiltController::update(const ImuSample& s, float dt) {
  if (!(dt > 0.0f)) return;
  if (dt > 0.1f) dt = 0.1f;

  // A fresh but unusable sample (non-finite, or |a| ~ 0 from a glitching sensor) is handled
  // exactly like a missing one: the calibration clock still runs (so a window always ends)
  // and the output goes stale instead of holding the last turn/climb forever.
  Vec3 a = v3(s.ax, s.ay, s.az);
  Vec3 g = v3(s.gx, s.gy, s.gz);
  float an = 0.0f;
  bool usable = s.fresh && finiteVec(a) && finiteVec(g);
  if (usable) {
    an = length(a);
    if (!(an >= 0.05f)) usable = false;  // sensor glitch
  }
  if (!usable) {
    // Never integrate the gyro twice. Keep the last output until stale.
    staleT_ += dt;
    if (calibrating_) {
      calElapsed_ += dt;
      if (calElapsed_ >= calWindow_) { calibrating_ = false; timedOut_ = true; }
    }
    if (staleT_ >= cfg::TILT_STALE_S) {
      out_.turn = 0.0f;
      out_.climb = 0.0f;
      out_.valid = false;
    }
    return;
  }
  staleT_ = 0.0f;

  Vec3 ah = a * (1.0f / an);
  float aerr = fabsf(an - 1.0f);

  if (!initialised_) {
    u_ = ah;
    initialised_ = true;
  }

  // ---- calibration window: needs TILT_CAL_STILL_S of continuous stillness.
  if (calibrating_) {
    calElapsed_ += dt;
    if (length(g) < cfg::TILT_CAL_STILL_DPS && aerr < cfg::TILT_ACC_TOL_FAST) {
      stillTime_ += dt;
      calSumA_ += ah;
      calSumG_ += g;
      ++calN_;
      if (stillTime_ >= cfg::TILT_CAL_STILL_S && calN_ > 0) {
        n_ = normalize(calSumA_);
        bias_ = calSumG_ * (1.0f / (float)calN_);
        nCal_ = n_;
        u_ = n_;
        calibrated_ = true;
        calibrating_ = false;
        timedOut_ = false;
        driftHold_ = 0.0f;
        out_.turn = 0.0f;
        out_.climb = 0.0f;
      }
    } else {
      stillTime_ = 0.0f;
      calSumA_ = v3(0.0f, 0.0f, 0.0f);
      calSumG_ = v3(0.0f, 0.0f, 0.0f);
      calN_ = 0;
    }
    if (calibrating_ && calElapsed_ >= calWindow_) {
      calibrating_ = false;
      timedOut_ = true;  // keep the stored neutral
    }
  }

  // ---- 1. predict with the gyro: u <- normalize(u + (u x w) dt)
  Vec3 w = (g - bias_) * DEG;
  u_ = normalize(u_ + cross(u_, w) * dt);

  // ---- 2. correct toward the measured gravity direction
  float tau = 0.0f;
  if (aerr < cfg::TILT_ACC_TOL_FAST) tau = cfg::TILT_TAU_STILL;
  else if (aerr < cfg::TILT_ACC_TOL_SLOW) tau = cfg::TILT_TAU_MOVING;
  if (tau > 0.0f) {
    float k = 1.0f - expf(-dt / tau);
    u_ = normalize(u_ + (ah - u_) * k);
  }

  // ---- 3. angles in the neutral basis
  const Vec3 X = v3(1.0f, 0.0f, 0.0f), Y = v3(0.0f, 1.0f, 0.0f);
  Vec3 zn = n_;
  Vec3 xn = X - zn * dot(X, zn);
  if (length(xn) < 0.2f) xn = Y - zn * dot(Y, zn);  // device held on its side (degenerate)
  xn = normalize(xn);
  Vec3 yn = cross(zn, xn);
  // Roll axis = the device's own long axis (Y) made perpendicular to xn; zd completes the
  // frame. Rolling about the device Y axis then measures exactly; rolling about the
  // horizontal forward axis (yn) reads slightly high. The 0.75*u.zn floor bounds that
  // over-read for steep hold poses (see the integrator notes).
  Vec3 yd = normalize(Y - xn * dot(Y, xn));
  Vec3 zd = cross(xn, yd);
  float uzn = dot(u_, zn);
  float den = dot(u_, zd);
  if (den < 0.75f * uzn) den = 0.75f * uzn;
  float rollRaw = atan2f(-dot(u_, xn), den) * RAD2DEG;
  float pitchRaw = atan2f(-dot(u_, yn), uzn) * RAD2DEG;
  float rollDeg = (float)rollSign_ * rollRaw;
  float pitchDeg = (float)pitchSign_ * pitchRaw;
  out_.rollDeg = rollDeg;
  out_.pitchDeg = pitchDeg;

  float alpha = 1.0f - expf(-TAU * cfg::TILT_OUT_HZ * dt);

  // ---- guard: upside down or extreme pitch -> output 0, invalid ("TILT?")
  if (fabsf(pitchDeg) > cfg::TILT_GUARD_PITCH_DEG || uzn < 0.2f) {
    out_.turn = 0.0f;
    out_.climb = 0.0f;
    out_.valid = false;
    driftHold_ = 0.0f;
    return;
  }

  // ---- 4. shape
  int si = (int)sens_;
  float turn = shapeAxis(rollDeg, cfg::TILT_ROLL_DZ[si], cfg::TILT_ROLL_FULL[si]);
  float climb = -shapeAxis(pitchDeg, cfg::TILT_PITCH_DZ[si], cfg::TILT_PITCH_FULL[si]);
  if (invert_) climb = -climb;

  // ---- 5. output low-pass (10 Hz)
  lowPass(out_.turn, turn, alpha);
  lowPass(out_.climb, climb, alpha);
  out_.valid = true;

  // ---- neutral drift: slowly re-centre the pitch neutral while flying level
  if (drift_ && !calibrating_) {
    if (fabsf(pitchDeg) < cfg::TILT_DRIFT_WINDOW_DEG && fabsf(out_.turn) < cfg::TILT_DRIFT_TURN)
      driftHold_ += dt;
    else
      driftHold_ = 0.0f;
    if (driftHold_ > cfg::TILT_DRIFT_HOLD_S) {
      // rotate n about xn toward u by k * (raw pitch angle)
      float phi = pitchRaw * DEG * (1.0f - expf(-dt / cfg::TILT_DRIFT_TAU));
      Vec3 nn = normalize(zn * cosf(phi) - yn * sinf(phi));
      if (dot(nn, nCal_) >= cosf(cfg::TILT_DRIFT_LIMIT_DEG * DEG)) n_ = nn;
    }
  }
}

}  // namespace kj
