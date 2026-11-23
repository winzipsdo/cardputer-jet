// tilt.h - IMU tilt steering: gyro-fused gravity ("up") estimate, neutral calibration,
// angles relative to the neutral pose, soft deadzone + curve, output low-pass.
// OWNER: group D (tilt.cpp). Pure C++, used on device and in the simulator (synthetic IMU).
//
// Algorithm (DESIGN.md section 8):
//  1. predict  u <- normalize(u + (u x w) dt), w = (gyro - bias) in rad/s   (device frame,
//     right-handed; sign of the cross product verified by the sim's synthetic rotation test)
//  2. correct  u <- normalize(u + k (a/|a| - u)), k = 1 - exp(-dt/tau),
//     tau = TILT_TAU_STILL if ||a|-1| < TOL_FAST, TILT_TAU_MOVING if < TOL_SLOW, else skip.
//  3. neutral basis zn = n, xn = normalize(X - (X.n)n), yn = zn x xn
//     roll axis = device Y made perpendicular to xn: yd = normalize(Y - (Y.xn)xn), zd = xn x yd
//     rollDeg  = rollSign  * deg(atan2(-(u.xn), max(u.zd, 0.75 u.zn)))   + = right edge down
//                (exact for a roll about the device's own long axis at any hold pose; the
//                 0.75 u.zn floor bounds the over-read for rolls about the horizontal axis)
//     pitchDeg = pitchSign * deg(atan2(-(u.yn), u.zn))   + = top edge pushed away -> dive
//     climb input = -pitchShaped (invert flips it)
//  4. shape  u = clamp((|ang|-dz)/(full-dz), 0, 1); out = sign*u*(LIN + (1-LIN)*u)
//  5. output one-pole low-pass at TILT_OUT_HZ.
#pragma once
#include <stdint.h>
#include "types.h"
#include "input.h"

namespace kj {

struct TiltOut {
  float turn;      // -1..1, + = right
  float climb;     // -1..1, + = climb
  float rollDeg;   // relative to neutral, signed (debug / HUD dot)
  float pitchDeg;
  bool valid;      // calibrated, fresh data, inside the guard range
};

class TiltController {
 public:
  void configure(Sens sens, bool invert, bool drift, int8_t rollSign, int8_t pitchSign);
  // Start a calibration window (TILT_CAL_STILL_S of stillness within windowSec timeout).
  void beginCalibration(float windowSec);
  bool calibrating() const { return calibrating_; }
  float calibProgress() const;      // 0..1 for the on-screen ring
  bool calibrated() const { return calibrated_; }
  bool calibTimedOut() const { return timedOut_; }   // fell back to the stored neutral
  void update(const ImuSample& s, float dt);
  TiltOut out() const { return out_; }
  Vec3 neutral() const { return n_; }
  void setNeutral(const Vec3& n);   // from NVS
  Vec3 up() const { return u_; }
  Vec3 gyroBias() const { return bias_; }

 private:
  Sens sens_ = Sens::Med;
  bool invert_ = false, drift_ = true;
  int8_t rollSign_ = 1, pitchSign_ = 1;
  Vec3 u_ = {0.0f, 0.0f, 1.0f};
  Vec3 n_ = {0.0f, 0.57f, 0.82f};   // typical "held toward the face" neutral until calibrated
  Vec3 bias_ = {0.0f, 0.0f, 0.0f};
  bool initialised_ = false;
  bool calibrating_ = false, calibrated_ = false, timedOut_ = false;
  float calWindow_ = 0.0f, calElapsed_ = 0.0f, stillTime_ = 0.0f;
  Vec3 calSumA_ = {0.0f, 0.0f, 0.0f}, calSumG_ = {0.0f, 0.0f, 0.0f};
  int calN_ = 0;
  float staleT_ = 0.0f, driftHold_ = 0.0f;
  Vec3 nCal_ = {0.0f, 0.57f, 0.82f};   // neutral at last calibration (drift limit reference)
  TiltOut out_ = {0.0f, 0.0f, 0.0f, 0.0f, false};
};

}  // namespace kj
