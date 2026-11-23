// synth_imu.h - synthetic BMI270 samples for the simulator (DESIGN.md section 18).
// OWNER: group E.
//
// Device frame = M5 doc axes (same as kj::ImuSample): X right (toward Enter), Y away from the
// user (toward the screen edge), Z out of the keys. World: z up.
//
// Orientation R maps device-frame vectors to the world ("hold pose x script roll/pitch",
// i.e. the script rotations are applied in the DEVICE frame after the hold pose):
//   R = Rx(hold) * Rx(-pitch) * Ry(roll)
//     hold  = hold-pose angle (default 35 deg): the screen edge raised toward the face,
//             a right-handed rotation about the device X axis;
//     pitch = script pitch about the device X axis, + = top (screen) edge pushed away / down
//             -> the game dives;
//     roll  = script roll about the device's own long (Y) axis, + = right edge down
//             -> the game turns right.
//   Gravity in the device frame is then u = (-cos(h-p) sin r, sin(h-p), cos(h-p) cos r), so
//   tilt.cpp (roll measured about the device Y axis, pitch about xn relative to a neutral
//   calibrated at the hold pose) reads back exactly (roll, pitch). NOTE: the plain tilt.h
//   formula roll = atan2(-(u.xn), u.zn) would read this roll ~cos(hold) low.
//
// Sample:
//   accel = R^T (0,0,1) + N(0, 0.01 g) per axis               (lying flat: az = +1)
//   gyro  = body rate from R(t-dt)^T R(t) (deg/s, right-handed) + N(0, 0.5 dps) + bias
//           bias = (0.8, -0.5, 0.3) dps
// Checks against DESIGN.md section 22: right edge down -> ax < 0; screen edge away and down
// -> ay < 0; rolling right -> gy > 0.
#pragma once
#include <stdint.h>
#include "input.h"
#include "types.h"

namespace sim {

class SynthImu {
 public:
  static constexpr float HOLD_DEFAULT_DEG = 35.0f;
  static constexpr float ACC_NOISE_G = 0.01f;
  static constexpr float GYRO_NOISE_DPS = 0.5f;

  void reset(uint32_t seed);
  // Ramp the script roll/pitch (degrees, relative to the hold pose) linearly over rampSec.
  void setTilt(float rollDeg, float pitchDeg, float rampSec);
  // Ramp the hold pose (degrees toward the face) linearly over rampSec.
  void setHoldPose(float holdDeg, float rampSec);
  void setNoise(bool on) { noise_ = on; }
  bool noise() const { return noise_; }
  // Advance the ramps by dt and return the sample for this instant (always fresh).
  kj::ImuSample step(float dt, uint32_t usec);

  float roll() const { return roll_.cur; }
  float pitch() const { return pitch_.cur; }
  float hold() const { return hold_.cur; }

  // Noise-free gravity ("up") vector in the device frame for the given angles (degrees).
  static kj::Vec3 upDevice(float holdDeg, float rollDeg, float pitchDeg);

 private:
  struct Ramp {
    float cur = 0.0f, from = 0.0f, to = 0.0f, t = 0.0f, dur = 0.0f;
    void set(float target, float sec);
    void advance(float dt);
  };
  static void orientation(float holdDeg, float rollDeg, float pitchDeg, float R[9]);
  float gauss();
  float uniform();

  Ramp roll_, pitch_, hold_;
  float prevR_[9] = {1, 0, 0, 0, 1, 0, 0, 0, 1};
  bool havePrev_ = false;
  bool noise_ = true;
  uint32_t rng_ = 0x5EED1234u;
  bool haveSpare_ = false;
  float spare_ = 0.0f;
};

}  // namespace sim
