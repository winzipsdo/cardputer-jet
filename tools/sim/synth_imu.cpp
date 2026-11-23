// synth_imu.cpp - see synth_imu.h. OWNER: group E.
#include "synth_imu.h"

#include <math.h>

namespace sim {

namespace {

const float DEG2RAD = 0.0174532925f;
const float RAD2DEG = 57.2957795f;
const float BIAS_DPS[3] = {0.8f, -0.5f, 0.3f};

}  // namespace

void SynthImu::Ramp::set(float target, float sec) {
  from = cur;
  to = target;
  t = 0.0f;
  dur = sec > 0.0f ? sec : 0.0f;
  if (dur <= 0.0f) cur = to;
}

void SynthImu::Ramp::advance(float dt) {
  if (dur <= 0.0f || t >= dur) {
    cur = to;
    return;
  }
  t += dt;
  if (t >= dur) {
    cur = to;
  } else {
    cur = from + (to - from) * (t / dur);
  }
}

void SynthImu::reset(uint32_t seed) {
  roll_ = Ramp();
  pitch_ = Ramp();
  hold_ = Ramp();
  hold_.cur = hold_.from = hold_.to = HOLD_DEFAULT_DEG;
  havePrev_ = false;
  noise_ = true;
  rng_ = seed ? seed : 0x5EED1234u;
  haveSpare_ = false;
  spare_ = 0.0f;
}

void SynthImu::setTilt(float rollDeg, float pitchDeg, float rampSec) {
  roll_.set(rollDeg, rampSec);
  pitch_.set(pitchDeg, rampSec);
}

void SynthImu::setHoldPose(float holdDeg, float rampSec) { hold_.set(holdDeg, rampSec); }

// R = Rx(hold) * Rx(-pitch) * Ry(roll) = Rx(hold - pitch) * Ry(roll), row-major.
void SynthImu::orientation(float holdDeg, float rollDeg, float pitchDeg, float R[9]) {
  const float a = (holdDeg - pitchDeg) * DEG2RAD, r = rollDeg * DEG2RAD;
  const float ca = cosf(a), sa = sinf(a), cr = cosf(r), sr = sinf(r);
  R[0] = cr;       R[1] = 0.0f; R[2] = sr;
  R[3] = sa * sr;  R[4] = ca;   R[5] = -sa * cr;
  R[6] = -ca * sr; R[7] = sa;   R[8] = ca * cr;
}

kj::Vec3 SynthImu::upDevice(float holdDeg, float rollDeg, float pitchDeg) {
  float R[9];
  orientation(holdDeg, rollDeg, pitchDeg, R);
  // R^T * (0,0,1) = third row of R.
  return kj::v3(R[6], R[7], R[8]);
}

float SynthImu::uniform() {
  rng_ ^= rng_ << 13;
  rng_ ^= rng_ >> 17;
  rng_ ^= rng_ << 5;
  return ((float)(rng_ >> 8) + 0.5f) * (1.0f / 16777216.0f);  // (0,1)
}

float SynthImu::gauss() {
  if (haveSpare_) {
    haveSpare_ = false;
    return spare_;
  }
  const float u1 = uniform(), u2 = uniform();
  const float mag = sqrtf(-2.0f * logf(u1));
  const float ang = 6.28318531f * u2;
  spare_ = mag * sinf(ang);
  haveSpare_ = true;
  return mag * cosf(ang);
}

kj::ImuSample SynthImu::step(float dt, uint32_t usec) {
  roll_.advance(dt);
  pitch_.advance(dt);
  hold_.advance(dt);

  float R[9];
  orientation(hold_.cur, roll_.cur, pitch_.cur, R);

  // Body angular velocity from the incremental rotation dR = Rprev^T * R (R = Rprev * dR).
  float w[3] = {0.0f, 0.0f, 0.0f};
  if (havePrev_ && dt > 0.0f) {
    float d[9];
    for (int i = 0; i < 3; ++i)
      for (int j = 0; j < 3; ++j)
        d[i * 3 + j] = prevR_[0 * 3 + i] * R[0 * 3 + j] + prevR_[1 * 3 + i] * R[1 * 3 + j] +
                       prevR_[2 * 3 + i] * R[2 * 3 + j];
    const float vx = d[7] - d[5], vy = d[2] - d[6], vz = d[3] - d[1];  // 2 sin(th) * axis
    float c = (d[0] + d[4] + d[8] - 1.0f) * 0.5f;
    c = c > 1.0f ? 1.0f : (c < -1.0f ? -1.0f : c);
    const float th = acosf(c);
    const float s = sinf(th);
    const float k = (s > 1e-6f) ? th / (2.0f * s) : 0.5f;
    w[0] = vx * k / dt * RAD2DEG;
    w[1] = vy * k / dt * RAD2DEG;
    w[2] = vz * k / dt * RAD2DEG;
  }
  for (int i = 0; i < 9; ++i) prevR_[i] = R[i];
  havePrev_ = true;

  kj::ImuSample smp;
  smp.ax = R[6];
  smp.ay = R[7];
  smp.az = R[8];
  smp.gx = w[0] + BIAS_DPS[0];
  smp.gy = w[1] + BIAS_DPS[1];
  smp.gz = w[2] + BIAS_DPS[2];
  if (noise_) {
    smp.ax += ACC_NOISE_G * gauss();
    smp.ay += ACC_NOISE_G * gauss();
    smp.az += ACC_NOISE_G * gauss();
    smp.gx += GYRO_NOISE_DPS * gauss();
    smp.gy += GYRO_NOISE_DPS * gauss();
    smp.gz += GYRO_NOISE_DPS * gauss();
  }
  smp.usec = usec;
  smp.fresh = true;
  return smp;
}

}  // namespace sim
