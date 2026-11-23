// flight.h - arcade flight model (fixed 1/120 s sub-steps), crash detection.
// OWNER: group D (flight.cpp). SHARED CONTRACT (JetState is read by B, C, D).
#pragma once
#include <stdint.h>
#include "types.h"

namespace kj {

class World;

struct JetState {
  Vec3 pos;            // pivot position; x,y wrapped to [0, WORLD_U)
  Vec3 vel;            // u/s (includes roll dash)
  float yaw;           // heading psi (rad), forward = (sin, -cos, 0)
  float yawRate;       // rad/s
  float bank;          // visual bank (rad), + = right wing down
  float rollAngle;     // barrel-roll stunt angle (rad), added to bank for rendering
  float pitch;         // climb angle (rad), + = nose up; can pass pi/2 only in loops
  float pitchRate;     // rad/s
  float speed;         // u/s along the nose
  float boost;         // 0..1 eased boost amount
  float boostEnergy;   // 0..1 tank
  float energy;        // speed multiplier from climb/dive, 0.8..ENERGY_MAX
  float g;             // 0..1 "pull" gauge: drives trails and audio
  float rollT;         // 0..1 progress of the current barrel roll
  int8_t rollDir;      // -1 left, +1 right, 0 none
  int8_t rollQueued;   // one buffered roll request
  float rollCooldown;
  float loopT;         // 0..1 progress of an auto loop
  bool looping;
  float invuln;        // s of respawn invulnerability left
  float time;          // flight clock (s)
  float heightAboveSurface;  // pos.z - ground (or sea) height under the pivot
  bool overWater;
  bool rolling() const { return rollDir != 0; }
  float totalRoll() const { return bank + rollAngle; }
};

struct FlightInput {
  float turn;    // -1..1 (+ = right), already includes tilt + keys + assist, clamped
  float climb;   // -1..1 (+ = climb)
  bool boost;    // held
  int8_t rollReq;  // -1 / +1 request this step (edge), 0 none
  bool loopReq;    // edge
};

enum class CrashCause : uint8_t { None = 0, Terrain, Sea };

struct FlightEvents {
  bool skimTouch;      // kissed the sea without crashing (shake 0.35, spray, combo kept)
  bool boostIgnite;    // boost rising edge
  bool rollStart;
  bool closeCall;      // passed within CLOSE_CALL_DIST of terrain without crashing
  bool enteredCloud, exitedCloud;
  bool inCloud;
  bool pullUp;         // GPWS warning active
  bool loopDone;
};

class Flight {
 public:
  void reset(const Vec3& pos, float yaw);
  // One fixed sub-step. Returns the crash cause (None if alive). Ignores crashes while invuln > 0.
  CrashCause step(const FlightInput& in, const World& w, Assist assist, float dt, FlightEvents& ev);
  const JetState& s() const { return s_; }
  JetState& mut() { return s_; }
  // Predicted min gap to ground/sea along the current velocity, `sec` ahead.
  float predictedGap(const World& w, float sec) const;

 private:
  JetState s_ = {};
  float easyFloor_ = 0.0f;
  float closeCallCd_ = 0.0f;
  bool wasInCloud_ = false;
  float prevBoostCmd_ = 0.0f;
  bool boostDry_ = false;   // tank ran dry while engaged: needs a fresh boost press
  // private (group D): EASY deliberate-dive timer, sea-contact edge detection
  float diveHold_ = 0.0f;
  bool touching_ = false;
};

}  // namespace kj
