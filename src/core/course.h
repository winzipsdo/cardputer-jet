// course.h - ring gates: chained placement, pass/miss detection, magnet assist, drawing,
// off-screen guide. OWNER: group D (course.cpp). Uses draw.h primitives from group C.
#pragma once
#include <stdint.h>
#include "types.h"
#include "frame.h"
#include "camera.h"
#include "perf.h"
#include "config.h"

namespace kj {

struct JetState;
class World;

enum class RingKind : uint8_t { Low = 0, Mid, High };
enum class RingResult : uint8_t { None = 0, Through, Perfect, Missed };

struct Ring {
  Vec3 c;          // centre (wrapped)
  float yaw;       // ring normal = forward(yaw)
  float radius;    // u
  RingKind kind;
  bool live;
  float passAnim;  // 0 = idle; >0 seconds since passed (scale 1 -> 1.6 + fade over 0.3 s)
};

struct RingEvent {
  RingResult result;
  float lateral;     // distance from the centre at the plane crossing (u)
  uint16_t index;    // ring number in this run (1-based)
  Vec3 pos;          // ring centre
  Vec3 normal;
  RingKind kind;
};

class Course {
 public:
  void reset(const JetState& s, const World& w, uint32_t seed);
  // prevPos: jet pivot at the start of this frame (wrapped). Returns at most one event per call.
  RingEvent update(const Vec3& prevPos, const JetState& s, const World& w, float dt);
  // Magnet assist (scaled by cfg::ASSIST_GAIN[assist] and by (1 - |player input|)).
  void assist(const JetState& s, Assist a, float playerTurn, float playerClimb, float& turnAdd,
              float& climbAdd) const;
  // Current ring amber (pulsing), next cyan (dithered), far one = beacon dot + light column.
  void draw(FrameBuf& fb, const View& v, Tod tod, float time, RenderStats& st) const;
  bool current(Ring& out) const;
  uint16_t passed() const { return passed_; }
  // Off-screen indicator for the current ring: edge position + vertical hint (-1 below, 0, +1 above).
  bool guide(const View& v, int& edgeX, int& edgeY, int& vertical) const;

 private:
  Ring rings_[cfg::LIVE_RINGS] = {};
  int cur_ = 0;
  uint16_t passed_ = 0, spawned_ = 0;
  float sinceProgress_ = 0.0f, lastDist_ = 0.0f;
  uint32_t seed_ = 1;
  int8_t lastTurnSign_ = 1;
  // private (group D): the ring just passed, kept for its 0.3 s pass animation
  Ring ghost_ = {};
};

}  // namespace kj
