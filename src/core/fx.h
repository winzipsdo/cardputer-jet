// fx.h - particles, wingtip trails, wake V, speed lines, in-cloud mist.
// OWNER: group C (fx.cpp). SHARED CONTRACT.
// All storage is fixed-size inside Fx (no allocation). Everything is drawn AFTER the jet,
// depth-tested against the frame depth (which already contains the jet), never writing depth.
#pragma once
#include <stdint.h>
#include "types.h"
#include "frame.h"
#include "camera.h"
#include "perf.h"
#include "config.h"

namespace kj {

struct JetState;
struct JetPose;
class World;

enum class Burst : uint8_t {
  RingPass,    // 16 sparks on the ring rim (pos = ring centre, dir = ring normal, strength = radius)
  Perfect,     // ring pass + extra white sparks
  CloudEntry,  // ring of 12 vapour puffs around the jet
  SkimTouch,   // spray burst at the jet
  Crash,       // fire 24 + debris 14 + smoke 10 (land)
  CrashSea,    // tall white spray column 40 + foam ring
  Ignite       // boost ignition puff at the nozzle
};

struct Particle {
  Vec3 p, v;
  float age, life;
  float size;       // u (projected with m)
  uint8_t kind;     // internal (smoke, spray, vapour, fire, debris, spark)
  uint8_t color;    // base palette index
  uint8_t pad[2];
};

struct TrailPoint {
  Vec3 p;
  float t;          // spawn time
};

struct WakeMark {
  Vec3 p;           // on the sea (z = 0)
  float ax, ay;     // across direction
  float t;          // spawn time
  float k;          // strength at spawn
};

class Fx {
 public:
  void reset();
  // dt is scaled sim time. inCloud from FlightEvents. Spawns smoke/spray/vapour/trails/wake.
  void update(const JetState& s, const JetPose& pose, const World& w, float dt, bool inCloud);
  void burst(Burst b, const Vec3& pos, const Vec3& dir, float strength);
  // Trails, wake, particles, speed lines. Depth-tested, no depth writes.
  void draw(FrameBuf& fb, const View& v, Tod tod, float time, RenderStats& st);
  // Full-frame dithered mist (pal::MIST) at coverage amount (0..1); frame = frame counter.
  void drawMist(FrameBuf& fb, float amount, uint32_t frame);
  float wakeStrength() const { return wake_; }

 private:
  Particle parts_[cfg::PARTICLES] = {};
  int nextPart_ = 0;
  TrailPoint trail_[2][cfg::TRAIL_POINTS] = {};
  int trailHead_[2] = {0, 0};
  float trailDist_ = 0.0f;
  WakeMark wake_marks_[cfg::WAKE_MARKS] = {};
  int wakeHead_ = 0;
  float wakeDist_ = 0.0f;
  Vec3 speed_[cfg::SPEEDLINES] = {};
  float smokeT_ = 0.0f, sprayT_ = 0.0f, vapourT_ = 0.0f;
  float wake_ = 0.0f;       // current wake strength 0..1
  float g_ = 0.0f;          // trail gauge copy
  float speedVis_ = 0.0f;   // speed-line visibility 0..1
  Vec3 vel_ = {0.0f, 0.0f, 0.0f};
  float now_ = 0.0f;
  uint32_t rng_ = 0x2545F491u;
};

}  // namespace kj
