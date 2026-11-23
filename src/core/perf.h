// perf.h - per-stage profiler and work counters. Same code on device and in the simulator,
// so [STAT] lines are directly comparable (device times are real; sim times are PC times
// plus an ESP32 estimate computed from the counters).
// OWNER: group A (perf.cpp). SHARED CONTRACT.
#pragma once
#include <stdint.h>

namespace kj {

enum Stage : uint8_t {
  ST_INPUT = 0,  // platform: keyboard drain + IMU read            (recorded by platform)
  ST_SIM,        // game update: tilt, flight, camera, course, fx update, audio params
  ST_PAL,        // palette finalize (fx lerps -> LUT)
  ST_WORLD,      // renderWorld (terrain, sea, clouds)
  ST_SKY,        // renderSky
  ST_OBJ,        // rings, beacons
  ST_JET,        // jet shadow + jet + flame + outline + nav lights
  ST_FX,         // particles, trails, wake, speed lines, mist
  ST_HUD,        // HUD / menus / text
  ST_AUDIO,      // synth render + playRaw   (recorded by platform)
  ST_EXPAND,     // 2x LUT expand            (recorded by platform)
  ST_DMAWAIT,    // time blocked in waitDMA  (recorded by platform; large = display-bound)
  ST_COUNT
};

constexpr const char* STAGE_NAMES[ST_COUNT] = {"in", "sim", "pal", "wld", "sky", "obj",
                                               "jet", "fx", "hud", "aud", "exp", "dmaw"};

// Work counters, cleared at the start of every Game::render(), accumulated per window.
struct RenderStats {
  uint32_t steps;      // world march depth steps
  uint32_t samples;    // world column samples (terrain + cloud lookups)
  uint32_t fills;      // pixels written by the world march
  uint32_t voxels;     // jet voxels transformed
  uint32_t voxWrites;  // jet pixel writes attempted
  uint32_t flamePx;    // flame pixel writes attempted
  uint32_t particles;  // particles drawn
  uint32_t linePx;     // line pixels (rings, trails, speed lines)
  void clear() { steps = samples = fills = voxels = voxWrites = flamePx = particles = linePx = 0; }
};

// ESP32-S3 cost model (cycles per unit) used by the simulator's "esp_est" column.
// Least-squares fit to 71 device [STAT] windows (cardputer-adv, 240 MHz, Ring Run autopilot
// bench at night and day plus title flyover; same counters as the sim), mean error 4-5 % for
// wld and jet. cStep carries the per-step work that has no counter of its own: the row
// conversions and divide (~750 cycles), the 15-chunk classification (~3,000) and the cloud
// slab chunks. A fill is ~46 cycles because most uniform-chunk calls write a single row, so
// the per-call setup is spread over few pixels. Real code runs at ~2-3 cycles per instruction
// on this core (taken branches, load-use stalls), so instruction counts x 1.3 under-estimate.
// The jet weights live in tools/sim/sim_main.cpp (JET_C_*); cVoxel / cVoxWrite / cFlame here
// are the same fit for Profiler::estEspMs. fx is only loosely explained by its counters (~30 %):
// cLinePx is pinned to a plausible 60 cycles (lineF's inner loop) and a particle is ~1,345
// cycles; the fx stage's fixed ~0.28 ms is in the sim's ESP_FIXED_MS.
struct CostModel {
  float cSample = 25.6f, cFill = 46.0f, cStep = 4920.0f, cVoxel = 421.0f, cVoxWrite = 38.5f,
        cFlame = 97.0f, cParticle = 1345.0f, cLinePx = 60.0f, cyclesPerUs = 240.0f;
};

class Profiler {
 public:
  using ClockUs = uint32_t (*)();
  void init(ClockUs clock);
  void begin(Stage s);                 // start timing stage s
  void end(Stage s);                   // stop timing stage s (accumulates into this frame)
  void add(Stage s, uint32_t us);      // add externally measured time
  RenderStats& stats() { return cur_; }
  // Close the frame: frameUs = wall time since previous endFrame (pacing included).
  void endFrame(uint32_t frameUs);
  // True once per cfg::STAT_PERIOD_S of accumulated frame time; then call format() and reset().
  bool windowReady() const;
  // "fps=59.9 ft=16.7/18.2 c1=4.1/6.3 in=0.71 sim=0.20 ... | smp=9402 stp=77 fill=6104 vox=598 vw=4200 fl=120 prt=38 lp=600"
  int format(char* buf, int n) const;
  float estEspMs(const CostModel& cm) const;  // average per-frame estimate from counters
  void reset();
  float fps() const;
  float avgMs(Stage s) const;
  float coreMsAvg() const;   // sum of all stages except ST_DMAWAIT
  float frameMsMax() const;

 private:
  ClockUs clock_ = nullptr;
  uint32_t t0_[ST_COUNT] = {};
  uint32_t frameUs_[ST_COUNT] = {};     // current frame
  uint64_t sumUs_[ST_COUNT] = {};       // window sums
  uint32_t maxCoreUs_ = 0, maxFrameUs_ = 0;
  uint64_t windowUs_ = 0;
  uint32_t frames_ = 0;
  RenderStats cur_ = {};
  RenderStats sum_ = {};
};

}  // namespace kj
