// audio.h - portable 16 kHz mono synth (engine, roar, wind, spray + one-shot voices).
// OWNER: group D (audio_synth.cpp). The platform (group A) only pumps Synth::render() into
// M5.Speaker.playRaw on one channel; the simulator writes the same samples to a WAV.
// Thread model: setParams/trigger/render are all called from the game loop task (no locks).
#pragma once
#include <stdint.h>

namespace kj {

enum class Sfx : uint8_t {
  Chime = 0,   // ring pass; pitch = 2^(combo/12)
  Perfect,     // chime + octave
  Miss,        // low two-note
  Roll,        // band-sweep swish 600 -> 3000 Hz, 0.4 s
  Ignite,      // boost thump + engine overshoot
  Slap,        // skim touch
  Whoosh,      // cloud entry
  Crash,       // noise burst tau 0.5 s + 90 Hz drop
  PullUp,      // 1000 Hz square 80 ms (game retriggers every 0.16 s while warning)
  Tick,        // timer < 5 s
  Blip,        // menu / countdown
  Go,          // countdown end
  Count
};

struct AudioParams {
  float speedRatio;   // speed / V_CRUISE
  float boost;        // 0..1
  float wind;         // 0..1.6 (dives louder)
  float spray;        // wake strength 0..1
  bool inCloud;
  bool engineOn;      // false on title-less screens / crash
};

class Synth {
 public:
  static constexpr int RATE = 16000;
  void init(uint32_t seed);
  void setParams(const AudioParams& p) { target_ = p; }
  void trigger(Sfx s, float pitch = 1.0f);
  // Render n mono samples (int16). Parameters ramp linearly across the block (no zipper).
  void render(int16_t* out, int n);
  void setMute(bool m) { mute_ = m; }
  bool muted() const { return mute_; }

 private:
  static constexpr int VOICES = 4;
  // s2: second filter state (group D private addition)
  struct Voice { uint8_t sfx; float t, pitch, phase, state; bool on; float s2; };
  AudioParams target_ = {1.0f, 0.0f, 0.0f, 0.0f, false, false};
  AudioParams cur_ = {1.0f, 0.0f, 0.0f, 0.0f, false, false};
  Voice voices_[VOICES] = {};
  float ph1_ = 0.0f, ph2_ = 0.0f, lp1_ = 0.0f, lp2_ = 0.0f, bp1_ = 0.0f, bp2_ = 0.0f, hp_ = 0.0f;
  uint32_t noise_ = 0x1u;
  bool mute_ = false;
  // private (group D): wind band filter, spray bursts, smoothed cloud / engine gates
  float wd1_ = 0.0f, wd2_ = 0.0f, hpIn_ = 0.0f;
  float burstT_ = 0.0f, burstAmp_ = 0.5f;
  float cloud_ = 0.0f, eng_ = 0.0f;
};

}  // namespace kj
