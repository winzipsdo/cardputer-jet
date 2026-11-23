// plat_audio.cpp - M5.Speaker setup (mixer task pinned to core 0) and the 3-buffer stream pump
// that feeds kj::Synth output to playRaw() on one virtual channel.
// OWNER: group A. See docs/DESIGN.md section 16.
//
// Pump rule: playRaw() keeps a POINTER to the samples, and a channel holds at most 2 queued
// buffers. With 3 rotating buffers and "refill only while isPlaying(ch) < 2", the buffer being
// rendered is never queued or playing. A hitch longer than ~48 ms just plays silence.
#include <Arduino.h>
#include <M5Cardputer.h>

#include "core/audio.h"
#include "core/config.h"
#include "core/game.h"
#include "core/perf.h"
#include "platform/cardputer/plat.h"
#include "platform/cardputer/plat_internal.h"

namespace plat {

namespace {

constexpr int BLOCK = kj::cfg::AUDIO_BLOCK;      // 384 samples = 24 ms at 16 kHz
constexpr int NBUF = kj::cfg::AUDIO_BUFFERS;     // 3
constexpr int CH = kj::cfg::AUDIO_CHANNEL;       // 0
constexpr int MAX_BLOCKS_PER_PUMP = 2;
static_assert(NBUF >= 3, "the stream pump needs 3 rotating buffers");

#if JET_AUDIO
int16_t g_buf[NBUF][BLOCK];                     // 2,304 B static
int g_next = 0;
#endif
bool g_ok = false;
int g_volume = -1;                              // last value given to setVolume (-1 = none)

}  // namespace

void audioPump(kj::Synth& synth, kj::Profiler& prof) {
#if JET_AUDIO
  if (!g_ok) return;
  prof.begin(kj::ST_AUDIO);
  for (int i = 0; i < MAX_BLOCKS_PER_PUMP; ++i) {
    if (M5.Speaker.isPlaying((uint8_t)CH) >= 2) break;
    int16_t* b = g_buf[g_next];
    synth.render(b, BLOCK);
    M5.Speaker.playRaw(b, (size_t)BLOCK, (uint32_t)kj::cfg::AUDIO_RATE, false, 1, CH, false);
    g_next = (g_next + 1) % NBUF;
  }
  prof.end(kj::ST_AUDIO);
#else
  (void)synth;
  (void)prof;
#endif
}

namespace detail {

bool audioBegin(uint8_t volume, bool mute) {
#if JET_AUDIO
  auto sc = M5.Speaker.config();
  sc.task_pinned_core = 0;   // mixer task on core 0: the game owns core 1
  sc.task_priority = 2;
  M5.Speaker.config(sc);
  g_ok = M5.Speaker.begin();  // the ES8311 is configured over I2C only here (setup, core 1)
  const uint8_t v = mute ? 0 : volume;
  M5.Speaker.setVolume(v);
  g_volume = v;
  logf("[SPK] ok=%d vol=%u core=%d", g_ok ? 1 : 0, (unsigned)volume, (int)sc.task_pinned_core);
  return g_ok;
#else
  (void)volume;
  (void)mute;
  logLine("[SPK] ok=0 disabled=JET_AUDIO0");
  return false;
#endif
}

bool audioOk() { return g_ok; }

uint8_t audioAppliedVolume() { return g_volume < 0 ? 0 : (uint8_t)g_volume; }

}  // namespace detail

void applySettings(const kj::Game& game) {
  detail::displaySetBrightness(game.brightness());
#if JET_AUDIO
  if (g_ok) {
    const int v = game.muted() ? 0 : (int)game.volume();
    if (v != g_volume) {
      M5.Speaker.setVolume((uint8_t)v);  // mute = volume 0 (the synth keeps running)
      g_volume = v;
    }
  }
#endif
}

}  // namespace plat
