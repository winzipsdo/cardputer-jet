// main.cpp - MOONSKIM entry point for the Cardputer-Adv (Arduino loop task, core 1).
// OWNER: group A. See docs/DESIGN.md sections 3 and 17.
//
// Frame (one per loop() call on core 1):
//   pollInput (snapshot of the core-0 input task) -> game.update -> game.render -> audioPump ->
//   present (copy px + LUT to the present task's staging buffer) -> profiler.endFrame ->
//   slow-frame / bench bookkeeping -> applySettings -> periodicLogs -> logFlush -> pace to the
//   60 fps grid (vTaskDelay(1) while > 1.5 ms remain, kicking the input poll at <= 2.6 ms
//   left, then spin).
// Core 0 meanwhile expands and pushes the previous frame (present task, ~13.4 ms of SPI at
// 40 MHz, fully overlapped) and polls I2C input (input task, ~1 ms per kick).
//
// Core-1 budget per frame ([STAT] c1 = sum of these, dmaw excluded):
//   in   snapshot copy + serial poll (~0.02 ms; the I2C poll runs on core 0, see in0=)
//   sim pal wld sky obj jet fx hud   the game (unchanged)
//   aud  synth + playRaw
//   exp  memcpy of px (8,160 B) + lut2x (1 KB) into the staging buffer (~0.03 ms; expand2x
//        itself now runs on core 0, see p0=)
//   dmaw blocked waiting for the staging buffer (0 unless the display is the bottleneck,
//        i.e. core-1 work < ~13.4 ms per frame at 40 MHz while uncapped)
// 60 fps therefore holds while c1 <= ~16.5 ms (was ~10 ms + the 6.5 ms push wait).
#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include "core/config.h"
#include "core/game.h"
#include "core/input.h"
#include "core/perf.h"
#include "platform/cardputer/plat.h"
#include "platform/cardputer/plat_internal.h"

SET_LOOP_TASK_STACK_SIZE(12 * 1024);

namespace {

using kj::Profiler;
using kj::Stage;

kj::Game g_game;  // THE game (static, ~20 KB; never on the stack)
kj::Persist g_persist;
kj::InputFrame g_in;

uint32_t g_lastEndUs = 0;     // time of the previous endFrame (frame time includes pacing)
uint32_t g_nextUs = 0;        // pacing grid
uint32_t g_frameNo = 0;
uint32_t g_lastSlowWarnUs = 0;

// Commands injected into the core on the first frames (debug / bench envs).
char g_bootCmds[4];
int g_bootN = 0, g_bootI = 0;

constexpr uint32_t BENCH_US = 20u * 1000000u;  // 20 s autopilot bench ('b')
constexpr uint32_t SLOW_US = (uint32_t)(kj::cfg::SLOW_FRAME_MS * 1000.0f);

// Per-frame stage times reconstructed from the profiler's window sums
// (avgMs(s) * framesInWindow == window sum; perf.cpp guarantees this).
struct StageTap {
  int n = 0;                       // frames in the current profiler window
  float prev[kj::ST_COUNT] = {};   // window sums after the previous frame (ms)
  float prevCore = 0.0f;
  float cur[kj::ST_COUNT] = {};    // this frame (ms)
  float curCore = 0.0f;

  void update(const Profiler& p) {
    ++n;
    const float fn = (float)n;
    for (int s = 0; s < kj::ST_COUNT; ++s) {
      const float sum = p.avgMs((Stage)s) * fn;
      cur[s] = sum - prev[s];
      prev[s] = sum;
    }
    const float core = p.coreMsAvg() * fn;
    curCore = core - prevCore;
    prevCore = core;
  }
  void reset() {
    n = 0;
    for (int s = 0; s < kj::ST_COUNT; ++s) prev[s] = 0.0f;
    prevCore = 0.0f;
  }
  int worstStage() const {  // largest CPU stage this frame (dmaw excluded: that is idle wait)
    int w = 0;
    for (int s = 1; s < kj::ST_COUNT; ++s)
      if (s != kj::ST_DMAWAIT && cur[s] > cur[w]) w = s;
    return w;
  }
};
StageTap g_tap;

// Device bench: started by serial 'b' (forwarded to the core, which flies the autopilot and
// prints its own "[BENCH] core ..." line) or at boot in the bench env. This one measures PER
// FRAME and prints "[BENCH] perframe ..." after 20 s:
//   start_ms  frame time of the first frame (the core's startRun: spawn search, ring layout;
//             a scene cut, so not counted in the figures below)
//   ft_max    worst frame time of the rest; over17 / over22 count frames longer than 17 ms /
//             22.2 ms (< 45 fps). ft is measured end to end, so a frame whose work is 2 ms
//             longer than the previous one reads ~18.7 ms while it still made its 60 fps slot;
//   late      frames that really missed their slot (pace() found the deadline already passed).
// Serial 'B' re-prints the last results (plat_log.cpp).
struct Bench {
  static constexpr uint32_t MISS60_US = 17000;   // a 60 fps slot is 16,667 us
  static constexpr uint32_t MISS45_US = 22222;
  bool on = false;
  bool first = false;
  uint32_t t0 = 0;
  uint32_t frames = 0, over60 = 0, over45 = 0, late = 0;
  float startMs = 0.0f;
  float c1Sum = 0.0f, c1Max = 0.0f, wldMax = 0.0f, jetMax = 0.0f, fpsMin = 0.0f;
  float ftMax = 0.0f, dmawSum = 0.0f;
  bool haveWindow = false;

  void start(uint32_t now) {
    on = true;
    first = true;
    t0 = now;
    frames = over60 = over45 = late = 0;
    startMs = 0.0f;
    c1Sum = c1Max = wldMax = jetMax = 0.0f;
    ftMax = dmawSum = 0.0f;
    fpsMin = 0.0f;
    haveWindow = false;
  }
  void frame(const StageTap& tap, uint32_t frameUs) {
    if (!on) return;
    ++frames;
    if (first) {
      first = false;
      startMs = (float)frameUs * 0.001f;
      c1Sum += tap.curCore;
      return;
    }
    c1Sum += tap.curCore;
    dmawSum += tap.cur[kj::ST_DMAWAIT];
    if (tap.curCore > c1Max) c1Max = tap.curCore;
    if (tap.cur[kj::ST_WORLD] > wldMax) wldMax = tap.cur[kj::ST_WORLD];
    if (tap.cur[kj::ST_JET] > jetMax) jetMax = tap.cur[kj::ST_JET];
    const float ft = (float)frameUs * 0.001f;
    if (ft > ftMax) ftMax = ft;
    if (frameUs > MISS60_US) ++over60;
    if (frameUs > MISS45_US) ++over45;
  }
  void window(float fps) {  // one closed 1 s profiler window inside the bench
    if (!on) return;
    if (!haveWindow || fps < fpsMin) fpsMin = fps;
    haveWindow = true;
  }
  void finishIfDue(uint32_t now) {
    if (!on || now - t0 < BENCH_US) return;
    on = false;
    const float secs = (float)(now - t0) * 1e-6f;
    const float avg = secs > 0.0f ? (float)frames / secs : 0.0f;
    const float fn = frames ? (float)frames : 1.0f;
    plat::detail::logf(
        "[BENCH] perframe frames=%lu fps_min=%.1f avg=%.1f start_ms=%.1f ft_max=%.1f over17=%lu "
        "over22=%lu late=%lu c1_avg=%.2f c1_max=%.2f dmaw_avg=%.2f wld_max=%.2f jet_max=%.2f",
        (unsigned long)frames, (double)(haveWindow ? fpsMin : avg), (double)avg, (double)startMs,
        (double)ftMax, (unsigned long)over60, (unsigned long)over45, (unsigned long)late,
        (double)(c1Sum / fn), (double)c1Max, (double)(dmawSum / fn), (double)wldMax, (double)jetMax);
  }
};
Bench g_bench;

// The input poll (core 0, ~1 ms) is kicked when <= KICK_LEAD_US remain, i.e. 1.5-2.6 ms before
// the frame starts given the 1 ms sleep steps, so it has finished when pollInput runs.
constexpr int32_t KICK_LEAD_US = 2600;

void pace() {
#if JET_FPS_CAP > 0
  constexpr uint32_t PERIOD = (JET_FPS_CAP == 60) ? kj::cfg::FRAME_US : 1000000u / JET_FPS_CAP;
  g_nextUs += PERIOD;
  int32_t rem = (int32_t)(g_nextUs - plat::micros32());
  if (rem < 0) {  // late: start the next frame now, no catch-up burst
    if (g_bench.on && g_bench.frames > 1) ++g_bench.late;  // the run-start frame is reported as start_ms
    g_nextUs = plat::micros32();
    plat::inputKick();
    return;
  }
  bool kicked = false;
  while (rem > 1500) {  // sleep in 1 tick (1 ms) steps: lets IDLE1 run and feed the watchdog
    if (!kicked && rem <= KICK_LEAD_US) {
      plat::inputKick();
      kicked = true;
    }
    vTaskDelay(1);
    rem = (int32_t)(g_nextUs - plat::micros32());
  }
  if (!kicked) plat::inputKick();
  while (rem > 0) rem = (int32_t)(g_nextUs - plat::micros32());  // final spin (< 1.5 ms)
#else
  // Uncapped (bench env): poll input for the next frame now; yield one tick every 16 frames so
  // IDLE1 / the task WDT stay happy.
  plat::inputKick();
  if ((g_frameNo & 15u) == 0) vTaskDelay(1);
#endif
}

}  // namespace

void setup() {
  if (!plat::begin(g_persist)) plat::detail::displayHaltRed("no DMA buffer");

  const kj::PlatformApi api = {plat::micros32, plat::logLine, plat::savePersist};
  g_game.init(api, g_persist, plat::detail::WORLD_SEED, plat::imuPresent());
  // Make sure the profiler runs on the device clock (harmless if Game::init already did it).
  g_game.profiler().init(plat::micros32);
  plat::detail::logMem("game");
  plat::applySettings(g_game);

  // JET_DEBUG: Game::init already turns the core IMU stream on (injecting 'i' would toggle it
  // back off); key echo is on by default in debug.
#if JET_BOOT_BENCH
  g_bootCmds[g_bootN++] = 'b';  // 20 s autopilot bench right after boot
#endif

  g_in = kj::InputFrame{};
  g_lastEndUs = plat::micros32();
  g_nextUs = g_lastEndUs;
  g_lastSlowWarnUs = g_lastEndUs;
}

void loop() {
  Profiler& prof = g_game.profiler();

  plat::pollInput(g_in, g_game);
  if (g_in.serialCmd == 0 && g_bootI < g_bootN) g_in.serialCmd = g_bootCmds[g_bootI++];
  if (g_in.serialCmd == 'b') g_bench.start(plat::micros32());

  g_game.update(g_in);
  g_game.render();
  plat::audioPump(g_game.synth(), prof);
  plat::present(g_game.frame().px, g_game.palette().lut2x(), prof);

  const uint32_t steps = prof.stats().steps;  // this frame's march steps (before endFrame)
  const uint32_t now = plat::micros32();
  const uint32_t frameUs = now - g_lastEndUs;
  g_lastEndUs = now;
  prof.endFrame(frameUs);
  g_tap.update(prof);
  ++g_frameNo;

  if (frameUs > SLOW_US && g_frameNo > 3 && now - g_lastSlowWarnUs >= 1000000u) {
    g_lastSlowWarnUs = now;
    plat::detail::logf("[WARN] slow_frame ms=%.1f stage=%s steps=%lu", (double)frameUs / 1000.0,
                       kj::STAGE_NAMES[g_tap.worstStage()], (unsigned long)steps);
  }
  g_bench.frame(g_tap, frameUs);

  plat::applySettings(g_game);

  const bool windowClosing = prof.windowReady();
  if (windowClosing) g_bench.window(prof.fps());
  plat::periodicLogs(g_game, g_in.dt);  // [STAT] + profiler reset when the window is ready
  if (windowClosing) g_tap.reset();
  g_bench.finishIfDue(plat::micros32());
  plat::logFlush();  // move queued log bytes to USB (non-blocking)

  pace();
}
