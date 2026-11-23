// sim_main.cpp - MOONSKIM PC simulator: CLI, deterministic frame loop, PlatformApi
// implementation (QPC clock, stdout log, INI persist), shots/sheets/WAV, timing tables,
// ESP32 cost estimate, self-test. OWNER: group E (DESIGN.md section 18).
//
// Frame (identical order to the device main.cpp):
//   script pre-phase + synthetic IMU (ST_INPUT) -> game.update -> game.render
//   -> synth 384-sample blocks (ST_AUDIO) -> expand2x into the 240x135 RGB565 panel (ST_EXPAND)
//   -> profiler.endFrame -> script post-phase (shots, sheets, asserts)
// Fixed dt = 1/60 s, so a run is fully deterministic (only the PC timings vary).
#ifndef _CRT_SECURE_NO_WARNINGS
#define _CRT_SECURE_NO_WARNINGS
#endif
#include <math.h>
#include <stdarg.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <filesystem>
#include <string>
#include <system_error>
#include <vector>

#include "compat.h"
#include "config.h"
#include "expand.h"
#include "frame.h"
#include "game.h"
#include "jet_model.h"
#include "mathx.h"
#include "palette.h"
#include "perf.h"
#include "render_jet.h"
#include "render_world.h"
#include "tilt.h"
#include "world.h"

#include "png_write.h"
#include "script.h"
#include "synth_imu.h"
#include "wav_write.h"

// Windows last, so its macros cannot touch the core headers.
#ifdef _WIN32
#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#ifndef NOMINMAX
#define NOMINMAX
#endif
#ifndef NOGDI
#define NOGDI
#endif
#include <windows.h>
#else
#include <chrono>
#endif

namespace fs = std::filesystem;
using kj::Stage;

namespace {

// ================================================================== constants
constexpr float SIM_DT = 1.0f / 60.0f;
constexpr int PANEL_W = kj::cfg::SCREEN_W, PANEL_H = kj::cfg::SCREEN_H;
constexpr int PANEL_PIX = PANEL_W * PANEL_H;
constexpr uint32_t DEFAULT_SEED = 0x4D4F4F4Eu;   // same as the device ("MOON")
constexpr int AUDIO_DUE_UNIT = kj::cfg::AUDIO_BLOCK * 60;  // one block, in 1/60-sample units
// ESP32 estimate of the stages the counters do not cover, measured on the device (cardputer-adv
// [STAT] averages, Ring Run autopilot bench): in 0.03 (snapshot of the core-0 input task),
// sim 0.80, pal 0.06, sky 0.55 (night; 0.45 day), obj 0.45 (rings + beacons), hud 0.30,
// aud 0.04, exp 0.03 (staging copy; expand2x runs on core 0), plus the fx stage's fixed part
// 0.28 (trail / wake / speed-line bookkeeping) that prt / lp do not explain.
constexpr float ESP_FIXED_MS = 0.03f + 0.80f + 0.06f + 0.55f + 0.45f + 0.30f + 0.04f + 0.03f + 0.28f;
// Display-bound floor: the core-0 present task pushes a frame every ~13.4 ms at 40 MHz (two
// 32 KB DMA halves back to back, fully overlapped with core 1; device lat= field).
constexpr float DMA_PUSH_MS = 13.4f;
constexpr int NCOUNT = 8;
const char* const COUNTER_NAMES[NCOUNT] = {"stp", "smp", "fill", "vox", "vw", "fl", "prt", "lp"};
constexpr int NESP = 5;  // wld jet fx total c1
const char* const ESP_NAMES[NESP] = {"wld", "jet", "fx", "total", "c1"};

// ================================================================== platform services
#ifdef _WIN32
LARGE_INTEGER g_qpcFreq, g_qpc0;
void clockInit() {
  QueryPerformanceFrequency(&g_qpcFreq);
  QueryPerformanceCounter(&g_qpc0);
}
uint32_t simMicros() {
  LARGE_INTEGER t;
  QueryPerformanceCounter(&t);
  const uint64_t d = (uint64_t)(t.QuadPart - g_qpc0.QuadPart);
  return (uint32_t)(d * 1000000ull / (uint64_t)g_qpcFreq.QuadPart);
}
std::string exeDir(const char* argv0) {
  char buf[4096];
  DWORD n = GetModuleFileNameA(nullptr, buf, (DWORD)sizeof(buf));
  if (n > 0 && n < sizeof(buf)) return fs::path(buf).parent_path().string();
  return fs::absolute(fs::path(argv0)).parent_path().string();
}
#else
std::chrono::steady_clock::time_point g_t0;
void clockInit() { g_t0 = std::chrono::steady_clock::now(); }
uint32_t simMicros() {
  return (uint32_t)std::chrono::duration_cast<std::chrono::microseconds>(
             std::chrono::steady_clock::now() - g_t0)
      .count();
}
std::string exeDir(const char* argv0) { return fs::absolute(fs::path(argv0)).parent_path().string(); }
#endif

void simLog(const char* line) {
  fputs(line, stdout);
  fputc('\n', stdout);
}

void say(const char* fmt, ...)
#if defined(__GNUC__)
    __attribute__((format(printf, 1, 2)))
#endif
    ;
void say(const char* fmt, ...) {
  char buf[2048];
  va_list ap;
  va_start(ap, fmt);
  vsnprintf(buf, sizeof(buf), fmt, ap);
  va_end(ap);
  simLog(buf);
}

std::string g_persistPath;

void simSave(const kj::Persist& p) {
  if (g_persistPath.empty()) return;
  FILE* f = fopen(g_persistPath.c_str(), "w");
  if (!f) {
    say("[SIM] cannot write %s", g_persistPath.c_str());
    return;
  }
  fprintf(f, "; MOONSKIM simulator persist (DESIGN.md section 18)\n");
  fprintf(f, "version=%u\nbest=%u\nbestcombo=%u\ntod=%u\ntilt=%u\nsens=%u\ninvert=%u\n", p.version,
          p.best, p.bestCombo, p.tod, p.tiltOn, p.sens, p.invert);
  fprintf(f, "assist=%u\nvolume=%u\nbright=%u\nview=%u\nmute=%u\ndrift=%u\n", p.assist, p.volume,
          p.bright, p.view, p.mute, p.drift);
  fprintf(f, "rollsign=%d\npitchsign=%d\nnx=%.6f\nny=%.6f\nnz=%.6f\n", p.rollSign, p.pitchSign,
          (double)p.nx, (double)p.ny, (double)p.nz);
  fclose(f);
  say("[SIM] persist saved -> %s (best=%u)", g_persistPath.c_str(), p.best);
}

bool applyPersistKey(kj::Persist& p, const std::string& k, double v) {
  const int iv = (int)v;
  if (k == "version") p.version = (uint32_t)iv;
  else if (k == "best") p.best = (uint32_t)iv;
  else if (k == "bestcombo") p.bestCombo = (uint8_t)iv;
  else if (k == "tod") p.tod = (uint8_t)iv;
  else if (k == "tilt") p.tiltOn = (uint8_t)iv;
  else if (k == "sens") p.sens = (uint8_t)iv;
  else if (k == "invert") p.invert = (uint8_t)iv;
  else if (k == "assist") p.assist = (uint8_t)iv;
  else if (k == "volume") p.volume = (uint8_t)iv;
  else if (k == "bright") p.bright = (uint8_t)iv;
  else if (k == "view") p.view = (uint8_t)iv;
  else if (k == "mute") p.mute = (uint8_t)iv;
  else if (k == "drift") p.drift = (uint8_t)iv;
  else if (k == "rollsign") p.rollSign = (int8_t)(iv < 0 ? -1 : 1);
  else if (k == "pitchsign") p.pitchSign = (int8_t)(iv < 0 ? -1 : 1);
  else if (k == "nx") p.nx = (float)v;
  else if (k == "ny") p.ny = (float)v;
  else if (k == "nz") p.nz = (float)v;
  else return false;
  return true;
}

bool loadPersistIni(const std::string& path, kj::Persist& p) {
  FILE* f = fopen(path.c_str(), "r");
  if (!f) return false;
  char line[256];
  while (fgets(line, sizeof(line), f)) {
    if (line[0] == ';' || line[0] == '#') continue;
    char* eq = strchr(line, '=');
    if (!eq) continue;
    *eq = '\0';
    applyPersistKey(p, line, atof(eq + 1));
  }
  fclose(f);
  if (p.version != kj::PERSIST_VERSION) {
    p = kj::defaultPersist();
    return false;
  }
  return true;
}

// ================================================================== options
struct Options {
  std::string script;
  std::string out;
  int scale = 2;
  bool haveSeed = false;
  uint32_t seed = DEFAULT_SEED;
  bool wav = false, model = false, selftest = false, hash = false, quiet = false;
  bool loadPersist = false;
  int bench = 0;
  int maxFrames = 60 * 600;
};

void usage() {
  printf(
      "MOONSKIM simulator (DESIGN.md section 18)\n"
      "usage: jetsim <script.jsc> [options]\n"
      "       jetsim --selftest | --model | --bench N   (script optional for these)\n"
      "options:\n"
      "  --out DIR        output root (default: the folder of jetsim.exe = tools/sim/out);\n"
      "                   shots go to DIR/<script>/NAME.png, persist to DIR/persist.ini\n"
      "  --scale 1|2|4    shot scale relative to the 240x135 panel (default 2 -> 480x270)\n"
      "  --seed N         world seed (default 0x4D4F4F4E, same as the device)\n"
      "  --wav            write DIR/<script>/audio.wav (16 kHz mono)\n"
      "  --hash           print an FNV-1a hash per shot and a run hash ([HASH] lines)\n"
      "  --bench N        run N extra frames after the script (or a built-in autopilot ring\n"
      "                   run without a script) and print the timing table + esp_est\n"
      "  --model          dump the jet model (JetModel::dumpAscii)\n"
      "  --selftest       expand2x, PNG round trip, WAV, tilt curve, synthetic IMU, depthQ\n"
      "  --load-persist   start from DIR/persist.ini instead of defaultPersist()\n"
      "  --max-frames N   safety cap on frames (default 36000 = 10 min)\n"
      "  --quiet          no per-second [STAT] lines\n"
      "script grammar: see tools/sim/script.h\n"
      "exit codes: 0 ok, 1 assert/self-test/runtime failure, 2 usage/parse/file error\n");
}

bool parseArgs(int argc, char** argv, Options& o) {
  for (int i = 1; i < argc; ++i) {
    std::string a = argv[i];
    auto next = [&](const char* what) -> const char* {
      if (i + 1 >= argc) {
        fprintf(stderr, "missing value for %s\n", what);
        return nullptr;
      }
      return argv[++i];
    };
    if (a == "--out") {
      const char* v = next("--out");
      if (!v) return false;
      o.out = v;
    } else if (a == "--scale") {
      const char* v = next("--scale");
      if (!v) return false;
      o.scale = atoi(v);
      if (o.scale != 1 && o.scale != 2 && o.scale != 4) {
        fprintf(stderr, "--scale must be 1, 2 or 4\n");
        return false;
      }
    } else if (a == "--seed") {
      const char* v = next("--seed");
      if (!v) return false;
      char* end = nullptr;
      unsigned long long s = strtoull(v, &end, 0);
      if (!end || *end || s > 0xFFFFFFFFull) {
        fprintf(stderr, "bad --seed '%s'\n", v);
        return false;
      }
      o.seed = (uint32_t)s;
      o.haveSeed = true;
    } else if (a == "--bench") {
      const char* v = next("--bench");
      if (!v) return false;
      o.bench = atoi(v);
      if (o.bench < 1) {
        fprintf(stderr, "--bench needs a frame count >= 1\n");
        return false;
      }
    } else if (a == "--max-frames") {
      const char* v = next("--max-frames");
      if (!v) return false;
      o.maxFrames = atoi(v);
      if (o.maxFrames < 1) return false;
    } else if (a == "--wav") {
      o.wav = true;
    } else if (a == "--model") {
      o.model = true;
    } else if (a == "--selftest") {
      o.selftest = true;
    } else if (a == "--hash") {
      o.hash = true;
    } else if (a == "--quiet") {
      o.quiet = true;
    } else if (a == "--load-persist") {
      o.loadPersist = true;
    } else if (a == "--help" || a == "-h" || a == "/?") {
      return false;
    } else if (!a.empty() && a[0] == '-') {
      fprintf(stderr, "unknown option %s\n", a.c_str());
      return false;
    } else if (o.script.empty()) {
      o.script = a;
    } else {
      fprintf(stderr, "only one script per run (got %s and %s)\n", o.script.c_str(), a.c_str());
      return false;
    }
  }
  return !o.script.empty() || o.selftest || o.model || o.bench > 0;
}

bool makeDirs(const std::string& d) {
  std::error_code ec;
  fs::create_directories(fs::path(d), ec);
  return fs::is_directory(fs::path(d), ec);
}

// ================================================================== statistics
struct FrameStats {
  float st[kj::ST_COUNT];  // PC ms per stage
  float c1;                // PC ms, all stages except dmaw
  uint32_t cnt[NCOUNT];
  float esp[NESP];         // ESP32 estimate (ms): wld jet fx total c1
  float fpsEst;
  int empty;
  float pcFrameMs;
};

void countersOf(const kj::RenderStats& r, uint32_t c[NCOUNT]) {
  c[0] = r.steps; c[1] = r.samples; c[2] = r.fills; c[3] = r.voxels;
  c[4] = r.voxWrites; c[5] = r.flamePx; c[6] = r.particles; c[7] = r.linePx;
}

void espOf(const uint32_t c[NCOUNT], float esp[NESP], float& fpsEst) {
  const kj::CostModel cm;
  const float k = 1.0f / (cm.cyclesPerUs * 1000.0f);
  esp[0] = ((float)c[0] * cm.cStep + (float)c[1] * cm.cSample + (float)c[2] * cm.cFill) * k;
  // Jet weights for the culled stamp-splat renderer (render_jet.cpp), fitted to the device
  // [STAT] jet stage (which also holds the per-frame setup, outline, nav lights and the
  // cloud-depth push around the jet): a drawn voxel ~421 cycles including its share of the
  // per-frame work, a stamp pixel write ~38, a flame pixel ~97. A shadow sample is counted as
  // 1/2 voxel by renderJetShadow. (Same values as CostModel::cVoxel / cVoxWrite / cFlame.)
  constexpr float JET_C_VOXEL = 421.0f, JET_C_VOXWRITE = 38.5f;
  esp[1] = ((float)c[3] * JET_C_VOXEL + (float)c[4] * JET_C_VOXWRITE + (float)c[5] * cm.cFlame) * k;
  esp[2] = ((float)c[6] * cm.cParticle + (float)c[7] * cm.cLinePx) * k;
  esp[3] = esp[0] + esp[1] + esp[2];  // == Profiler::estEspMs for one frame
  esp[4] = esp[3] + ESP_FIXED_MS;
  float period = esp[4] > DMA_PUSH_MS ? esp[4] : DMA_PUSH_MS;
#if JET_FPS_CAP > 0
  const float capMs = 1000.0f / (float)JET_FPS_CAP;
  if (period < capMs) period = capMs;
#endif
  fpsEst = 1000.0f / period;
}

struct Acc {
  int frames = 0;
  double sumSt[kj::ST_COUNT] = {};
  float maxSt[kj::ST_COUNT] = {};
  double sumC1 = 0.0, sumPc = 0.0;
  float maxC1 = 0.0f;
  double sumCnt[NCOUNT] = {};
  uint32_t maxCnt[NCOUNT] = {};
  double sumEsp[NESP] = {};
  float maxEsp[NESP] = {};
  double sumFps = 0.0;
  float minFps = 1e9f;
  int maxEmpty = 0;
  void clear() { *this = Acc(); }
  void add(const FrameStats& f) {
    ++frames;
    for (int i = 0; i < kj::ST_COUNT; ++i) {
      sumSt[i] += f.st[i];
      if (f.st[i] > maxSt[i]) maxSt[i] = f.st[i];
    }
    sumC1 += f.c1;
    sumPc += f.pcFrameMs;
    if (f.c1 > maxC1) maxC1 = f.c1;
    for (int i = 0; i < NCOUNT; ++i) {
      sumCnt[i] += f.cnt[i];
      if (f.cnt[i] > maxCnt[i]) maxCnt[i] = f.cnt[i];
    }
    for (int i = 0; i < NESP; ++i) {
      sumEsp[i] += f.esp[i];
      if (f.esp[i] > maxEsp[i]) maxEsp[i] = f.esp[i];
    }
    sumFps += f.fpsEst;
    if (f.fpsEst < minFps) minFps = f.fpsEst;
    if (f.empty > maxEmpty) maxEmpty = f.empty;
  }
  double avg(double s) const { return frames ? s / frames : 0.0; }
};

void printTable(const char* tag, const Acc& a) {
  if (a.frames == 0) {
    say("[%s] no frames", tag);
    return;
  }
  const double pcfps = a.sumPc > 0.0 ? 1000.0 * a.frames / a.sumPc : 0.0;
  say("[%s] frames=%d sim_s=%.2f pc_fps=%.0f", tag, a.frames, a.frames / 60.0, pcfps);
  say("[%s] stage   pc_avg_ms  pc_max_ms", tag);
  for (int i = 0; i < kj::ST_COUNT; ++i)
    say("[%s] %-6s  %9.3f  %9.3f", tag, kj::STAGE_NAMES[i], a.avg(a.sumSt[i]), (double)a.maxSt[i]);
  say("[%s] %-6s  %9.3f  %9.3f", tag, "c1", a.avg(a.sumC1), (double)a.maxC1);
  char buf[512];
  int p = snprintf(buf, sizeof(buf), "[%s] counters avg/max:", tag);
  for (int i = 0; i < NCOUNT && p > 0 && p < (int)sizeof(buf); ++i)
    p += snprintf(buf + p, sizeof(buf) - (size_t)p, " %s=%.0f/%u", COUNTER_NAMES[i], a.avg(a.sumCnt[i]),
                  a.maxCnt[i]);
  simLog(buf);
  p = snprintf(buf, sizeof(buf), "[%s] esp_est_ms avg/max:", tag);
  for (int i = 0; i < NESP && p > 0 && p < (int)sizeof(buf); ++i)
    p += snprintf(buf + p, sizeof(buf) - (size_t)p, " %s=%.2f/%.2f", ESP_NAMES[i], a.avg(a.sumEsp[i]),
                  (double)a.maxEsp[i]);
  simLog(buf);
  say("[%s] fps_est min/avg=%.1f/%.1f (c1 = counters + %.2f ms fixed, floor %.2f ms DMA, cap %d)"
       " empty_px_max=%d",
       tag, (double)a.minFps, a.avg(a.sumFps), (double)ESP_FIXED_MS, (double)DMA_PUSH_MS, JET_FPS_CAP,
       a.maxEmpty);
}

// ================================================================== the game (one static global)
kj::Game g_game;
kj::JetModel g_model;   // --model and studio frames (the game's own model is private)
bool g_modelBuilt = false;

bool ensureModel() {
  if (!g_modelBuilt) g_modelBuilt = g_model.build();
  return g_modelBuilt;
}

// ================================================================== simulator
class Sim : public sim::SimHost {
 public:
  Options opt;
  sim::Script script;
  sim::Scheduler sched;
  sim::SynthImu imu;
  sim::WavWriter wav;
  std::string outRoot, outDir;
  std::vector<uint16_t> panel = std::vector<uint16_t>((size_t)PANEL_PIX + 8u);
  std::vector<sim::Image> sheetTiles;
  std::vector<std::string> sheetNames;
  int frame = 0;
  bool imuPresent = true;
  uint32_t seed = DEFAULT_SEED;
  // studio frames (jetpose)
  bool studio = false;
  float studioYaw = 0.0f, studioRoll = 0.0f, studioPitch = 0.0f, studioBoost = 0.0f;
  // statistics
  FrameStats last = {};
  Acc run, window, bench, track;
  bool benchOn = false;
  int winFrames = 0;
  double prevSumMs[kj::ST_COUNT] = {};
  int crashes = 0, prevState = -1;
  uint32_t runHash = 2166136261u;
  int shots = 0;
  int audioDue = 0;
  FILE* csv = nullptr;
  bool fileError = false;

  ~Sim() {
    if (csv) fclose(csv);
  }

  // ---------------------------------------------------------------- SimHost
  int gameState() const override { return (int)g_game.telemetry().state; }
  void log(const char* line) override { simLog(line); }

  void benchBegin(int frames) override {
    bench.clear();
    benchOn = true;
    say("[BENCH] begin frames=%d t=%.2f", frames, frame / 60.0);
  }
  void benchEnd() override {
    benchOn = false;
    printTable("BENCH", bench);
  }

  bool getVar(const char* name, float& out) const override {
    const kj::Telemetry tm = g_game.telemetry();
    const kj::JetState& j = g_game.jet();
    const std::string n = name;
    auto cnt = [&](int i) { return (float)last.cnt[i]; };
    if (n == "t" || n == "time") out = (float)(frame / 60.0);
    else if (n == "frame") out = (float)frame;
    else if (n == "state") out = (float)(int)tm.state;
    else if (n == "mode") out = (float)(int)tm.mode;
    else if (n == "crashes") out = (float)crashes;
    else if (n == "empty_px") out = (float)last.empty;
    else if (n == "fps_est") out = last.fpsEst;
    else if (n == "esp_wld") out = last.esp[0];
    else if (n == "esp_jet") out = last.esp[1];
    else if (n == "esp_fx") out = last.esp[2];
    else if (n == "esp_est") out = last.esp[3];
    else if (n == "c1_est") out = last.esp[4];
    else if (n == "steps") out = cnt(0);
    else if (n == "samples") out = cnt(1);
    else if (n == "fills") out = cnt(2);
    else if (n == "voxels") out = cnt(3);
    else if (n == "vox_writes") out = cnt(4);
    else if (n == "flame_px") out = cnt(5);
    else if (n == "particles") out = cnt(6);
    else if (n == "line_px") out = cnt(7);
    else if (n == "max_steps") out = (float)track.maxCnt[0];
    else if (n == "max_samples") out = (float)track.maxCnt[1];
    else if (n == "max_fills") out = (float)track.maxCnt[2];
    else if (n == "max_voxels") out = (float)track.maxCnt[3];
    else if (n == "max_vox_writes") out = (float)track.maxCnt[4];
    else if (n == "max_flame_px") out = (float)track.maxCnt[5];
    else if (n == "max_particles") out = (float)track.maxCnt[6];
    else if (n == "max_line_px") out = (float)track.maxCnt[7];
    else if (n == "max_esp_wld") out = track.maxEsp[0];
    else if (n == "max_esp_jet") out = track.maxEsp[1];
    else if (n == "max_esp_fx") out = track.maxEsp[2];
    else if (n == "max_esp_est") out = track.maxEsp[3];
    else if (n == "max_c1_est") out = track.maxEsp[4];
    else if (n == "max_empty_px") out = (float)track.maxEmpty;
    else if (n == "min_fps_est") out = track.frames ? track.minFps : 0.0f;
    else if (n == "turn") out = tm.turn;
    else if (n == "climb") out = tm.climb;
    else if (n == "tilt_roll") out = tm.tiltRoll;
    else if (n == "tilt_pitch") out = tm.tiltPitch;
    else if (n == "tilt_valid") out = tm.tiltValid ? 1.0f : 0.0f;
    else if (n == "tilt_on") out = tm.tiltOn ? 1.0f : 0.0f;
    else if (n == "calibrated") out = tm.calibrated ? 1.0f : 0.0f;
    else if (n == "x") out = j.pos.x;
    else if (n == "y") out = j.pos.y;
    else if (n == "z") out = j.pos.z;
    else if (n == "vz") out = j.vel.z;
    else if (n == "yaw") out = j.yaw * kj::RAD2DEG;
    else if (n == "yaw_rate") out = j.yawRate;
    else if (n == "pitch") out = j.pitch;
    else if (n == "bank") out = j.bank;
    else if (n == "roll_angle") out = j.rollAngle;
    else if (n == "g") out = j.g;
    else if (n == "boost_energy") out = j.boostEnergy;
    else if (n == "invuln") out = j.invuln;
    else if (n == "in_cloud") out = g_game.world().insideCloud(j.pos.x, j.pos.y, j.pos.z) ? 1.0f : 0.0f;
    else if (n == "cloud_thick") out = (float)g_game.world().cloudThick(j.pos.x, j.pos.y);
    else if (n == "imu_roll") out = imu.roll();
    else if (n == "imu_pitch") out = imu.pitch();
    else if (n == "imu_hold") out = imu.hold();
    else return g_game.debugVar(name, out);  // alt agl speed score combo crashed rings horizon
                                              // boost time_left jet_w (+ any the game adds)
    return true;
  }

  bool execCommand(const sim::Cmd& c, std::string& err) override {
    using sim::CmdType;
    const kj::JetState& j = g_game.jet();
    switch (c.type) {
      case CmdType::ImuNoise: imu.setNoise(c.i0 != 0); return true;
      case CmdType::Tod: g_game.setTod((kj::Tod)c.i0); return true;
      case CmdType::View: g_game.setView(c.i0); return true;
      case CmdType::State:
        g_game.debugStart(c.i0 == 1 ? kj::Mode::RingRun : kj::Mode::Free);
        return true;
      case CmdType::Tilt: imu.setTilt(c.f[0], c.f[1], c.nf >= 3 ? c.f[2] : 0.0f); return true;
      case CmdType::Pose: imu.setHoldPose(c.f[0], c.nf >= 2 ? c.f[1] : 0.0f); return true;
      case CmdType::Teleport: {
        const float x = c.rel[0] ? j.pos.x + c.f[0] : c.f[0];
        const float y = c.rel[1] ? j.pos.y + c.f[1] : c.f[1];
        const float z = c.rel[2] ? j.pos.z + c.f[2] : c.f[2];
        const float yawDeg = c.rel[3] ? j.yaw * kj::RAD2DEG + c.f[3] : c.f[3];
        teleport(x, y, z, yawDeg * kj::DEG);
        return true;
      }
      case CmdType::Autopilot: g_game.setAutopilot(c.i0 != 0); return true;
      case CmdType::Spawn: {
        float x = 0.0f, y = 0.0f, yaw = 0.0f;
        g_game.world().findSpawn((uint32_t)c.i0, x, y, yaw);
        teleport(x, y, c.f[0], yaw);
        return true;
      }
      case CmdType::Seek: return seek(c, err);
      case CmdType::JetPose:
        studio = c.i0 != 0;
        if (studio) {
          studioYaw = c.f[0];
          studioRoll = c.f[1];
          studioPitch = c.f[2];
          studioBoost = c.nf >= 4 ? kj::sat(c.f[3]) : 0.0f;
          if (!ensureModel()) {
            err = "jetpose: JetModel::build() failed";
            return false;
          }
        }
        return true;
      case CmdType::ResetMax: track.clear(); return true;
      case CmdType::Echo: say("[ECHO] t=%.3f %s", frame / 60.0, c.s0.c_str()); return true;
      case CmdType::Shot: return shot(c.s0, err);
      case CmdType::Sheet: return sheet(c.s0, c.i0, err);
      default:
        err = "command not handled by the simulator host";
        return false;
    }
  }

  // ---------------------------------------------------------------- helpers
  void teleport(float x, float y, float z, float yawRad) {
    g_game.debugTeleport(kj::wrapPos(x), kj::wrapPos(y), z, kj::wrapPi(yawRad));
    say("[SIM] teleport x=%.1f y=%.1f z=%.1f yaw=%.1f", (double)kj::wrapPos(x), (double)kj::wrapPos(y),
         (double)z, (double)(kj::wrapPi(yawRad) * kj::RAD2DEG));
  }

  static float groundAt(const kj::World& w, float x, float y) {
    const int cx = (int)floorf(kj::wrapPos(x) / kj::cfg::CELL_U);
    const int cy = (int)floorf(kj::wrapPos(y) / kj::cfg::CELL_U);
    const uint8_t cell = w.cell(cx, cy);
    if (kj::cellIsWater(cell)) return -1.0f;
    return (float)kj::cfg::HEIGHT_U[kj::cellLevel(cell)];
  }

  bool seek(const sim::Cmd& c, std::string& err) {
    const kj::World& w = g_game.world();
    const kj::JetState& j = g_game.jet();
    const kj::Vec3 fwd = kj::forwardOf(j.yaw);
    const kj::Vec3 rgt = kj::rightOf(j.yaw);
    const float px = j.pos.x, py = j.pos.y;
    if (c.s0 == "land") {
      const float R = 1600.0f, STEP = 8.0f;
      bool found = false;
      float best = -1e30f, bx = 0.0f, by = 0.0f, bh = 0.0f;
      for (float dy = -R; dy <= R; dy += STEP)
        for (float dx = -R; dx <= R; dx += STEP) {
          const float d2 = dx * dx + dy * dy;
          if (d2 > R * R) continue;
          const float h = groundAt(w, px + dx, py + dy);
          if (h < 0.0f) continue;
          float score = h * 1000.0f - sqrtf(d2);
          if (c.i0 && w.cloudThick(px + dx, py + dy) > 0) score += 1e6f;
          if (score > best) {
            best = score;
            bx = px + dx;
            by = py + dy;
            bh = h;
            found = true;
          }
        }
      if (!found) {
        err = "seek land: no land within 1600 u";
        return false;
      }
      say("[SIM] seek land: peak h=%.0f at (%.0f,%.0f)", (double)bh, (double)kj::wrapPos(bx),
           (double)kj::wrapPos(by));
      teleport(bx - fwd.x * c.f[1], by - fwd.y * c.f[1], c.f[0], j.yaw);
      return true;
    }
    if (c.s0 == "cloud") {
      const float R = 800.0f, STEP = 8.0f;
      bool found = false;
      float best = -1e30f, bx = 0.0f, by = 0.0f;
      int bt = 0;
      for (float dy = -R; dy <= R; dy += STEP)
        for (float dx = -R; dx <= R; dx += STEP) {
          const float d2 = dx * dx + dy * dy;
          if (d2 > R * R) continue;
          const int t = w.cloudThick(px + dx, py + dy);
          if (t <= 0) continue;
          int sum = 0;
          for (int oy = -2; oy <= 2; ++oy)
            for (int ox = -2; ox <= 2; ++ox) sum += w.cloudThick(px + dx + ox * STEP, py + dy + oy * STEP);
          const float score = (float)sum * 100.0f - sqrtf(d2) * 0.01f;
          if (score > best) {
            best = score;
            bx = px + dx;
            by = py + dy;
            bt = t;
            found = true;
          }
        }
      if (!found) {
        err = "seek cloud: no cloud within 800 u";
        return false;
      }
      float z = c.f[0];
      const float base = kj::cfg::CLOUD_BASE, layer = kj::cfg::CLOUD_LAYER_U;
      if (c.s1 == "under") z = base - 18.0f;
      else if (c.s1 == "in") z = base + (float)bt * layer * 0.5f;
      else if (c.s1 == "above") z = base + (float)bt * layer + 14.0f;
      say("[SIM] seek cloud: thick=%d at (%.0f,%.0f)", bt, (double)kj::wrapPos(bx), (double)kj::wrapPos(by));
      teleport(bx - fwd.x * c.f[1], by - fwd.y * c.f[1], z, j.yaw);
      return true;
    }
    // sea: nearest point with open water 0..600 u ahead and +-200 u to the sides
    for (float r = 0.0f; r <= 3000.0f; r += 64.0f) {
      const int n = r <= 0.0f ? 1 : (int)(kj::TAU * r / 64.0f);
      for (int i = 0; i < n; ++i) {
        const float a = kj::TAU * (float)i / (float)n;
        const float cx = px + r * cosf(a), cy = py + r * sinf(a);
        bool open = true;
        for (float d = 0.0f; d <= 600.0f && open; d += 16.0f)
          for (float l = -200.0f; l <= 200.0f && open; l += 40.0f)
            if (groundAt(w, cx + fwd.x * d + rgt.x * l, cy + fwd.y * d + rgt.y * l) >= 0.0f) open = false;
        if (open) {
          teleport(cx, cy, c.f[0], j.yaw);
          return true;
        }
      }
    }
    err = "seek sea: no open water within 3000 u";
    return false;
  }

  void renderStudio() {
    kj::FrameBuf fb = g_game.frame();
    memset(fb.px, kj::pal::EMPTY, (size_t)kj::LPIX);
    memset(fb.depth, kj::DEPTH_FAR, (size_t)kj::LPIX);
    const kj::View& v = g_game.view();
    const kj::World& w = g_game.world();
    const kj::Palette& pal = g_game.palette();
    const kj::Tod tod = pal.tod();
    const float time = (float)(frame / 60.0);
    kj::RenderStats rs;
    rs.clear();
    int mx = 0, my = 0;
    const bool moon = kj::skyMoonScreen(v, tod, mx, my);
    kj::SeaFx sea;
    sea.time = time;
    sea.moonX = moon ? mx : -1000;
    sea.moonPath = moon;
    sea.cloudShadows = tod != kj::Tod::Night;
    kj::renderWorld(fb, v, w, tod, sea, rs);
    kj::renderSky(fb, v, tod, time, 0.0f, rs);  // dt 0: the game already advanced the sky
    const kj::JetState& j = g_game.jet();
    kj::JetPose p;
    p.pos = j.pos;
    p.yaw = kj::wrapPi(j.yaw + studioYaw * kj::DEG);
    p.roll = studioRoll * kj::DEG;
    p.pitch = studioPitch * kj::DEG;
    p.boost = studioBoost;
    p.yawRateN = 0.0f;
    p.climbRateN = 0.0f;
    p.time = time;
    p.assemble = 1.0f;
    p.visible = true;
    kj::renderJetShadow(fb, v, g_model, p, w, pal, tod, &rs);
    kj::Rect bounds = {0, 0, 0, 0};
    kj::renderJet(fb, v, g_model, p, tod, rs, &bounds);
  }

  void panelToImage(sim::Image& img) const {
    img.resize(PANEL_W, PANEL_H);
    for (int i = 0; i < PANEL_PIX; ++i) {
      const uint32_t c = kj::rgb565swToRgb888(panel[(size_t)i]);
      img.rgb[(size_t)i * 3 + 0] = (uint8_t)(c >> 16);
      img.rgb[(size_t)i * 3 + 1] = (uint8_t)(c >> 8);
      img.rgb[(size_t)i * 3 + 2] = (uint8_t)c;
    }
  }

  bool shot(const std::string& name, std::string& err) {
    sim::Image img;
    panelToImage(img);
    const uint32_t h = sim::fnv1a(img.rgb.data(), img.rgb.size());
    const std::string path = (fs::path(outDir) / (name + ".png")).string();
    bool ok;
    if (opt.scale == 1) {
      ok = sim::pngWrite(path.c_str(), img);
    } else {
      sim::Image big;
      sim::scaleNearest(img, opt.scale, big);
      ok = sim::pngWrite(path.c_str(), big);
    }
    if (!ok) {
      err = "shot: cannot write " + path;
      fileError = true;
      return false;
    }
    ++shots;
    uint8_t hb[4] = {(uint8_t)h, (uint8_t)(h >> 8), (uint8_t)(h >> 16), (uint8_t)(h >> 24)};
    runHash = sim::fnv1a(hb, 4, runHash);
    say("[SHOT] %s t=%.3f -> %s (%dx%d)", name.c_str(), frame / 60.0, path.c_str(), PANEL_W * opt.scale,
         PANEL_H * opt.scale);
    if (opt.hash) say("[HASH] shot=%s fnv=%08x", name.c_str(), h);
    sheetTiles.push_back(img);
    sheetNames.push_back(name);
    return true;
  }

  bool sheet(const std::string& name, int cols, std::string& err) {
    sim::Image s;
    if (!sim::makeSheet(sheetTiles, sheetNames, cols, s)) {
      err = "sheet: no shots since the previous sheet";
      return false;
    }
    const std::string path = (fs::path(outDir) / (name + ".png")).string();
    if (!sim::pngWrite(path.c_str(), s)) {
      err = "sheet: cannot write " + path;
      fileError = true;
      return false;
    }
    say("[SHEET] %s tiles=%d cols=%d -> %s (%dx%d)", name.c_str(), (int)sheetTiles.size(), cols, path.c_str(),
         s.w, s.h);
    sheetTiles.clear();
    sheetNames.clear();
    return true;
  }

  // ---------------------------------------------------------------- frame
  void runFrame() {
    kj::Profiler& prof = g_game.profiler();
    const uint32_t f0 = simMicros();

    // ---- input (script pre-phase + synthetic IMU)
    uint32_t t0 = simMicros();
    sim::FrameKeys keys;
    sched.preFrame(frame, *this, keys);
    kj::InputFrame in = {};
    in.dt = SIM_DT;
    in.keysDown = keys.down;
    in.keysPressed = keys.pressed;
    in.btnA = keys.btnA;
    in.btnAPressed = keys.btnAPressed;
    in.imuPresent = imuPresent;
    const uint32_t usec = (uint32_t)((uint64_t)frame * 1000000ull / 60ull);
    if (imuPresent) {
      in.imu = imu.step(SIM_DT, usec);
    } else {
      in.imu = kj::ImuSample{0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, usec, false};
    }
    in.serialCmd = keys.serialCmd;
    in.batteryPct = 87;
    in.batteryMv = 4050;
    prof.add(kj::ST_INPUT, simMicros() - t0);

    // ---- game
    g_game.update(in);
    g_game.render();
    kj::RenderStats rs = prof.stats();
    if (studio) renderStudio();

    FrameStats fsx = {};
    const kj::FrameBuf& fb = g_game.frame();
    for (int i = 0; i < kj::LPIX; ++i)
      if (fb.px[i] == kj::pal::EMPTY) ++fsx.empty;

    // ---- audio: 384-sample blocks at 16 kHz, same block structure as the device pump
    t0 = simMicros();
    audioDue += kj::cfg::AUDIO_RATE;  // per frame, in 1/60-sample units
    while (audioDue >= AUDIO_DUE_UNIT) {
      int16_t blk[kj::cfg::AUDIO_BLOCK];
      g_game.synth().render(blk, kj::cfg::AUDIO_BLOCK);
      if (wav.isOpen()) wav.write(blk, kj::cfg::AUDIO_BLOCK);
      audioDue -= AUDIO_DUE_UNIT;
    }
    prof.add(kj::ST_AUDIO, simMicros() - t0);

    // ---- present: device-exact expand into the panel buffer
    t0 = simMicros();
    kj::expand2x(fb.px, g_game.palette().lut2x(), panel.data());
    prof.add(kj::ST_EXPAND, simMicros() - t0);

    const uint32_t frameUs = simMicros() - f0;
    prof.endFrame(kj::cfg::FRAME_US);  // nominal 60 fps: [STAT] windows are 1 s of sim time
    ++winFrames;

    // ---- per-frame values (Profiler window sums are exact: avgMs * frames)
    fsx.c1 = 0.0f;
    for (int i = 0; i < kj::ST_COUNT; ++i) {
      const double sum = (double)prof.avgMs((Stage)i) * winFrames;
      fsx.st[i] = (float)(sum - prevSumMs[i]);
      if (fsx.st[i] < 0.0f) fsx.st[i] = 0.0f;
      prevSumMs[i] = sum;
      if (i != kj::ST_DMAWAIT) fsx.c1 += fsx.st[i];
    }
    countersOf(rs, fsx.cnt);
    espOf(fsx.cnt, fsx.esp, fsx.fpsEst);
    fsx.pcFrameMs = (float)frameUs / 1000.0f;
    last = fsx;

    const int st = (int)g_game.telemetry().state;
    if (st == (int)kj::GameState::Crash && prevState != (int)kj::GameState::Crash) ++crashes;
    prevState = st;

    run.add(fsx);
    window.add(fsx);
    if (st != (int)kj::GameState::Boot) track.add(fsx);
    if (benchOn) bench.add(fsx);

    if (csv) {
      const kj::JetState& j = g_game.jet();
      fprintf(csv, "%d,%.4f,%s", frame, frame / 60.0, sim::stateName(st));
      for (int i = 0; i < kj::ST_COUNT; ++i) fprintf(csv, ",%.4f", (double)fsx.st[i]);
      fprintf(csv, ",%.4f", (double)fsx.c1);
      for (int i = 0; i < NCOUNT; ++i) fprintf(csv, ",%u", fsx.cnt[i]);
      for (int i = 0; i < NESP; ++i) fprintf(csv, ",%.4f", (double)fsx.esp[i]);
      fprintf(csv, ",%.2f,%d,%.2f,%.2f,%.2f,%.2f,%.2f\n", (double)fsx.fpsEst, fsx.empty, (double)j.pos.x,
              (double)j.pos.y, (double)j.pos.z, (double)(j.yaw * kj::RAD2DEG), (double)j.speed);
    }

    // ---- script post-phase (shots use the panel buffer of this frame)
    sched.postFrame(frame, *this);

    // ---- [STAT] once per simulated second
    if (prof.windowReady()) {
      if (!opt.quiet) {
        char pf[1024];
        prof.format(pf, (int)sizeof(pf));
        const double pcfps = window.sumPc > 0.0 ? 1000.0 * window.frames / window.sumPc : 0.0;
        say("[STAT] t=%d st=%s %s | esp_est=%.2f/%.2f wld=%.2f jet=%.2f fx=%.2f fps_est=%.1f pcfps=%.0f"
             " empty=%d",
             (int)lround((frame + 1) / 60.0), sim::stateName(st), pf, window.avg(window.sumEsp[3]),
             (double)window.maxEsp[3], window.avg(window.sumEsp[0]), window.avg(window.sumEsp[1]),
             window.avg(window.sumEsp[2]), (double)window.minFps, pcfps, window.maxEmpty);
      }
      prof.reset();
      winFrames = 0;
      for (double& s : prevSumMs) s = 0.0;
      window.clear();
    }
    ++frame;
  }

  // ---------------------------------------------------------------- run
  int runScript() {
    outDir = (fs::path(outRoot) / script.name()).string();
    if (!makeDirs(outDir)) {
      say("[SIM] ERROR cannot create output folder %s", outDir.c_str());
      return 2;
    }
    g_persistPath = (fs::path(outRoot) / "persist.ini").string();

    kj::Persist p = kj::defaultPersist();
    if (opt.loadPersist) {
      if (loadPersistIni(g_persistPath, p)) say("[SIM] persist loaded from %s", g_persistPath.c_str());
      else say("[SIM] no valid %s, using defaults", g_persistPath.c_str());
    }
    for (const sim::PersistOverride& o : script.persist()) applyPersistKey(p, o.key, (double)o.value);
    seed = opt.haveSeed ? opt.seed : (script.hasSeed() ? script.seed() : DEFAULT_SEED);
    imuPresent = script.imuPresent();

    const std::string csvPath = (fs::path(outDir) / "frames.csv").string();
    csv = fopen(csvPath.c_str(), "w");
    if (csv) {
      fprintf(csv, "frame,t,state");
      for (int i = 0; i < kj::ST_COUNT; ++i) fprintf(csv, ",%s_ms", kj::STAGE_NAMES[i]);
      fprintf(csv, ",c1_ms");
      for (int i = 0; i < NCOUNT; ++i) fprintf(csv, ",%s", COUNTER_NAMES[i]);
      for (int i = 0; i < NESP; ++i) fprintf(csv, ",esp_%s", ESP_NAMES[i]);
      fprintf(csv, ",fps_est,empty_px,x,y,z,yaw_deg,speed\n");
    }
    if (opt.wav) {
      const std::string wavPath = (fs::path(outDir) / "audio.wav").string();
      if (!wav.open(wavPath.c_str(), kj::cfg::AUDIO_RATE)) {
        say("[SIM] ERROR cannot write %s", wavPath.c_str());
        return 2;
      }
    }

    say("[SIM] script=%s seed=0x%08X imu=%d scale=%d out=%s", script.file().c_str(), seed, imuPresent ? 1 : 0,
         opt.scale, outDir.c_str());
    const kj::PlatformApi api = {simMicros, simLog, simSave};
    const uint32_t ti = simMicros();
    g_game.init(api, p, seed, imuPresent);
    say("[SIM] init_ms=%.1f world_crc=%08x beacons=%d", (simMicros() - ti) / 1000.0, g_game.world().crc(),
         g_game.world().beaconCount());
    g_game.profiler().reset();
    imu.reset(seed ^ 0x1D5EEDu);
    sched.begin(script);

    while (!sched.done() && frame < opt.maxFrames) runFrame();
    if (!sched.done()) say("[SIM] WARNING stopped at --max-frames %d before the script ended", opt.maxFrames);

    if (opt.bench > 0 && !sched.failed()) {
      benchBegin(opt.bench);
      for (int i = 0; i < opt.bench; ++i) runFrame();
      benchEnd();
    }

    if (wav.isOpen()) {
      const uint32_t n = wav.samples();
      const bool ok = wav.close();
      say("[WAV] %s samples=%u (%.2f s) %s", (fs::path(outDir) / "audio.wav").string().c_str(), n,
           n / (double)kj::cfg::AUDIO_RATE, ok ? "ok" : "WRITE ERROR");
      if (!ok) fileError = true;
    }
    if (csv) {
      fclose(csv);
      csv = nullptr;
    }

    printTable("SUMMARY", run);
    say("[SUMMARY] world_crc=%08x seed=0x%08X frames=%d crashes=%d shots=%d asserts=%d failed=%d", g_game.world().crc(),
         seed, frame, crashes, shots, sched.assertsRun(), sched.assertsFailed());
    if (opt.hash) say("[HASH] run=%08x shots=%d world_crc=%08x", runHash, shots, g_game.world().crc());
    say("[SUMMARY] outputs: %s (PNG shots/sheets, frames.csv%s)", outDir.c_str(), opt.wav ? ", audio.wav" : "");
    if (!sheetTiles.empty())
      say("[SUMMARY] note: %d shot(s) after the last sheet were not put on a sheet", (int)sheetTiles.size());

    if (sched.failed() || fileError) return 1;
    if (sched.assertsFailed() > 0) return 1;
    return 0;
  }
};

// ================================================================== --model
int runModel() {
  const bool ok = ensureModel();
  say("[MODEL] build=%s vox=%d", ok ? "ok" : "FAILED", g_model.count());
  if (!ok) return 1;
  g_model.dumpAscii(simLog);
  const int n = g_model.count();
  const bool inRange = n >= 900 && n <= 1300;
  say("[MODEL] voxel count %d %s the 900..1300 target", n, inRange ? "within" : "OUTSIDE");
  return inRange ? 0 : 1;
}

// ================================================================== --selftest
int g_stFail = 0, g_stPass = 0;

void check(bool ok, const char* fmt, ...)
#if defined(__GNUC__)
    __attribute__((format(printf, 2, 3)))
#endif
    ;
void check(bool ok, const char* fmt, ...) {
  char buf[1024];
  va_list ap;
  va_start(ap, fmt);
  vsnprintf(buf, sizeof(buf), fmt, ap);
  va_end(ap);
  say("[SELFTEST] %s %s", ok ? "ok  " : "FAIL", buf);
  if (ok) ++g_stPass;
  else ++g_stFail;
}

uint32_t xs32(uint32_t& s) {
  s ^= s << 13;
  s ^= s >> 17;
  s ^= s << 5;
  return s;
}

void selftestExpand() {
  static uint8_t px[kj::LPIX];
  static uint32_t lut[256];
  static uint16_t colour[256];
  for (int i = 0; i < 256; ++i) {
    colour[i] = kj::rgb565sw(kj::hash32((uint32_t)i * 977u + 1u) & 0xFFFFFFu);
    lut[i] = ((uint32_t)colour[i] << 16) | colour[i];
  }
  for (int y = 0; y < kj::LH; ++y)
    for (int x = 0; x < kj::LW; ++x)
      px[y * kj::LW + x] = (uint8_t)(kj::hash32((uint32_t)(x * 131 + y * 7919)) & 255u);
  for (int x = 0; x < kj::LW; ++x) px[67 * kj::LW + x] = (uint8_t)(200 + x % 50);  // distinct row 67

  const uint16_t SENT = 0xA5C3;
  std::vector<uint16_t> out((size_t)PANEL_PIX + 16u, SENT);
  kj::expand2x(px, lut, out.data());
  int bad = 0;
  for (int oy = 0; oy < PANEL_H; ++oy)
    for (int ox = 0; ox < PANEL_W; ++ox)
      if (out[(size_t)(oy * PANEL_W + ox)] != colour[px[(oy >> 1) * kj::LW + (ox >> 1)]]) ++bad;
  check(bad == 0, "expand2x: every output pixel = lut[px[y/2][x/2]] (%d mismatches)", bad);
  int bad134 = 0;
  for (int ox = 0; ox < PANEL_W; ++ox)
    if (out[(size_t)(134 * PANEL_W + ox)] != colour[px[67 * kj::LW + (ox >> 1)]]) ++bad134;
  check(bad134 == 0, "expand2x: output row 134 = logical row 67 (%d mismatches)", bad134);
  int over = 0;
  for (size_t i = (size_t)PANEL_PIX; i < out.size(); ++i)
    if (out[i] != SENT) ++over;
  check(over == 0, "expand2x: no write past 240x135 (%d words touched)", over);

  // byte order: the panel wants big-endian RGB565 in memory
  static uint8_t px1[kj::LPIX];
  memset(px1, 1, sizeof(px1));
  static uint32_t lut1[256];
  const uint16_t red = kj::rgb565sw(0xFF0000u);
  for (int i = 0; i < 256; ++i) lut1[i] = ((uint32_t)red << 16) | red;
  std::vector<uint16_t> o1((size_t)PANEL_PIX);
  kj::expand2x(px1, lut1, o1.data());
  const uint8_t* b = (const uint8_t*)o1.data();
  check(b[0] == 0xF8 && b[1] == 0x00 && b[2] == 0xF8 && b[3] == 0x00,
        "expand2x: byte order big-endian RGB565 (bytes %02X %02X %02X %02X, want F8 00 F8 00)", b[0], b[1],
        b[2], b[3]);
  check(kj::rgb565swToRgb888(o1[0]) == 0xFF0000u, "rgb565swToRgb888(rgb565sw(red)) = %06X",
        kj::rgb565swToRgb888(o1[0]));

  // band variant (DMA fallback path): 16-row bands + an odd-start band
  std::vector<uint16_t> band((size_t)PANEL_W * 16u + 8u);
  int badBand = 0;
  for (int r0 = 0; r0 < PANEL_H; r0 += 16) {
    const int r1 = r0 + 16 < PANEL_H ? r0 + 16 : PANEL_H;
    kj::expand2xRows(px, lut, band.data(), r0, r1);
    for (int r = r0; r < r1; ++r)
      for (int x = 0; x < PANEL_W; ++x)
        if (band[(size_t)((r - r0) * PANEL_W + x)] != out[(size_t)(r * PANEL_W + x)]) ++badBand;
  }
  kj::expand2xRows(px, lut, band.data(), 133, 135);
  for (int r = 133; r < 135; ++r)
    for (int x = 0; x < PANEL_W; ++x)
      if (band[(size_t)((r - 133) * PANEL_W + x)] != out[(size_t)(r * PANEL_W + x)]) ++badBand;
  check(badBand == 0, "expand2xRows: bands match expand2x (%d mismatches)", badBand);
}

void selftestPng(const std::string& dir) {
  const uint8_t v1[] = "123456789";
  check(sim::crc32(v1, 9) == 0xCBF43926u, "crc32(\"123456789\") = %08X (want CBF43926)", sim::crc32(v1, 9));
  const uint8_t v2[] = "Wikipedia";
  check(sim::adler32(v2, 9) == 0x11E60398u, "adler32(\"Wikipedia\") = %08X (want 11E60398)",
        sim::adler32(v2, 9));
  const uint8_t v3[] = "a";
  check(sim::fnv1a(v3, 1) == 0xE40C292Cu, "fnv1a(\"a\") = %08X (want E40C292C)", sim::fnv1a(v3, 1));

  const int sizes[][2] = {{1, 1}, {37, 23}, {300, 300}, {480, 270}};
  uint32_t s = 0xC0FFEEu;
  for (const auto& sz : sizes) {
    sim::Image img;
    img.resize(sz[0], sz[1]);
    for (uint8_t& v : img.rgb) v = (uint8_t)xs32(s);
    std::vector<uint8_t> enc;
    const bool ok = sim::pngEncode(img.rgb.data(), img.w, img.h, enc);
    check(ok && enc.size() == sim::pngExpectedSize(img.w, img.h), "png %dx%d: size %u == expected %u", img.w,
          img.h, (unsigned)enc.size(), (unsigned)sim::pngExpectedSize(img.w, img.h));
    sim::Image dec;
    std::string err;
    const bool dok = sim::pngDecodeStored(enc.data(), enc.size(), dec, &err);
    check(dok && dec.w == img.w && dec.h == img.h && dec.rgb == img.rgb, "png %dx%d: decode round trip %s",
          img.w, img.h, dok ? "" : err.c_str());
  }

  // a real file through the device-exact path: test pattern -> expand2x -> PNG on disk -> decode
  static uint8_t px[kj::LPIX];
  static uint32_t lut[256];
  for (int i = 0; i < 256; ++i) {
    const uint32_t rgb = ((uint32_t)(i * 37 & 255) << 16) | ((uint32_t)(i * 91 & 255) << 8) | (uint32_t)(255 - i);
    const uint16_t c = kj::rgb565sw(rgb);
    lut[i] = ((uint32_t)c << 16) | c;
  }
  for (int y = 0; y < kj::LH; ++y)
    for (int x = 0; x < kj::LW; ++x) px[y * kj::LW + x] = (uint8_t)((x / 8) * 16 + (y / 5));
  std::vector<uint16_t> panel((size_t)PANEL_PIX);
  kj::expand2x(px, lut, panel.data());
  sim::Image img;
  img.resize(PANEL_W, PANEL_H);
  for (int i = 0; i < PANEL_PIX; ++i) img.set(i % PANEL_W, i / PANEL_W, kj::rgb565swToRgb888(panel[(size_t)i]));
  sim::drawLabel(img, 4, 4, "MOONSKIM SELFTEST", 0xFFFFFFu, 2);
  sim::Image big;
  sim::scaleNearest(img, 2, big);
  const std::string path = (fs::path(dir) / "selftest.png").string();
  bool wrote = sim::pngWrite(path.c_str(), big);
  bool reread = false;
  if (wrote) {
    FILE* f = fopen(path.c_str(), "rb");
    std::vector<uint8_t> data;
    if (f) {
      uint8_t buf[65536];
      size_t n;
      while ((n = fread(buf, 1, sizeof(buf), f)) > 0) data.insert(data.end(), buf, buf + n);
      fclose(f);
    }
    sim::Image dec;
    reread = data.size() == sim::pngExpectedSize(big.w, big.h) &&
             sim::pngDecodeStored(data.data(), data.size(), dec, nullptr) && dec.rgb == big.rgb;
  }
  check(wrote && reread, "png file %s written and re-read (480x270)", path.c_str());
}

void selftestWav(const std::string& dir) {
  const std::string path = (fs::path(dir) / "tone.wav").string();
  sim::WavWriter w;
  bool ok = w.open(path.c_str(), kj::cfg::AUDIO_RATE);
  int16_t buf[1600];
  for (int i = 0; i < 1600; ++i)
    buf[i] = (int16_t)(8000.0f * sinf(kj::TAU * 440.0f * (float)i / (float)kj::cfg::AUDIO_RATE));
  ok = ok && w.write(buf, 1600);
  ok = ok && w.close();
  std::error_code ec;
  const uintmax_t sz = fs::file_size(fs::path(path), ec);
  uint8_t h[44] = {};
  FILE* f = fopen(path.c_str(), "rb");
  if (f) {
    if (fread(h, 1, 44, f) != 44) ok = false;
    fclose(f);
  }
  const uint32_t riff = (uint32_t)h[4] | ((uint32_t)h[5] << 8) | ((uint32_t)h[6] << 16) | ((uint32_t)h[7] << 24);
  const uint32_t data = (uint32_t)h[40] | ((uint32_t)h[41] << 8) | ((uint32_t)h[42] << 16) | ((uint32_t)h[43] << 24);
  check(ok && !ec && sz == 44u + 3200u && riff == 36u + 3200u && data == 3200u && memcmp(h, "RIFF", 4) == 0 &&
            memcmp(h + 8, "WAVE", 4) == 0,
        "wav: %s size=%u riff=%u data=%u", path.c_str(), (unsigned)sz, riff, data);
}

void selftestDepth() {
  int bad = 0;
  uint8_t prev = 0;
  for (float z = -10.0f; z <= 3000.0f; z += 0.25f) {
    const uint8_t q = kj::depthQ(z);
    if (q < prev || q == kj::DEPTH_FAR) ++bad;
    prev = q;
  }
  check(bad == 0 && kj::depthQ(0.0f) == 0 && kj::depthQ(159.5f) == 159 && kj::depthQ(1e9f) == 254,
        "depthQ: monotonic, 0 at 0, 1 u steps below 160, saturates at 254 (%d violations)", bad);
}

void selftestImuSigns() {
  sim::SynthImu s;
  s.reset(7);
  s.setNoise(false);
  s.setHoldPose(0.0f, 0.0f);
  kj::ImuSample a = s.step(SIM_DT, 0);
  check(fabsf(a.az - 1.0f) < 1e-4f && fabsf(a.ax) < 1e-4f && fabsf(a.ay) < 1e-4f,
        "synth imu: flat on a table az=%.3f ax=%.3f ay=%.3f", (double)a.az, (double)a.ax, (double)a.ay);
  s.setTilt(20.0f, 0.0f, 0.0f);
  a = s.step(SIM_DT, 0);
  check(a.ax < -0.3f, "synth imu: right edge down -> ax < 0 (ax=%.3f)", (double)a.ax);
  s.reset(7);
  s.setNoise(false);
  s.setHoldPose(0.0f, 0.0f);
  s.setTilt(0.0f, 20.0f, 0.0f);
  a = s.step(SIM_DT, 0);
  check(a.ay < -0.3f, "synth imu: screen edge away and down -> ay < 0 (ay=%.3f)", (double)a.ay);
  // rolling right at 20 deg/s -> gy = +20 dps (+ bias -0.5)
  s.reset(7);
  s.setNoise(false);
  s.setHoldPose(0.0f, 0.0f);
  s.step(SIM_DT, 0);
  s.setTilt(20.0f, 0.0f, 1.0f);
  float gy = 0.0f;
  for (int i = 0; i < 30; ++i) gy = s.step(SIM_DT, 0).gy;
  check(fabsf(gy - (20.0f - 0.5f)) < 0.1f, "synth imu: rolling right 20 dps -> gy=%.2f (want 19.5 incl. bias)",
        (double)gy);
  // the neutral the game assumes before calibration matches the default hold pose
  const kj::Vec3 u = sim::SynthImu::upDevice(sim::SynthImu::HOLD_DEFAULT_DEG, 0.0f, 0.0f);
  check(fabsf(u.y - 0.57f) < 0.01f && fabsf(u.z - 0.82f) < 0.01f,
        "synth imu: hold pose up=(%.3f,%.3f,%.3f) ~ defaultPersist neutral (0,0.57,0.82)", (double)u.x,
        (double)u.y, (double)u.z);
}

// Tilt checks through the synthetic IMU (end-to-end mapping: device pose -> TiltController).
// Part A is noise-free, so the DESIGN.md section 21 numbers are checked exactly (roll +15 deg
// gives turn 0.409 at Med, only 0.4 deg of margin over 0.4). Part B repeats the essentials with
// the section 18 sensor noise and bias, with thresholds that leave room for it.
struct TiltRig {
  kj::TiltController tc;
  sim::SynthImu s;
  uint32_t us = 0;
  void run(float sec) {
    const int n = (int)lroundf(sec * 60.0f);
    for (int i = 0; i < n; ++i) {
      us += 16667u;
      tc.update(s.step(SIM_DT, us), SIM_DT);
    }
  }
  bool calibrate(float window, float settle) {
    tc.beginCalibration(window);
    run(settle);
    return tc.calibrated() && !tc.calibrating() && !tc.calibTimedOut();
  }
};

void selftestTilt() {
  static TiltRig a;  // large-ish objects: keep them off the stack
  a.s.reset(1234);
  a.s.setNoise(false);
  a.tc.configure(kj::Sens::Med, false, false, 1, 1);
  a.run(0.5f);
  bool cal = a.calibrate(4.0f, 1.5f);
  check(cal, "tilt[A]: calibration at the 35 deg hold pose completes");
  kj::TiltOut o = a.tc.out();
  check(o.valid, "tilt[A]: output valid after calibration");

  a.s.setTilt(2.0f, 0.0f, 0.2f);
  a.run(1.0f);
  o = a.tc.out();
  check(o.turn == 0.0f, "tilt[A]: roll +2 (inside the deadzone) -> turn == 0 (turn=%.5f roll=%.2f)", (double)o.turn,
        (double)o.rollDeg);
  a.s.setTilt(15.0f, 0.0f, 0.3f);
  a.run(1.5f);
  o = a.tc.out();
  check(o.turn > 0.4f, "tilt[A]: roll +15 -> turn > 0.4 (turn=%.3f roll=%.2f)", (double)o.turn, (double)o.rollDeg);
  a.s.setTilt(-15.0f, 0.0f, 0.5f);
  a.run(1.5f);
  o = a.tc.out();
  check(o.turn < -0.4f, "tilt[A]: roll -15 -> turn < -0.4 (turn=%.3f roll=%.2f)", (double)o.turn, (double)o.rollDeg);
  a.s.setTilt(0.0f, -10.0f, 0.3f);
  a.run(1.5f);
  o = a.tc.out();
  check(o.climb > 0.0f && o.turn == 0.0f,
        "tilt[A]: top toward the face (pitch -10) -> climb > 0 (climb=%.3f pitch=%.2f turn=%.3f)", (double)o.climb,
        (double)o.pitchDeg, (double)o.turn);
  a.tc.configure(kj::Sens::Med, true, false, 1, 1);
  a.run(0.5f);
  const kj::TiltOut oi = a.tc.out();
  check(oi.climb < 0.0f && fabsf(oi.climb + o.climb) < 0.01f, "tilt[A]: invert flips it (climb %.3f -> %.3f)",
        (double)o.climb, (double)oi.climb);
  a.tc.configure(kj::Sens::Med, false, false, 1, 1);
  a.s.setTilt(0.0f, 10.0f, 0.3f);
  a.run(1.5f);
  o = a.tc.out();
  check(o.climb < 0.0f, "tilt[A]: top pushed away (pitch +10) -> dive, climb < 0 (climb=%.3f)", (double)o.climb);
  a.tc.configure(kj::Sens::Med, false, false, -1, 1);
  a.s.setTilt(15.0f, 0.0f, 0.3f);
  a.run(1.5f);
  o = a.tc.out();
  check(o.turn < -0.4f, "tilt[A]: rollSign -1 flips the turn (turn=%.3f)", (double)o.turn);
  a.tc.configure(kj::Sens::Med, false, false, 1, 1);

  // curve shape (Med: roll dz / full from config.h): out = u (LIN + (1 - LIN) u)
  const float medDz = kj::cfg::TILT_ROLL_DZ[(int)kj::Sens::Med];
  const float medFull = kj::cfg::TILT_ROLL_FULL[(int)kj::Sens::Med];
  a.s.setTilt(medFull, 0.0f, 0.3f);
  a.run(2.0f);
  o = a.tc.out();
  check(fabsf(o.turn - 1.0f) < 0.01f, "tilt[A]: roll full scale -> turn 1 (turn=%.3f)", (double)o.turn);
  a.s.setTilt(14.0f, 0.0f, 0.3f);
  a.run(2.0f);
  o = a.tc.out();
  const float uu = (14.0f - medDz) / (medFull - medDz);
  const float want = uu * (kj::cfg::TILT_CURVE_LIN + (1.0f - kj::cfg::TILT_CURVE_LIN) * uu);
  check(fabsf(o.turn - want) < 0.01f, "tilt[A]: roll 14 -> turn %.4f (curve %.4f)", (double)o.turn, (double)want);
  a.s.setTilt(0.0f, 70.0f, 0.5f);
  a.run(1.5f);
  o = a.tc.out();
  check(!o.valid && o.turn == 0.0f && o.climb == 0.0f,
        "tilt[A]: guard beyond 60 deg pitch -> invalid, output 0 (valid=%d turn=%.3f climb=%.3f)", o.valid ? 1 : 0,
        (double)o.turn, (double)o.climb);

  // ---- Part B: with sensor noise and gyro bias (section 18)
  static TiltRig b;
  b.s.reset(98765);
  b.tc.configure(kj::Sens::Med, false, false, 1, 1);
  b.run(0.5f);
  cal = b.calibrate(4.0f, 1.5f);
  const kj::Vec3 bias = b.tc.gyroBias();
  check(cal && fabsf(bias.x - 0.8f) < 0.3f && fabsf(bias.y + 0.5f) < 0.3f && fabsf(bias.z - 0.3f) < 0.3f,
        "tilt[B]: noisy calibration estimates the gyro bias (%.2f, %.2f, %.2f) ~ (0.8, -0.5, 0.3)", (double)bias.x,
        (double)bias.y, (double)bias.z);
  b.s.setTilt(2.0f, 0.0f, 0.2f);
  b.run(1.0f);
  o = b.tc.out();
  check(o.turn == 0.0f, "tilt[B]: roll +2 -> turn == 0 (turn=%.5f roll=%.2f)", (double)o.turn, (double)o.rollDeg);
  b.s.setTilt(20.0f, 0.0f, 0.3f);
  b.run(1.5f);
  o = b.tc.out();
  check(o.turn > 0.55f && fabsf(o.rollDeg - 20.0f) < 1.5f, "tilt[B]: roll +20 -> turn > 0.55 (turn=%.3f roll=%.2f)",
        (double)o.turn, (double)o.rollDeg);
  b.s.setTilt(0.0f, -12.0f, 0.3f);
  b.run(1.5f);
  o = b.tc.out();
  check(o.climb > 0.2f, "tilt[B]: pitch -12 -> climb > 0.2 (climb=%.3f pitch=%.2f)", (double)o.climb,
        (double)o.pitchDeg);
  // recalibrating at a new hold pose zeroes the output
  b.s.setTilt(0.0f, 0.0f, 0.5f);
  b.s.setHoldPose(55.0f, 0.5f);
  b.run(1.5f);
  const kj::TiltOut before = b.tc.out();
  cal = b.calibrate(2.0f, 2.0f);
  o = b.tc.out();
  check(cal && before.climb > 0.3f && o.turn == 0.0f && o.climb == 0.0f,
        "tilt[B]: recalibration at a 55 deg hold pose zeroes the output (before climb=%.3f, after turn=%.4f "
        "climb=%.4f)",
        (double)before.climb, (double)o.turn, (double)o.climb);
}


int runSelftest(const std::string& outRoot) {
  const std::string dir = (fs::path(outRoot) / "selftest").string();
  if (!makeDirs(dir)) {
    say("[SELFTEST] ERROR cannot create %s", dir.c_str());
    return 2;
  }
  selftestExpand();
  selftestPng(dir);
  selftestWav(dir);
  selftestDepth();
  selftestImuSigns();
  selftestTilt();
  say("[SELFTEST] %s: %d passed, %d failed (files in %s)", g_stFail ? "FAILED" : "PASSED", g_stPass, g_stFail,
       dir.c_str());
  return g_stFail ? 1 : 0;
}

Sim g_sim;  // large (panel buffer, accumulators): static, like the game

}  // namespace

int main(int argc, char** argv) {
  clockInit();
  Options opt;
  if (!parseArgs(argc, argv, opt)) {
    usage();
    return 2;
  }
  const std::string outRoot = opt.out.empty() ? exeDir(argv[0]) : fs::absolute(fs::path(opt.out)).string();
  if (!makeDirs(outRoot)) {
    fprintf(stderr, "cannot create output folder %s\n", outRoot.c_str());
    return 2;
  }
  int rc = 0;
  if (opt.selftest) {
    const int r = runSelftest(outRoot);
    if (r > rc) rc = r;
  }
  if (opt.model) {
    const int r = runModel();
    if (r > rc) rc = r;
  }
  if (!opt.script.empty() || opt.bench > 0) {
    Sim& s = g_sim;
    s.opt = opt;
    s.outRoot = outRoot;
    std::string err;
    bool ok;
    if (!opt.script.empty()) {
      ok = s.script.load(opt.script.c_str(), err);
    } else {
      // built-in bench: autopilot Ring Run at night
      char text[256];
      snprintf(text, sizeof(text), "0 state title\n0 state ring\n0 autopilot on\n0 bench %d\n0 end\n", opt.bench);
      ok = s.script.parse(text, "bench.jsc", err);
      s.opt.bench = 0;
    }
    if (!ok) {
      fprintf(stdout, "[SCRIPT] ERROR %s\n", err.c_str());
      fflush(stdout);
      return 2;
    }
    const int r = s.runScript();
    if (r > rc) rc = r;
  }
  fflush(stdout);
  return rc;
}
