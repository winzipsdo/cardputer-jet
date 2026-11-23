// plat_input.cpp - keyboard drain (by physical key position), G0 button, IMU read with the
// raw -> device axis remap, battery, serial command intake.
// OWNER: group A. See docs/DESIGN.md sections 8, 9 and 17.
//
// Device polling runs in the INPUT TASK (pinned to core 0, prio 3): after setup it is the only
// I2C user (TCA8418 keyboard, BMI270 IMU; the ES8311 codec is configured once inside
// Speaker.begin() during setup and never touched again). It also owns M5.update() / BtnA and
// the battery ADC. Core 1 kicks it from the pacing wait in main.cpp, ~1.5-2.6 ms before the
// next frame starts (immediately when the frame is late / uncapped), so a poll (~1 ms, mostly
// I2C at 400 kHz) completes before pollInput() and core 1 no longer spends ~1 ms on it.
//
// pollInput() (core 1) only takes the latest completed snapshot under a spinlock (~10 us):
//   - keys: `down` from the last drain, `seen` = OR of every key list since the previous
//     pollInput (taps shorter than a frame survive, as before), keysPressed as before;
//   - BtnA: held state + wasPressed OR-ed since the previous pollInput;
//   - IMU: the latest valid sample; fresh = true only if a new valid sample arrived since the
//     previous pollInput.
// Latency: in paced frames the sample is ~1.5-2.6 ms old when the frame starts (it was
// ~0.3-1 ms old before, read inside the frame). In a late frame the kick lands right before
// the frame, the poll is still running, and the frame uses the previous snapshot (fresh =
// false, keys still accumulated): at most one frame older. Nothing is ever lost.
// Logging stays on core 1 (the log ring is not thread-safe): the task only counts.
#include <Arduino.h>
#include <M5Cardputer.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <math.h>

#include "core/config.h"
#include "core/game.h"
#include "core/input.h"
#include "core/perf.h"
#include "platform/cardputer/plat.h"
#include "platform/cardputer/plat_internal.h"

namespace plat {

namespace {

constexpr int KB_COLS = 14, KB_ROWS = 4;
constexpr int EXTRA_DRAINS = 3;               // + the one inside M5Cardputer.update() = 4
constexpr float IMU_MAX_G = 16.0f;            // anything beyond is a bus glitch
constexpr float IMU_MAX_DPS = 4000.0f;
constexpr uint32_t IMU_STALE_US = (uint32_t)(kj::cfg::TILT_STALE_S * 1000000.0f);
constexpr uint32_t BATTERY_US = (uint32_t)(kj::cfg::BATTERY_PERIOD_S * 1000000.0f);

constexpr uint32_t TASK_STACK = 4096;         // bytes
constexpr UBaseType_t TASK_PRIO = 3;          // above the speaker mixer (2), below present (5)
constexpr BaseType_t TASK_CORE = 0;

// Snapshot shared between the input task (writer) and pollInput (reader), under g_mux.
struct Snap {
  uint64_t down = 0;          // keys held after the last drain
  uint64_t seen = 0;          // OR of all key lists since the last consume
  bool btnA = false;
  bool btnAPressed = false;   // OR since the last consume
  bool imuNew = false;        // a new valid sample since the last consume
  kj::ImuSample imu = {0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 0u, false};
  uint32_t seq = 0;           // completed polls
  uint32_t pollUs = 0;        // total poll time
  uint32_t rejects = 0;       // total rejected IMU samples
  uint16_t batMv = 0;
  uint8_t batPct = 0;
};
Snap g_snap;
portMUX_TYPE g_mux = portMUX_INITIALIZER_UNLOCKED;

TaskHandle_t g_task = nullptr;
StaticTask_t g_taskTcb;
StackType_t g_taskStack[TASK_STACK];

// Task-side state (or core 1 before the task exists).
bool g_imuPresent = false;
uint32_t g_batUs = 0;

// Core-1 state.
uint64_t g_prevDown = 0;
uint32_t g_lastPollUs = 0;
bool g_firstPoll = true;
kj::ImuSample g_imu = {0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 0u, false};
uint32_t g_lastFreshUs = 0;
bool g_staleLogged = false;
bool g_rejectLogged = false;
uint32_t g_lastSeq = 0;
uint32_t g_staleFrames = 0;
uint16_t g_batMv = 0;
uint8_t g_batPct = 0;

// Current key list -> bit mask by physical position kp = y*14 + x (never by character, so
// Shift / Ctrl / Fn / caps lock cannot change a key's meaning).
uint64_t scanKeys() {
  uint64_t m = 0;
  const auto& list = M5Cardputer.Keyboard.keyList();
  for (const auto& p : list) {
    if (p.x < 0 || p.x >= KB_COLS || p.y < 0 || p.y >= KB_ROWS) continue;
    m |= kj::keyBit(p.y * KB_COLS + p.x);
  }
  return m;
}

void readBattery(uint16_t& mvOut, uint8_t& pctOut) {
  const int16_t mv = M5.Power.getBatteryVoltage();
  int32_t pct = M5.Power.getBatteryLevel();
  if (pct < 0) pct = 0;
  if (pct > 100) pct = 100;
  mvOut = mv > 0 ? (uint16_t)mv : 0;
  pctOut = (uint8_t)pct;
}

inline float pick3(const float* v, int idx) { return v[idx < 0 ? 0 : (idx > 2 ? 2 : idx)]; }

bool sane(float v, float lim) { return isfinite(v) && fabsf(v) <= lim; }

// Reads one BMI270 sample if a new one is ready; applies cfg::IMU_SRC_* / IMU_SGN_*.
// Returns 1 = new valid sample in `out`, 0 = nothing new, -1 = sample rejected.
int readImu(kj::ImuSample& out) {
  if (!g_imuPresent) return 0;
  if (M5.Imu.update() == 0) return 0;
  const m5::IMU_Class::imu_data_t& d = M5.Imu.getImuData();
  const float a[3] = {d.accel.x, d.accel.y, d.accel.z};
  const float g[3] = {d.gyro.x, d.gyro.y, d.gyro.z};
  kj::ImuSample s;
  s.ax = kj::cfg::IMU_SGN_X * pick3(a, kj::cfg::IMU_SRC_X);
  s.ay = kj::cfg::IMU_SGN_Y * pick3(a, kj::cfg::IMU_SRC_Y);
  s.az = kj::cfg::IMU_SGN_Z * pick3(a, kj::cfg::IMU_SRC_Z);
  s.gx = kj::cfg::IMU_SGN_X * pick3(g, kj::cfg::IMU_SRC_X);
  s.gy = kj::cfg::IMU_SGN_Y * pick3(g, kj::cfg::IMU_SRC_Y);
  s.gz = kj::cfg::IMU_SGN_Z * pick3(g, kj::cfg::IMU_SRC_Z);
  s.usec = d.usec;
  s.fresh = true;
  if (sane(s.ax, IMU_MAX_G) && sane(s.ay, IMU_MAX_G) && sane(s.az, IMU_MAX_G) &&
      sane(s.gx, IMU_MAX_DPS) && sane(s.gy, IMU_MAX_DPS) && sane(s.gz, IMU_MAX_DPS) &&
      s.ax * s.ax + s.ay * s.ay + s.az * s.az >= 0.0025f) {  // |a| >= 0.05 g (not all-zero)
    out = s;
    return 1;
  }
  return -1;
}

// One full device poll (I2C + GPIO + ADC) merged into g_snap. Runs in the input task, or on
// core 1 when the task could not be created.
void pollDevices() {
  const uint32_t t0 = micros32();

  // Keyboard: the TCA8418 reader returns ONE FIFO event per updateKeyList() (and does no I2C
  // at all while its INT flag is clear), so drain it 4 times per poll and OR every
  // intermediate list (taps shorter than a frame survive).
  M5Cardputer.update();  // M5.update() (BtnA) + updateKeyList() + updateKeysState()
  uint64_t seen = scanKeys();
  uint64_t down = seen;
  for (int i = 0; i < EXTRA_DRAINS; ++i) {
    M5Cardputer.Keyboard.updateKeyList();
    down = scanKeys();
    seen |= down;
  }
  const bool btn = M5.BtnA.isPressed();
  const bool btnPressed = M5.BtnA.wasPressed();

  kj::ImuSample s;
  const int imu = readImu(s);

  bool bat = false;
  uint16_t mv = 0;
  uint8_t pct = 0;
  if (t0 - g_batUs >= BATTERY_US) {
    g_batUs = t0;
    readBattery(mv, pct);
    bat = true;
  }
  const uint32_t cost = micros32() - t0;

  portENTER_CRITICAL(&g_mux);
  g_snap.down = down;
  g_snap.seen |= seen;
  g_snap.btnA = btn;
  g_snap.btnAPressed = g_snap.btnAPressed || btnPressed;
  if (imu > 0) {
    g_snap.imu = s;
    g_snap.imuNew = true;
  } else if (imu < 0) {
    ++g_snap.rejects;
  }
  if (bat) {
    g_snap.batMv = mv;
    g_snap.batPct = pct;
  }
  g_snap.pollUs += cost;
  ++g_snap.seq;
  portEXIT_CRITICAL(&g_mux);
}

void inputTask(void*) {
  for (;;) {
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);  // kicked by core 1 (main.cpp pace)
    pollDevices();
  }
}

}  // namespace

// ------------------------------------------------------------------ public (plat.h)

bool isAdv() { return M5.getBoard() == m5::board_t::board_M5CardputerADV; }

bool imuPresent() { return isAdv() && M5.Imu.getType() != m5::imu_none; }

void inputKick() {
  if (g_task != nullptr) xTaskNotifyGive(g_task);
}

void pollInput(kj::InputFrame& in, kj::Game& game) {
  kj::Profiler& prof = game.profiler();
  prof.begin(kj::ST_INPUT);
  const uint32_t now = micros32();

  // Real frame time (pacing included); the game clamps it to cfg::DT_MAX.
  float dt = 1.0f / 60.0f;
  if (!g_firstPoll) {
    dt = (float)(now - g_lastPollUs) * 1e-6f;
  } else {
    g_lastFreshUs = now;  // game init (world gen) ran since inputBegin: not an IMU stall
  }
  g_firstPoll = false;
  g_lastPollUs = now;

  if (g_task == nullptr) pollDevices();  // no task: poll synchronously (old behaviour)

  Snap s;
  portENTER_CRITICAL(&g_mux);
  s = g_snap;
  g_snap.seen = 0;
  g_snap.btnAPressed = false;
  g_snap.imuNew = false;
  portEXIT_CRITICAL(&g_mux);

  if (s.seq == g_lastSeq) ++g_staleFrames;  // the kicked poll has not completed yet
  g_lastSeq = s.seq;

  in.dt = dt;
  in.keysDown = s.down;
  in.keysPressed = (s.seen | s.down) & ~g_prevDown;
  g_prevDown = s.down;

  in.btnA = s.btnA;
  in.btnAPressed = s.btnAPressed;

  if (s.imuNew) {
    g_imu = s.imu;
    g_imu.fresh = true;
    g_lastFreshUs = now;
    g_staleLogged = false;  // re-arm the stale warning
  } else {
    g_imu.fresh = false;    // keep the last sample
    const uint32_t age = now - g_lastFreshUs;
    if (g_imuPresent && age > IMU_STALE_US && !g_staleLogged) {
      g_staleLogged = true;
      detail::logf("[ERR] imu_stale ms=%lu", (unsigned long)(age / 1000u));
    }
  }
  if (s.rejects != 0 && !g_rejectLogged) {
    g_rejectLogged = true;
    logLine("[WARN] imu_reject");
  }
  in.imuPresent = g_imuPresent;
  in.imu = g_imu;

  if (s.batMv != 0 || s.batPct != 0) {
    g_batMv = s.batMv;
    g_batPct = s.batPct;
  }
  in.batteryPct = g_batPct;
  in.batteryMv = g_batMv;

  in.serialCmd = detail::serialPoll();

  prof.end(kj::ST_INPUT);
}

// ------------------------------------------------------------------ detail
namespace detail {

void inputBegin() {
  g_imuPresent = imuPresent();
  g_prevDown = 0;
  g_firstPoll = true;
  g_lastFreshUs = micros32();
  g_staleLogged = false;
  readBattery(g_batMv, g_batPct);
  g_snap.batMv = g_batMv;
  g_snap.batPct = g_batPct;
  g_batUs = micros32();
}

void inputStartTask() {
  g_task = xTaskCreateStaticPinnedToCore(inputTask, "input", TASK_STACK, nullptr, TASK_PRIO,
                                         g_taskStack, &g_taskTcb, TASK_CORE);
  logf("[INPUT] task=%d core=%d prio=%u", g_task != nullptr ? 1 : 0, (int)TASK_CORE,
       (unsigned)TASK_PRIO);
  inputKick();  // first poll now, so the first frame already has one
}

const kj::ImuSample& lastImu() { return g_imu; }
uint64_t lastKeysDown() { return g_prevDown; }
uint16_t batteryMv() { return g_batMv; }
uint8_t batteryPct() { return g_batPct; }

void inputTotals(InputTotals& out) {
  portENTER_CRITICAL(&g_mux);
  out.polls = g_snap.seq;
  out.pollUs = g_snap.pollUs;
  portEXIT_CRITICAL(&g_mux);
  out.stale = g_staleFrames;
}

uint32_t inputStackFree() {
  return g_task != nullptr ? (uint32_t)uxTaskGetStackHighWaterMark(g_task) : 0u;
}

}  // namespace detail
}  // namespace plat
