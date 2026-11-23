// plat_log.cpp - USB-CDC serial: boot/mem lines, [STAT], [IMU] stream, [KBD] key echo, serial
// command parsing and the [SHOT] screenshot dump.
// OWNER: group A. See docs/DESIGN.md section 14.
//
// Log path (core 1 only; nothing here is locked): every line goes into a 4 KB software ring
// first and is moved into the HWCDC TX ring (2 KB) only as far as Serial.availableForWrite()
// allows: from logLine() itself and once per frame from logFlush() (main.cpp). Neither ever
// blocks, so the game cannot stall on a slow or absent host. A line is dropped (counted in
// drop=) only when it does not fit in the ring:
//   * periodic lines ([STAT], [IMU] stream, [KBD] echo, [WARN] slow_frame) may only use the
//     ring up to 3 KB, so 1 KB always stays free for event lines ([GAME] [BENCH] [ERR] ...);
//   * an event line that does not fit while a host IS reading (CDC connected and the TX ring
//     drained within the last 300 ms) waits for room, bounded to 30 ms (vTaskDelay, no spin);
//     with no host reading it is dropped and counted in dropi= as well.
// The last [BENCH] result lines are kept and re-printed by serial 'B'.
// The opt-in [SHOT] dump drains the ring first, then writes directly with bounded waits.
#include <Arduino.h>
#include <M5Cardputer.h>
#include <esp_heap_caps.h>
#include <esp_idf_version.h>
#include <esp_system.h>
#include <esp_timer.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <math.h>
#include <stdarg.h>
#include <stdio.h>
#include <string.h>

#include "core/config.h"
#include "core/game.h"
#include "core/input.h"
#include "core/perf.h"
#include "platform/cardputer/plat.h"
#include "platform/cardputer/plat_internal.h"

namespace plat {

namespace {

constexpr size_t TX_RING_BYTES = 2048;   // ring for HWCDC TX (default 256 is too small)
constexpr int LOG_LINE_MAX = 512;            // longest line we ever emit ([STAT] ~ 330 chars)
constexpr int SHOT_LINE_CHARS = 96;      // base64 chars per [SHOT] line (= 72 payload bytes)
constexpr uint32_t SHOT_DEADLINE_US = 6000000;  // whole dump must finish within 6 s

constexpr size_t RING_BYTES = 4096;           // software log ring (in front of the HWCDC ring)
constexpr size_t PERIODIC_CAP = RING_BYTES - 1024;  // periodic lines leave 1 KB for events
constexpr uint32_t HOST_STALL_US = 300000;     // queued data, no TX progress -> nobody reads
constexpr uint32_t EVENT_WAIT_US = 30000;      // max wait for room for one event line
constexpr int BENCH_KEEP = 2;                  // [BENCH] result lines kept for 'B'
constexpr int BENCH_LINE = 256;

char g_ring[RING_BYTES];
size_t g_rTail = 0;      // oldest queued byte
size_t g_rUsed = 0;      // queued bytes
uint32_t g_lastTxUs = 0; // last time the HWCDC accepted bytes (or nothing was queued)
uint32_t g_drop = 0;     // all dropped lines
uint32_t g_dropEvt = 0;  // dropped event (non-periodic) lines
char g_bench[BENCH_KEEP][BENCH_LINE];
int g_benchN = 0;
// Boot transcript: every line logged before the first [STAT] (the host usually attaches after
// the USB re-enumeration, too late for [BOOT] / [DISP] / [IMU] ...). Serial 'v' re-prints it.
constexpr size_t BOOTLOG_BYTES = 1536;
char g_bootLog[BOOTLOG_BYTES];
size_t g_bootLogN = 0;
bool g_bootLogOpen = true;

// Previous totals for the per-window core-0 figures in [STAT].
detail::PresentTotals g_prevPres = {};
detail::InputTotals g_prevIn = {};

bool g_statOn = true;
bool g_keyEcho = (JET_DEBUG != 0);
bool g_shotReq = false;
float g_imuAcc = 0.0f;
uint64_t g_lastEcho = ~0ull;
char g_line[LOG_LINE_MAX + 2];

const char* stateName(kj::GameState s) {
  switch (s) {
    case kj::GameState::Boot: return "BOOT";
    case kj::GameState::Calibrate: return "CAL";
    case kj::GameState::Title: return "TITLE";
    case kj::GameState::Countdown: return "COUNTDOWN";
    case kj::GameState::Play: return "PLAY";
    case kj::GameState::Pause: return "PAUSE";
    case kj::GameState::Crash: return "CRASH";
    case kj::GameState::GameOver: return "OVER";
  }
  return "?";
}

const char* resetName(esp_reset_reason_t r) {
  switch (r) {
    case ESP_RST_POWERON: return "POWERON";
    case ESP_RST_EXT: return "EXT";
    case ESP_RST_SW: return "SW";
    case ESP_RST_PANIC: return "PANIC";
    case ESP_RST_INT_WDT: return "INT_WDT";
    case ESP_RST_TASK_WDT: return "TASK_WDT";
    case ESP_RST_WDT: return "WDT";
    case ESP_RST_DEEPSLEEP: return "DEEPSLEEP";
    case ESP_RST_BROWNOUT: return "BROWNOUT";
    case ESP_RST_SDIO: return "SDIO";
    case ESP_RST_USB: return "USB";
    case ESP_RST_JTAG: return "JTAG";
    default: return "UNKNOWN";
  }
}

const char* flashModeName(FlashMode_t m) {
  switch (m) {
    case FM_QIO: return "QIO";
    case FM_QOUT: return "QOUT";
    case FM_DIO: return "DIO";
    case FM_DOUT: return "DOUT";
    case FM_FAST_READ: return "FAST_READ";
    case FM_SLOW_READ: return "SLOW_READ";
    default: return "UNKNOWN";
  }
}

// Move as much of the ring as the HWCDC TX ring takes right now (never blocks).
size_t txPump() {
  if (g_rUsed == 0) {
    g_lastTxUs = micros32();
    return 0;
  }
  const int avail = Serial.availableForWrite();
  if (avail <= 0) return 0;
  size_t n = (size_t)avail < g_rUsed ? (size_t)avail : g_rUsed;
  size_t done = 0;
  while (n > 0) {
    const size_t contig = RING_BYTES - g_rTail;
    const size_t chunk = n < contig ? n : contig;
    const size_t w = Serial.write(reinterpret_cast<const uint8_t*>(g_ring + g_rTail), chunk);
    if (w == 0) break;
    g_rTail = (g_rTail + w) % RING_BYTES;
    g_rUsed -= w;
    n -= w;
    done += w;
    if (w < chunk) break;
  }
  if (done != 0) g_lastTxUs = micros32();
  return done;
}

// A host is reading: CDC connected and queued data has moved recently.
bool hostReading() {
  if (!Serial) return false;
  return g_rUsed == 0 || (micros32() - g_lastTxUs) < HOST_STALL_US;
}

void ringPut(const char* s, size_t n) {
  size_t head = (g_rTail + g_rUsed) % RING_BYTES;
  const size_t first = (RING_BYTES - head) < n ? (RING_BYTES - head) : n;
  memcpy(g_ring + head, s, first);
  if (n > first) memcpy(g_ring, s + first, n - first);
  g_rUsed += n;
}

bool startsWith(const char* s, const char* p) { return strncmp(s, p, strlen(p)) == 0; }

// Periodic / streaming lines: they come back next second, so they never take the last 1 KB.
bool isPeriodic(const char* s) {
  return startsWith(s, "[STAT]") || startsWith(s, "[IMU] ax=") || startsWith(s, "[KBD] down=") ||
         startsWith(s, "[WARN] slow_frame");
}

void rememberBoot(const char* s, size_t n) {
  if (!g_bootLogOpen) return;
  if (startsWith(s, "[STAT]")) {
    g_bootLogOpen = false;
    return;
  }
  if (g_bootLogN + n + 1 > BOOTLOG_BYTES) return;
  memcpy(g_bootLog + g_bootLogN, s, n);
  g_bootLogN += n;
  g_bootLog[g_bootLogN++] = '\n';
}

void rememberBench(const char* s, size_t n) {
  if (!startsWith(s, "[BENCH] ") || startsWith(s, "[BENCH] start") ||
      startsWith(s, "[BENCH] none")) {
    return;
  }
  if (g_benchN == BENCH_KEEP) {  // keep the newest BENCH_KEEP lines
    for (int i = 1; i < BENCH_KEEP; ++i) memcpy(g_bench[i - 1], g_bench[i], BENCH_LINE);
    g_benchN = BENCH_KEEP - 1;
  }
  const size_t m = n < (size_t)(BENCH_LINE - 1) ? n : (size_t)(BENCH_LINE - 1);
  memcpy(g_bench[g_benchN], s, m);
  g_bench[g_benchN][m] = '\0';
  ++g_benchN;
}

// Drain the whole ring (bounded): used before the [SHOT] dump writes around the ring.
bool ringDrain(uint32_t deadline) {
  for (;;) {
    txPump();
    if (g_rUsed == 0) return true;
    if ((int32_t)(micros32() - deadline) > 0) return false;
    vTaskDelay(1);
  }
}

// Blocking (bounded) write for the [SHOT] dump. Returns false on deadline.
bool writeWait(const char* s, size_t n, uint32_t deadline) {
  while (Serial.availableForWrite() < (int)n) {
    if ((int32_t)(micros32() - deadline) > 0) return false;
    vTaskDelay(1);
  }
  return Serial.write(reinterpret_cast<const uint8_t*>(s), n) == n;
}

// ---- CRC-32 (IEEE, reflected, same as zlib.crc32) for the screenshot payload.
uint32_t crc32Update(uint32_t crc, const uint8_t* p, size_t n) {
  crc = ~crc;
  for (size_t i = 0; i < n; ++i) {
    crc ^= p[i];
    for (int k = 0; k < 8; ++k) crc = (crc >> 1) ^ (0xEDB88320u & (0u - (crc & 1u)));
  }
  return ~crc;
}

const char B64[] = "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";

// Encode n (<= 72) bytes as base64 into out (needs 4*ceil(n/3) + 1 chars). Returns length.
int base64Encode(const uint8_t* in, int n, char* out) {
  int o = 0;
  int i = 0;
  for (; i + 2 < n; i += 3) {
    const uint32_t v = ((uint32_t)in[i] << 16) | ((uint32_t)in[i + 1] << 8) | in[i + 2];
    out[o++] = B64[(v >> 18) & 63];
    out[o++] = B64[(v >> 12) & 63];
    out[o++] = B64[(v >> 6) & 63];
    out[o++] = B64[v & 63];
  }
  if (i < n) {
    uint32_t v = (uint32_t)in[i] << 16;
    if (i + 1 < n) v |= (uint32_t)in[i + 1] << 8;
    out[o++] = B64[(v >> 18) & 63];
    out[o++] = B64[(v >> 12) & 63];
    out[o++] = (i + 1 < n) ? B64[(v >> 6) & 63] : '=';
    out[o++] = '=';
  }
  out[o] = '\0';
  return o;
}

void printHelp() {
  logLine("[HELP] platform: ? help | s stat on/off | k key echo | h heap | g screenshot | "
          "B repeat last bench result | v repeat boot lines");
  logLine("[HELP] core: i imu stream | r/p flip roll/pitch sign | c recal | t tilt | n tod | "
          "a autopilot | b bench | x crash | X reset best | f stats | m jet dump");
}

void logKeyEcho(uint64_t down) {
  char names[320];
  char kps[200];
  int pn = 0, pk = 0;
  names[0] = '\0';
  kps[0] = '\0';
  for (int kp = 0; kp < kj::KP_COUNT; ++kp) {
    if ((down & kj::keyBit(kp)) == 0) continue;
    const int wn = snprintf(names + pn, sizeof(names) - (size_t)pn, "%s%s", pn ? "," : "",
                            kj::KEY_NAMES[kp]);
    if (wn > 0 && pn + wn < (int)sizeof(names)) pn += wn;
    const int wk = snprintf(kps + pk, sizeof(kps) - (size_t)pk, "%s%d", pk ? "," : "", kp);
    if (wk > 0 && pk + wk < (int)sizeof(kps)) pk += wk;
  }
  detail::logf("[KBD] down=%s kp=%s", pn ? names : "-", pk ? kps : "-");
}

void logImuLine(const kj::Game& game) {
  const kj::ImuSample& s = detail::lastImu();
  const kj::Telemetry t = game.telemetry();
  const kj::Persist& p = game.persist();
  detail::logf(
      "[IMU] ax=%.3f ay=%.3f az=%.3f gx=%.2f gy=%.2f gz=%.2f fresh=%d roll=%.1f pitch=%.1f "
      "turn=%.2f climb=%.2f up=(%.3f,%.3f,%.3f) n=(%.3f,%.3f,%.3f) rs=%d ps=%d cal=%d",
      (double)s.ax, (double)s.ay, (double)s.az, (double)s.gx, (double)s.gy, (double)s.gz,
      s.fresh ? 1 : 0, (double)t.tiltRoll, (double)t.tiltPitch, (double)t.turn, (double)t.climb,
      (double)t.up.x, (double)t.up.y, (double)t.up.z, (double)t.neutral.x, (double)t.neutral.y,
      (double)t.neutral.z, (int)p.rollSign, (int)p.pitchSign, t.calibrated ? 1 : 0);
}

void logStat(kj::Game& game) {
  kj::Profiler& prof = game.profiler();
  const kj::Telemetry t = game.telemetry();
  char perf[400];
  prof.format(perf, (int)sizeof(perf));

  // Core-0 figures for this window (deltas of the tasks' monotonic totals):
  //   p0  = present-task CPU per frame (expand2x + the two push calls), ms
  //   sp0 = present-task spin per frame after its timed sleeps, ms
  //   lat = hand-off -> last pixel on the wire, ms
  //   dw0 = present task waited for the previous frame's push (display-bound), ms/frame
  //   in0 = input-task poll time (I2C keyboard + IMU, battery), ms per poll
  //   ist = frames that found no new completed input poll (used the previous snapshot)
  //   pto = present hand-off timeouts + bus hangs (both should stay 0)
  detail::PresentTotals pt;
  detail::presentTotals(pt);
  detail::InputTotals it;
  detail::inputTotals(it);
  const uint32_t pf = pt.frames - g_prevPres.frames;
  const uint32_t ip = it.polls - g_prevIn.polls;
  const double pfd = pf ? 1000.0 * (double)pf : 1.0;
  const double ipd = ip ? 1000.0 * (double)ip : 1.0;
  char c0[160];
  snprintf(c0, sizeof(c0), " p0=%.2f sp0=%.2f lat=%.1f dw0=%.2f in0=%.2f ist=%lu pto=%lu",
           (double)(pt.busyUs - g_prevPres.busyUs) / pfd,
           (double)(pt.spinUs - g_prevPres.spinUs) / pfd,
           (double)(pt.latUs - g_prevPres.latUs) / pfd,
           (double)(pt.waitUs - g_prevPres.waitUs) / pfd,
           (double)(it.pollUs - g_prevIn.pollUs) / ipd,
           (unsigned long)(it.stale - g_prevIn.stale),
           (unsigned long)((pt.timeouts - g_prevPres.timeouts) +
                           (pt.busHang - g_prevPres.busHang)));
  g_prevPres = pt;
  g_prevIn = it;

  detail::logf("[STAT] t=%lu st=%s %s | heap=%lu min=%lu drop=%lu dropi=%lu bat=%umV/%u%% |%s",
               (unsigned long)(millis() / 1000u), stateName(t.state), perf,
               (unsigned long)heap_caps_get_free_size(MALLOC_CAP_INTERNAL),
               (unsigned long)heap_caps_get_minimum_free_size(MALLOC_CAP_INTERNAL),
               (unsigned long)g_drop, (unsigned long)g_dropEvt, (unsigned)detail::batteryMv(),
               (unsigned)detail::batteryPct(), c0);
}

}  // namespace

// ------------------------------------------------------------------ public (plat.h)

uint32_t micros32() { return (uint32_t)esp_timer_get_time(); }

void logLine(const char* s) {
  if (s == nullptr) return;
  size_t n = strlen(s);
  if (n == 0) return;
  if (n > (size_t)LOG_LINE_MAX) n = (size_t)LOG_LINE_MAX;
  rememberBench(s, n);
  rememberBoot(s, n);
  const bool periodic = isPeriodic(s);
  const size_t need = n + 1;  // + '\n'
  const size_t cap = periodic ? PERIODIC_CAP : RING_BYTES;
  txPump();
  if (g_rUsed + need > cap && !periodic && hostReading()) {
    // Event line, host reading, ring full (a burst such as the 'm' jet dump): wait for the
    // host, bounded. vTaskDelay lets IDLE1 run; no spin.
    const uint32_t t0 = micros32();
    while (g_rUsed + need > cap && micros32() - t0 < EVENT_WAIT_US) {
      vTaskDelay(1);
      if (txPump() == 0 && !hostReading()) break;
    }
  }
  if (g_rUsed + need > cap) {
    ++g_drop;
    if (!periodic) ++g_dropEvt;
    return;
  }
  ringPut(s, n);
  ringPut("\n", 1);
  txPump();
}

void logFlush() { txPump(); }

void periodicLogs(kj::Game& game, float dt) {
  kj::Profiler& prof = game.profiler();
  if (prof.windowReady()) {
    if (g_statOn) logStat(game);
    prof.reset();
  }

  if (game.imuStream()) {
    const float period = 1.0f / kj::cfg::IMU_STREAM_HZ;
    g_imuAcc += dt;
    if (g_imuAcc >= period) {
      g_imuAcc -= period;
      if (g_imuAcc > period) g_imuAcc = 0.0f;  // after a hitch: do not burst
      logImuLine(game);
    }
  } else {
    g_imuAcc = 0.0f;
  }

  if (g_keyEcho) {
    const uint64_t down = detail::lastKeysDown();
    if (down != g_lastEcho) {
      g_lastEcho = down;
      logKeyEcho(down);
    }
  }

  if (g_shotReq) {
    g_shotReq = false;
    dumpShot(game.frame().px, game.palette().rgb888());
  }
}

void dumpShot(const uint8_t* px, const uint8_t* rgb888) {
  if (px == nullptr || rgb888 == nullptr) return;
  if (!Serial) {  // no host attached: nobody would read it
    ++g_drop;
    return;
  }
  if (!ringDrain(micros32() + 1000000u)) {  // queued lines first: keeps the output ordered
    logLine("[SHOT] abort reason=log_backlog");
    return;
  }
  const int idxBytes = kj::cfg::LPIX;  // 8160
  const int palBytes = 768;
  const int total = idxBytes + palBytes;  // 8928 = 124 lines of 72 bytes
  const int chunk = SHOT_LINE_CHARS / 4 * 3;  // 72
  const uint32_t deadline = micros32() + SHOT_DEADLINE_US;

  uint32_t crc = crc32Update(0, px, (size_t)idxBytes);
  crc = crc32Update(crc, rgb888, (size_t)palBytes);

  char line[SHOT_LINE_CHARS + 8];
  char hdr[96];
  int n = snprintf(hdr, sizeof(hdr), "[SHOT] begin w=%d h=%d fmt=idx8+pal888 bytes=%d\n",
                   kj::cfg::LW, kj::cfg::LH, total);
  if (n <= 0 || !writeWait(hdr, (size_t)n, deadline)) {
    ++g_drop;
    return;
  }
  uint8_t buf[72];
  for (int off = 0; off < total; off += chunk) {
    const int len = (total - off) < chunk ? (total - off) : chunk;
    for (int i = 0; i < len; ++i) {
      const int k = off + i;
      buf[i] = k < idxBytes ? px[k] : rgb888[k - idxBytes];
    }
    int m = base64Encode(buf, len, line);
    line[m++] = '\n';
    if (!writeWait(line, (size_t)m, deadline)) {
      ++g_drop;
      logLine("[SHOT] abort");
      return;
    }
  }
  n = snprintf(hdr, sizeof(hdr), "[SHOT] end crc=%08lx\n", (unsigned long)crc);
  if (n > 0) writeWait(hdr, (size_t)n, deadline);
}

// ------------------------------------------------------------------ detail (plat_internal.h)
namespace detail {

void serialBegin() {
  // Must run before M5.begin() calls Serial.begin(): HWCDC::begin keeps a preset ring.
  Serial.setTxBufferSize(TX_RING_BYTES);
  Serial.setTxTimeoutMs(0);
}

void serialSetNonBlocking() { Serial.setTxTimeoutMs(0); }

void logf(const char* fmt, ...) {
  va_list ap;
  va_start(ap, fmt);
  const int w = vsnprintf(g_line, (size_t)LOG_LINE_MAX, fmt, ap);
  va_end(ap);
  if (w <= 0) return;
  logLine(g_line);
}

void logMem(const char* stage) {
  logf("[MEM] stage=%s free_int=%lu largest_dma=%lu min_free=%lu loop_stack_hw=%lu", stage,
       (unsigned long)heap_caps_get_free_size(MALLOC_CAP_INTERNAL),
       (unsigned long)heap_caps_get_largest_free_block(MALLOC_CAP_DMA | MALLOC_CAP_INTERNAL),
       (unsigned long)heap_caps_get_minimum_free_size(MALLOC_CAP_INTERNAL),
       (unsigned long)uxTaskGetStackHighWaterMark(nullptr));
}

void logBootLines(uint32_t seed) {
  const int board = (int)M5.getBoard();
  const bool adv = M5.getBoard() == m5::board_t::board_M5CardputerADV;
  logf("[BOOT] fw=%s %s build=%s %s board=%d adv=%d cpu=%lu flash=%luMB/%s@%luMHz "
       "arduino=%s idf=%s spi=%dMHz fps_cap=%d",
       JET_FW_NAME, JET_FW_VERSION, __DATE__, __TIME__, board, adv ? 1 : 0,
       (unsigned long)getCpuFrequencyMhz(),
       (unsigned long)(ESP.getFlashChipSize() / (1024u * 1024u)),
       flashModeName(ESP.getFlashChipMode()), (unsigned long)(ESP.getFlashChipSpeed() / 1000000u),
       ESP_ARDUINO_VERSION_STR, esp_get_idf_version(), (int)JET_SPI_MHZ, (int)JET_FPS_CAP);
  logf("[BOOT] reset=%s seed=0x%08lX", resetName(esp_reset_reason()), (unsigned long)seed);
}

char serialPoll() {
  if (Serial.available() <= 0) return 0;
  const int c = Serial.read();
  if (c < 0) return 0;
  switch (c) {
    case '?':
      printHelp();
      return 0;
    case 's':
      g_statOn = !g_statOn;
      logf("[CMD] stat=%d", g_statOn ? 1 : 0);
      return 0;
    case 'k':
      g_keyEcho = !g_keyEcho;
      g_lastEcho = ~0ull;  // echo the current state right away
      logf("[CMD] key_echo=%d", g_keyEcho ? 1 : 0);
      return 0;
    case 'h':
      logMem("cmd");
      logf("[MEM] dma_free=%lu dma_largest=%lu disp=%s present_stack_free=%lu "
           "input_stack_free=%lu log_queued=%u",
           (unsigned long)heap_caps_get_free_size(MALLOC_CAP_DMA),
           (unsigned long)heap_caps_get_largest_free_block(MALLOC_CAP_DMA),
           displayFullDma() ? "full" : "band", (unsigned long)presentStackFree(),
           (unsigned long)inputStackFree(), (unsigned)g_rUsed);
      return 0;
    case 'g':
      g_shotReq = true;
      return 0;
    case 'v': {  // repeat the boot transcript, one line at a time
      g_bootLogOpen = false;           // the replay must not append to the transcript it reads
      const size_t total = g_bootLogN;
      size_t a = 0;
      char tmp[LOG_LINE_MAX + 1];
      for (size_t i = 0; i < total; ++i) {
        if (g_bootLog[i] != '\n') continue;
        size_t n = i - a;
        if (n > (size_t)LOG_LINE_MAX) n = (size_t)LOG_LINE_MAX;
        memcpy(tmp, g_bootLog + a, n);
        tmp[n] = '\0';
        logLine(tmp);
        a = i + 1;
      }
      return 0;
    }
    case 'B': {  // repeat the last [BENCH] results (in case the host missed them)
      if (g_benchN == 0) logLine("[BENCH] none");
      char tmp[BENCH_KEEP][BENCH_LINE];
      const int n = g_benchN;
      memcpy(tmp, g_bench, sizeof(tmp));  // logLine re-records each line (shifts g_bench)
      for (int i = 0; i < n; ++i) logLine(tmp[i]);
      return 0;
    }
    default:
      break;
  }
  if (c <= ' ' || c > '~') return 0;  // CR/LF/space/control/non-ASCII: ignored
  return (char)c;
}

uint32_t droppedLines() { return g_drop; }

}  // namespace detail
}  // namespace plat
