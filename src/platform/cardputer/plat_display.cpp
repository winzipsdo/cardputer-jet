// plat_display.cpp - platform bring-up (plat::begin), DMA frame buffer (+ band fallback),
// display init (landscape rotation, brightness, optional 80 MHz SPI) and the PRESENT TASK.
// OWNER: group A. See docs/DESIGN.md sections 3, 14 and 17.
//
// Present pipeline (full-frame path, the normal case). The display transfer overlaps the
// rendering of the next frame completely:
//   core 1, present():  blocking take of g_semFree (ST_DMAWAIT; normally 0: the staging buffer
//                       was released ~0.4 ms after the previous hand-off) -> memcpy px (8,160 B)
//                       + lut2x (1 KB) into the staging buffer -> give g_semFrame (ST_EXPAND,
//                       ~0.03 ms). Core 1 then goes straight on with the next frame.
//   core 0, presentTask (pinned, prio 5, above the M5 speaker mixer at 2):
//                       take g_semFrame -> wait until the previous frame's bottom half has left
//                       the GDMA -> expand2x staging -> g_fb (0.37 ms) -> give g_semFree ->
//                       push rows 0..67 (32,640 B) -> sleep (esp_timer one-shot) until that
//                       half is nearly out, spin the last ~0.1 ms -> push rows 68..134
//                       (32,160 B) -> wait for the next frame.
//   Why two halves: the panel sits on SPI3, where the ESP32-S3 has no segmented transfer (SCT
//   is SPI2 only), so M5GFX's Bus_SPI::writeBytes splits anything > 32,768 B into 32 KB SPI
//   transactions and BUSY-WAITS for every segment but the last. A push of <= 32,768 B is one
//   transaction and returns at once, so this task sleeps instead of spinning (IDLE0 keeps
//   running, which matters in the uncapped bench env where the display is the bottleneck).
//   Display throughput: 0.37 + 6.53 + 6.43 ms = ~13.4 ms per frame at 40 MHz (~75 fps max),
//   ~6.9 ms at 80 MHz. Frame time on core 1 = max(core-1 work, display period).
//
// Ownership: after setup, EVERY M5.Display call happens in presentTask. It opens the SPI
// transaction itself (startWrite, which also takes the Arduino SPI bus lock for SPI3 in this
// task) and never closes it: Panel_LCD::endWrite would block on the bus. Brightness requests
// from core 1 (applySettings) are handed over through g_wantBright and applied by the task.
// The [SHOT] dump reads the game's px / rgb888 on core 1 after present(); core 0 only ever
// reads its own staging copy, so the screenshot is always the frame that was just rendered.
//
// Band fallback (64,800 B DMA block not available): same hand-off; the task expands band b
// (240x16 RGB565) into one of two buffers while band b-1 DMAs, 9 bands, the last one 7 rows.
#include <Arduino.h>
#include <M5Cardputer.h>
#include <esp_heap_caps.h>
#include <esp_timer.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include <freertos/task.h>
#include <string.h>

#include "core/config.h"
#include "core/expand.h"
#include "core/game.h"
#include "core/perf.h"
#include "platform/cardputer/plat.h"
#include "platform/cardputer/plat_internal.h"

namespace plat {

namespace {

constexpr int W = kj::cfg::SCREEN_W;                 // 240
constexpr int H = kj::cfg::SCREEN_H;                 // 135
constexpr size_t FRAME_BYTES = (size_t)W * H * 2;    // 64,800
constexpr int BAND_ROWS = 16;
constexpr size_t BAND_BYTES = (size_t)W * BAND_ROWS * 2;  // 7,680
constexpr int BANDS = (H + BAND_ROWS - 1) / BAND_ROWS;    // 9
constexpr uint32_t DMA_CAPS = MALLOC_CAP_DMA | MALLOC_CAP_INTERNAL;
constexpr int TOP_ROWS = 68;                               // 240*68*2 = 32,640 B
static_assert((size_t)W * TOP_ROWS * 2 <= 32768, "top half must be one SPI transaction");
static_assert((size_t)W * (H - TOP_ROWS) * 2 <= 32768, "bottom half must be one SPI transaction");

constexpr uint32_t TASK_STACK = 4096;     // bytes (ESP-IDF FreeRTOS counts bytes)
constexpr UBaseType_t TASK_PRIO = 5;      // above the speaker mixer (2) and the input task (3)
constexpr BaseType_t TASK_CORE = 0;
constexpr uint32_t WAKE_EARLY_US = 120;   // wake this long before the predicted DMA end, then spin
constexpr uint32_t SPIN_LIMIT_US = 40000; // a push never takes this long: bus hang -> give up
constexpr uint32_t HANDOFF_TIMEOUT_MS = 100;  // core 1 never blocks longer on a stuck task

uint16_t* g_fb = nullptr;                  // full frame (normal path)
uint16_t* g_band[2] = {nullptr, nullptr};  // band fallback
int g_bright = -1;                         // applied brightness (setup: core 1; then the task)
volatile int g_wantBright = -1;            // requested by core 1 after the task started

// Staging copy of the frame (core 1 writes it, core 0 expands it). Internal DRAM (.bss).
alignas(4) uint8_t g_stPx[kj::cfg::LPIX];
alignas(4) uint32_t g_stLut[256];

TaskHandle_t g_task = nullptr;
StaticTask_t g_taskTcb;
StackType_t g_taskStack[TASK_STACK];
SemaphoreHandle_t g_semFrame = nullptr;  // core 1 -> task: staging holds a new frame
SemaphoreHandle_t g_semFree = nullptr;   // task -> core 1: staging may be overwritten
SemaphoreHandle_t g_semReady = nullptr;  // task -> setup: transaction open, DMA primed
StaticSemaphore_t g_semFrameBuf, g_semFreeBuf, g_semReadyBuf;
esp_timer_handle_t g_wakeTimer = nullptr;

// Last push issued by the task (task-local state).
uint32_t g_pushStartUs = 0;
uint32_t g_pushUs = 0;      // predicted wire time of that push
bool g_pushPending = false;

// Hand-off time written by core 1 before giving g_semFrame (the give/take pair orders it).
volatile uint32_t g_handoffUs = 0;

// Totals (monotonic, wrap-safe deltas). Written by the task, read by core 1.
volatile uint32_t g_totFrames = 0, g_totBusyUs = 0, g_totSpinUs = 0, g_totLatUs = 0,
                  g_totWaitUs = 0;
volatile uint32_t g_totTimeouts = 0;  // core-1 hand-off timeouts (written by core 1)
volatile uint32_t g_totBusHang = 0;   // waitPush gave up (written by the task)

inline const lgfx::swap565_t* asPanel(const uint16_t* p) {
  // The buffer already holds byte-swapped RGB565 (= the ST7789's native order), so it is
  // pushed as swap565_t: no per-pixel conversion, the GDMA reads it directly.
  return reinterpret_cast<const lgfx::swap565_t*>(p);
}

// Wire time of n bytes at the configured SPI clock (40 MHz: 0.2 us per byte).
constexpr uint32_t wireUs(size_t bytes) { return (uint32_t)(bytes * 8u / (size_t)JET_SPI_MHZ); }

void onWakeTimer(void*) {
  // esp_timer task context (not an ISR): a plain notify is allowed.
  if (g_task != nullptr) xTaskNotifyGive(g_task);
}

// Sleep (no CPU) until about WAKE_EARLY_US before `target`.
void sleepUntil(uint32_t target) {
  for (;;) {
    const int32_t rem = (int32_t)(target - micros32());
    if (rem <= (int32_t)(2 * WAKE_EARLY_US)) return;
    esp_timer_stop(g_wakeTimer);  // not running -> ESP_ERR_INVALID_STATE, ignored
    if (esp_timer_start_once(g_wakeTimer, (uint64_t)(rem - (int32_t)WAKE_EARLY_US)) != ESP_OK) {
      vTaskDelay(1);
      continue;
    }
    // Timeout only as a safety net (a lost wake costs one tick, never a hang).
    ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(20) + 1);
  }
}

// Wait until the last push has left the SPI peripheral. Returns the time spent spinning.
uint32_t waitPush() {
  if (!g_pushPending) return 0;
  g_pushPending = false;
  sleepUntil(g_pushStartUs + g_pushUs);
  const uint32_t s0 = micros32();
  while (M5.Display.dmaBusy()) {
    if (micros32() - s0 > SPIN_LIMIT_US) {
      g_totBusHang = g_totBusHang + 1;
      break;
    }
  }
  return micros32() - s0;
}

void push(int y, int rows, const uint16_t* src) {
  M5.Display.pushImageDMA(0, y, W, rows, asPanel(src));
  // The data phase started inside the call: predicting from "now" is a little pessimistic,
  // the final spin absorbs the difference.
  g_pushStartUs = micros32();
  g_pushUs = wireUs((size_t)W * (size_t)rows * 2u);
  g_pushPending = true;
}

void applyBrightness() {
  const int want = g_wantBright;
  if (want >= 0 && want != g_bright) {
    M5.Display.setBrightness((uint8_t)want);  // PWM backlight, not the SPI bus
    g_bright = want;
  }
}

void presentFull(uint32_t handoff) {
  const uint32_t w0 = micros32();
  const uint32_t spinPrev = waitPush();  // previous frame's bottom half (normally long done)
  const uint32_t t0 = micros32();
  kj::expand2x(g_stPx, g_stLut, g_fb);
  xSemaphoreGive(g_semFree);  // px/lut copy consumed: core 1 may hand over the next frame
  push(0, TOP_ROWS, g_fb);
  const uint32_t busy1 = micros32() - t0;
  const uint32_t spinTop = waitPush();
  const uint32_t t1 = micros32();
  push(TOP_ROWS, H - TOP_ROWS, g_fb + (size_t)W * TOP_ROWS);
  const uint32_t t2 = micros32();
  g_totBusyUs = g_totBusyUs + busy1 + (t2 - t1);
  g_totSpinUs = g_totSpinUs + spinPrev + spinTop;
  g_totWaitUs = g_totWaitUs + (t0 - w0);
  g_totLatUs = g_totLatUs + (g_pushStartUs + g_pushUs - handoff);
}

void presentBands(uint32_t handoff) {
  uint32_t busy = 0, spin = 0;
  const uint32_t w0 = micros32();
  // Band 0 reuses g_band[0], which the previous frame's last band (8) may still be pushing.
  spin += waitPush();
  const uint32_t wait = micros32() - w0;
  for (int b = 0; b < BANDS; ++b) {
    const int r0 = b * BAND_ROWS;
    const int r1 = (r0 + BAND_ROWS) < H ? (r0 + BAND_ROWS) : H;
    uint16_t* buf = g_band[b & 1];
    // b >= 2: buf was pushed as band b-2, which finished before band b-1 was queued.
    // b == 1: g_band[1] was band 7 of the previous frame, finished before its band 8.
    const uint32_t t0 = micros32();
    kj::expand2xRows(g_stPx, g_stLut, buf, r0, r1);
    if (b == BANDS - 1) xSemaphoreGive(g_semFree);  // last read of the staging copy
    busy += micros32() - t0;
    spin += waitPush();  // band b-1 done
    const uint32_t t1 = micros32();
    push(r0, r1 - r0, buf);
    busy += micros32() - t1;
  }
  g_totBusyUs = g_totBusyUs + busy;
  g_totSpinUs = g_totSpinUs + spin;
  g_totWaitUs = g_totWaitUs + wait;
  g_totLatUs = g_totLatUs + (g_pushStartUs + g_pushUs - handoff);
}

void primeDma() {
  // Push the (black) buffers once so any lazy allocation inside the bus layer (descriptor
  // chains) happens now and not in the first game frame.
  if (g_fb != nullptr) {
    push(0, TOP_ROWS, g_fb);
    waitPush();
    push(TOP_ROWS, H - TOP_ROWS, g_fb + (size_t)W * TOP_ROWS);
    waitPush();
  } else if (g_band[0] != nullptr && g_band[1] != nullptr) {
    for (int b = 0; b < BANDS; ++b) {
      const int r0 = b * BAND_ROWS;
      const int r1 = (r0 + BAND_ROWS) < H ? (r0 + BAND_ROWS) : H;
      waitPush();
      push(r0, r1 - r0, g_band[b & 1]);
    }
    waitPush();
  }
}

void presentTask(void*) {
  g_task = xTaskGetCurrentTaskHandle();  // the wake timer may fire before the creator returns
  M5.Display.startWrite();  // owned by THIS task from now on, never ended (see file header)
  primeDma();
  xSemaphoreGive(g_semReady);
  for (;;) {
    if (xSemaphoreTake(g_semFrame, portMAX_DELAY) != pdTRUE) continue;
    const uint32_t handoff = g_handoffUs;
    applyBrightness();
    if (g_fb != nullptr) {
      presentFull(handoff);
    } else {
      presentBands(handoff);
    }
    g_totFrames = g_totFrames + 1;
  }
}

}  // namespace

// ------------------------------------------------------------------ public (plat.h)

bool begin(kj::Persist& outPersist) {
  // 1. M5 + keyboard. Serial TX ring is enlarged BEFORE M5 calls Serial.begin().
  detail::serialBegin();
  auto cfg = M5.config();
  cfg.serial_baudrate = 115200;
  cfg.fallback_board = m5::board_t::board_M5CardputerADV;
  cfg.internal_mic = false;  // shares I2S pins with the speaker
  M5Cardputer.begin(cfg, true);

  // 2. Non-blocking serial, M5 log never on the display, boot lines.
  detail::serialSetNonBlocking();
  M5.Log.setLogLevel(m5::log_target_display, ESP_LOG_NONE);
  detail::logBootLines(detail::WORLD_SEED);
  detail::logMem("boot");
  if (!isAdv()) logLine("[WARN] board_not_adv");

  // 3. DMA buffer FIRST, while the heap is least fragmented.
  const bool haveBuffers = detail::displayAllocBuffers();
  detail::logMem("dma");

  // 4. NVS (needed for brightness / volume), then the display.
  detail::storageLoad(outPersist);
  if (!detail::displayBegin(outPersist.bright)) detail::displayHaltRed("display size");
  if (!haveBuffers) return false;

  // 5. Input devices (board / IMU detection, battery) + their boot lines.
  detail::inputBegin();
  detail::logf("[IMU] ok=%d type=%d", imuPresent() ? 1 : 0, (int)M5.Imu.getType());
  detail::logf("[KBD] reader=%s", isAdv() ? "TCA8418" : "IOMatrix");

  // 6. Speaker (task pinned to core 0). The ES8311 is touched over I2C only here, so from
  //    now on the input task (core 0) is the only I2C user.
  detail::audioBegin(outPersist.volume, outPersist.mute != 0);
  detail::inputStartTask();

  // 7. Hand the display to the present task (it opens the transaction and primes the DMA).
  detail::displayStartFrames();
  return true;
}

void present(const uint8_t* px, const uint32_t* lut2x, kj::Profiler& prof) {
  if (g_task == nullptr) return;
  // Blocking take (no spin): returns at once unless the display is the bottleneck.
  prof.begin(kj::ST_DMAWAIT);
  const bool free = xSemaphoreTake(g_semFree, pdMS_TO_TICKS(HANDOFF_TIMEOUT_MS)) == pdTRUE;
  prof.end(kj::ST_DMAWAIT);
  if (!free) {  // present task stuck (bus hang): drop this frame, keep the game alive
    g_totTimeouts = g_totTimeouts + 1;
    return;
  }
  prof.begin(kj::ST_EXPAND);
  memcpy(g_stPx, px, sizeof(g_stPx));
  memcpy(g_stLut, lut2x, sizeof(g_stLut));
  g_handoffUs = micros32();
  xSemaphoreGive(g_semFrame);
  prof.end(kj::ST_EXPAND);
}

// ------------------------------------------------------------------ detail
namespace detail {

bool displayAllocBuffers() {
  g_fb = static_cast<uint16_t*>(heap_caps_aligned_alloc(4, FRAME_BYTES, DMA_CAPS));
  if (g_fb != nullptr) {
    memset(g_fb, 0, FRAME_BYTES);
    return true;
  }
  logf("[ERR] dma_alloc size=%u largest=%lu", (unsigned)FRAME_BYTES,
       (unsigned long)heap_caps_get_largest_free_block(DMA_CAPS));
  for (int i = 0; i < 2; ++i) {
    g_band[i] = static_cast<uint16_t*>(heap_caps_aligned_alloc(4, BAND_BYTES, DMA_CAPS));
    if (g_band[i] != nullptr) memset(g_band[i], 0, BAND_BYTES);
  }
  if (g_band[0] == nullptr || g_band[1] == nullptr) {
    logf("[ERR] dma_alloc size=%u (band) largest=%lu", (unsigned)BAND_BYTES,
         (unsigned long)heap_caps_get_largest_free_block(DMA_CAPS));
    return false;
  }
  logLine("[WARN] dma_fallback");
  return true;
}

bool displayBegin(uint8_t brightness) {
  // M5GFX 0.3.0 board data gives the Cardputer(-ADV) ST7789 width=135 height=240 with
  // offset_rotation=1, so LANDSCAPE is setRotation(0) (internal rotation 1, the MADCTL the
  // older M5 examples got from setRotation(1)). Pick whichever rotation reports 240x135.
  // If the picture is upside down on the device, use rotation + 2 (and recheck IMU signs).
  M5.Display.setRotation(0);
  if (M5.Display.width() != W || M5.Display.height() != H) {
    M5.Display.setRotation(1);
  }
  M5.Display.fillScreen(TFT_BLACK);
  displaySetBrightness(brightness);
  int spiMHz = 40;
#if JET_SPI_MHZ == 80
  {
    auto* bus = static_cast<lgfx::Bus_SPI*>(M5.Display.getPanel()->getBus());
    auto c = bus->config();
    c.freq_write = 80000000;
    bus->config(c);
    bus->init();  // re-init is a no-op for the SPI host/device; only the clock changes
    spiMHz = 80;
  }
#endif
  char mode[24];
  if (g_fb != nullptr) {
    snprintf(mode, sizeof(mode), "ok(%u)", (unsigned)FRAME_BYTES);
  } else if (g_band[0] != nullptr && g_band[1] != nullptr) {
    snprintf(mode, sizeof(mode), "band(2x%u)", (unsigned)BAND_BYTES);
  } else {
    snprintf(mode, sizeof(mode), "none");
  }
  logf("[DISP] rot=%d bri=%d rgb_dma=%s spi=%dMHz w=%d h=%d", (int)M5.Display.getRotation(),
       g_bright, mode, spiMHz, (int)M5.Display.width(), (int)M5.Display.height());
  if (M5.Display.width() != W || M5.Display.height() != H) {
    logf("[ERR] disp_size w=%d h=%d", (int)M5.Display.width(), (int)M5.Display.height());
    return false;
  }
  return true;
}

void displayStartFrames() {
  if (g_fb == nullptr && (g_band[0] == nullptr || g_band[1] == nullptr)) return;
  g_semFrame = xSemaphoreCreateBinaryStatic(&g_semFrameBuf);
  g_semFree = xSemaphoreCreateBinaryStatic(&g_semFreeBuf);
  g_semReady = xSemaphoreCreateBinaryStatic(&g_semReadyBuf);
  xSemaphoreGive(g_semFree);  // the staging buffer starts out free

  esp_timer_create_args_t targs = {};
  targs.callback = onWakeTimer;
  targs.arg = nullptr;
  targs.dispatch_method = ESP_TIMER_TASK;
  targs.name = "present_wake";
  if (esp_timer_create(&targs, &g_wakeTimer) != ESP_OK) {
    logLine("[ERR] present_timer");
    g_wakeTimer = nullptr;
  }
  g_wantBright = g_bright;
  if (g_wakeTimer != nullptr) {
    g_task = xTaskCreateStaticPinnedToCore(presentTask, "present", TASK_STACK, nullptr, TASK_PRIO,
                                           g_taskStack, &g_taskTcb, TASK_CORE);
  }
  if (g_task == nullptr) {
    logLine("[ERR] present_task");
    return;
  }
  const bool ok = xSemaphoreTake(g_semReady, pdMS_TO_TICKS(1000)) == pdTRUE;
  logf("[DISP] present_task ok=%d core=%d prio=%u mode=%s top_us=%lu bot_us=%lu", ok ? 1 : 0,
       (int)TASK_CORE, (unsigned)TASK_PRIO, g_fb != nullptr ? "halves" : "bands",
       (unsigned long)wireUs((size_t)W * TOP_ROWS * 2u),
       (unsigned long)wireUs((size_t)W * (H - TOP_ROWS) * 2u));
}

void displayHaltRed(const char* why) {
  logf("[ERR] halt %s", why ? why : "?");
  // Only reachable from setup, before displayStartFrames (no present task yet): core 1 may
  // still draw. If that ever changes, the task owns the panel and we only log.
  if (g_task == nullptr) {
    M5.Display.waitDMA();
    M5.Display.fillScreen(TFT_RED);
    M5.Display.setTextColor(TFT_WHITE, TFT_RED);
    M5.Display.setCursor(4, 4);
    M5.Display.print("MOONSKIM: ");
    M5.Display.print(why ? why : "fatal");
  }
  for (;;) vTaskDelay(pdMS_TO_TICKS(1000));
}

void displaySetBrightness(uint8_t b) {
  if (g_task != nullptr) {
    g_wantBright = b;  // applied by the present task before its next frame
    return;
  }
  if ((int)b == g_bright) return;
  M5.Display.setBrightness(b);  // setup only (core 1 still owns the display)
  g_bright = b;
  g_wantBright = b;
}

bool displayFullDma() { return g_fb != nullptr; }

void presentTotals(PresentTotals& out) {
  out.frames = g_totFrames;
  out.busyUs = g_totBusyUs;
  out.spinUs = g_totSpinUs;
  out.latUs = g_totLatUs;
  out.waitUs = g_totWaitUs;
  out.timeouts = g_totTimeouts;
  out.busHang = g_totBusHang;
}

uint32_t presentStackFree() {
  return g_task != nullptr ? (uint32_t)uxTaskGetStackHighWaterMark(g_task) : 0u;
}

}  // namespace detail
}  // namespace plat
