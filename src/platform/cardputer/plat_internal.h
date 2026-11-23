// plat_internal.h - private glue between the Cardputer platform .cpp files and main.cpp.
// OWNER: group A. NOT a contract header: nothing in src/core may include it.
#pragma once
#include <stdint.h>
#include "core/input.h"
#include "core/game.h"

namespace plat {
namespace detail {

constexpr uint32_t WORLD_SEED = 0x4D4F4F4Eu;   // "MOON": world seed passed to Game::init

// ---- plat_log.cpp (core 1 only: the log ring is not locked)
void serialBegin();                          // TX ring size + non-blocking writes (before M5 begin)
void serialSetNonBlocking();                 // setTxTimeoutMs(0) (after M5 begin)
#if defined(__GNUC__)
__attribute__((format(printf, 1, 2)))
#endif
void logf(const char* fmt, ...);             // formatted logLine
void logMem(const char* stage);              // [MEM] stage=... line
void logBootLines(uint32_t seed);            // [BOOT] x2
// Reads at most one char from Serial. Platform commands ('?','s','k','h','g','B') are handled
// here and return 0; anything else printable is returned for forwarding to the core.
char serialPoll();
uint32_t droppedLines();

// ---- plat_display.cpp
bool displayAllocBuffers();                  // DMA frame (or band fallback); false = nothing
bool displayBegin(uint8_t brightness);       // rotation, clear, brightness, optional 80 MHz
void displayStartFrames();                   // start the present task (core 0), prime the DMA
void displayHaltRed(const char* why);        // fatal (setup only): red screen + [ERR]
void displaySetBrightness(uint8_t b);        // setup: direct; afterwards handed to the task
bool displayFullDma();                       // true = 64,800 B buffer, false = band fallback
// Present-task totals since boot (monotonic, wrap-safe: use unsigned deltas).
struct PresentTotals {
  uint32_t frames;    // frames pushed
  uint32_t busyUs;    // task CPU: expand + push calls
  uint32_t spinUs;    // task spin on the SPI busy bit after the timed sleep
  uint32_t latUs;     // hand-off -> predicted end of the frame's last push
  uint32_t waitUs;    // waited for the previous frame's last push (display-bound)
  uint32_t timeouts;  // core-1 hand-offs that timed out (frame dropped)
  uint32_t busHang;   // pushes that never finished within 40 ms
};
void presentTotals(PresentTotals& out);
uint32_t presentStackFree();                 // bytes never used (0 = no task)

// ---- plat_input.cpp
void inputBegin();                           // board/IMU detection, first battery read
void inputStartTask();                       // input task on core 0 (all I2C from then on)
const kj::ImuSample& lastImu();              // last device-frame sample delivered to the core
uint64_t lastKeysDown();
uint16_t batteryMv();
uint8_t batteryPct();
struct InputTotals {
  uint32_t polls;     // completed device polls (input task)
  uint32_t pollUs;    // their total duration on core 0
  uint32_t stale;     // frames that got no new poll (fresh = false because none completed)
};
void inputTotals(InputTotals& out);
uint32_t inputStackFree();                   // bytes never used (0 = no task)

// ---- plat_audio.cpp
bool audioBegin(uint8_t volume, bool mute);  // speaker task pinned to core 0, begin()
bool audioOk();
uint8_t audioAppliedVolume();

// ---- plat_storage.cpp
bool storageLoad(kj::Persist& out);          // false = defaults were used

}  // namespace detail
}  // namespace plat
