// plat.h - Cardputer-Adv platform layer (Arduino + M5Cardputer/M5Unified/M5GFX).
// OWNER: group A. The ONLY code that touches M5.Display, M5.Imu, the keyboard, M5.Speaker,
// Preferences and Serial. Threads after setup (one owner per resource):
//   core 1  Arduino loop task: the game, Serial (log ring + commands), Speaker pump, NVS.
//   core 0  "present" task (prio 5): every M5.Display call; holds the SPI transaction.
//   core 0  "input" task (prio 3): every I2C transfer (TCA8418 keyboard, BMI270), BtnA,
//           battery ADC. The ES8311 is configured over I2C only inside Speaker.begin (setup).
//   core 0  M5 speaker mixer task (prio 2).
#pragma once
#include <stdint.h>
#include "core/config.h"
#include "core/input.h"
#include "core/game.h"

namespace plat {

// M5Cardputer.begin (fallback board = ADV, mic off), Serial (non-blocking), display
// (landscape = rotation 0 on M5GFX 0.3.0, brightness, optional 80 MHz bus), DMA frame
// buffer (64,800 B, MALLOC_CAP_DMA | MALLOC_CAP_INTERNAL, allocated FIRST), speaker (task
// pinned to core 0, begin()), NVS load,
// boot log lines [BOOT]/[MEM]/[DISP]/[IMU]/[KBD]/[SPK]/[INPUT]. Starts the input task and the
// present task (which opens startWrite() for good and primes the DMA).
// Returns false only if nothing can be displayed (then main halts with a red screen + [ERR]).
bool begin(kj::Persist& outPersist);

bool isAdv();          // M5.getBoard() == board_M5CardputerADV
bool imuPresent();     // ADV and M5.Imu.getType() != none

// Takes the latest snapshot of the input task (keys down + every key seen since the last
// call, BtnA + wasPressed, newest IMU sample with fresh = new since the last call, battery);
// never waits for I2C. One char from Serial (platform commands are consumed here; core
// commands are forwarded in in.serialCmd). Records ST_INPUT (~0.02 ms).
void pollInput(kj::InputFrame& in, kj::Game& game);

// Start one device poll on the input task (core 0, ~1 ms of I2C). main.cpp calls it from the
// pacing wait ~1.5-2.6 ms before the next frame (right away when late or uncapped), so the
// poll is complete when pollInput runs. Harmless if a poll is still running.
void inputKick();

// Hands the frame to the present task (core 0): blocks (no spin) until the staging buffer is
// free (-> ST_DMAWAIT, 0 unless display-bound), copies px + lut2x into it (-> ST_EXPAND,
// ~0.03 ms) and returns. The task expands and pushes it while core 1 renders the next frame.
void present(const uint8_t* px, const uint32_t* lut2x, kj::Profiler& prof);

// Refill the speaker stream: while M5.Speaker.isPlaying(ch) < 2, synth.render() the next of 3
// rotating buffers and playRaw(buf, 384, 16000, false, 1, ch, false). Records ST_AUDIO.
void audioPump(kj::Synth& synth, kj::Profiler& prof);

// Apply brightness / volume / mute when they differ from the last applied values.
void applySettings(const kj::Game& game);

void savePersist(const kj::Persist& p);   // Preferences "moonskim", key "p" (blob)
void logLine(const char* s);              // core 1 only; queued in a 4 KB ring (see plat_log)
void logFlush();                          // move queued log bytes to USB (non-blocking)
uint32_t micros32();

// Once-per-second [STAT] line, [IMU] stream (20 Hz when game.imuStream()), [KBD] key echo.
void periodicLogs(kj::Game& game, float dt);

// Serial screenshot: [SHOT] begin w=120 h=68 fmt=idx8+pal888, base64 lines, [SHOT] end crc=.
void dumpShot(const uint8_t* px, const uint8_t* rgb888);

}  // namespace plat
