// hud.h - 3x5 pixel font, HUD overlay, popups, menus, title/calibration/game-over screens.
// OWNER: group D (hud.cpp, font3x5.h). Draws in LOGICAL pixels with pal::HUD_* indices only
// (HUD is exempt from flash/mist/fade). No depth test.
#pragma once
#include <stdint.h>
#include "frame.h"

namespace kj {

constexpr int FONT_W = 3, FONT_H = 5, FONT_ADV = 4;  // glyph advance incl. 1 px spacing

// Printable ASCII 32..95 (upper case; lower case is folded to upper) + a few symbols.
void hudText(FrameBuf& fb, int x, int y, const char* s, uint8_t color, bool shadow = true,
             int scale = 1);
int hudTextWidth(const char* s, int scale = 1);
void hudTextCentered(FrameBuf& fb, int cx, int y, const char* s, uint8_t color, bool shadow = true,
                     int scale = 1);

enum class HudScreen : uint8_t { None, Calibrate, Title, Countdown, Pause, Help, GameOver };
enum class TiltBadge : uint8_t { Keys, TiltOk, NeedsCal, NoImu, Guard };

constexpr int HUD_POPUPS = 4;
struct Popup {
  char text[12];
  float age;        // s; removed at 0.8 s
  int16_t x, y;     // spawn position (logical px), rises 6 px over its life
  uint8_t color;
};

constexpr int MENU_ITEMS = 9;
struct HudData {
  HudScreen screen;
  // play HUD
  bool showPlay;
  uint32_t score;
  uint8_t combo;           // 1..8
  float comboPulse;        // 0..1, decays
  float timeLeft;          // s; < 0 hides the timer (free flight)
  uint16_t ring;           // rings passed
  float speedKt;           // displayed speed
  float alt;               // height above surface (u)
  float boostEnergy;       // 0..1
  bool skimming;
  bool pullUp;
  TiltBadge tilt;
  uint8_t batteryPct;      // icon shown below 15
  // ring guide (off-screen chevron)
  bool guide;
  int16_t guideX, guideY;
  int8_t guideVertical;    // -1 below, 0, +1 above
  // stats overlay (f)
  bool stats;
  float fps, coreMs;
  // IMU debug overlay (g) and calibration screen
  bool imuDebug;
  float dotX, dotY;        // -1..1 tilt dot (turn, -pitch)
  float calibProgress;     // 0..1
  bool calibTimedOut;
  // popups and centre message
  Popup popups[HUD_POPUPS];
  char centre[16];         // e.g. "PERFECT" "PULL UP" "CRASH" "GAME OVER"; empty = none
  float centreAge;
  // title / game over
  uint32_t best;
  uint32_t lastScore;
  uint16_t lastRings;
  uint8_t bestCombo;
  bool newBest;
  int8_t countdown;        // 3,2,1,0(GO)
  // pause menu
  int8_t menuSel;
  const char* menuLabels[MENU_ITEMS];
  const char* menuValues[MENU_ITEMS];
  float time;              // for blinking
};

void drawHud(FrameBuf& fb, const HudData& h);

}  // namespace kj
