// hud.cpp - 3x5 text, play HUD, popups, menus and screens (DESIGN.md section 15).
// OWNER: group D. Logical pixels, pal::HUD_* indices only, no depth test.
#include "hud.h"
#include <stdio.h>
#include <string.h>
#include "config.h"
#include "mathx.h"
#include "palette.h"
#include "font3x5.h"
#include "fastmath_d.h"

namespace kj {

namespace {

// Tiny bounded string builder: the HUD's integer formatting without snprintf (newlib's printf
// core is several KB of flash code that missed the I-cache on every frame). Output is the same
// as the "%d" / "%u" / "%s" conversions it replaces; it truncates like snprintf.
struct Fmt {
  char* p;
  char* end;  // last usable char (room for the terminator)
  Fmt(char* buf, int n) : p(buf), end(buf + (n > 0 ? n - 1 : 0)) { if (n > 0) *p = 0; }
  Fmt& s(const char* t) {
    while (*t && p < end) *p++ = *t++;
    *p = 0;
    return *this;
  }
  Fmt& u(uint32_t v) {
    char tmp[12];
    int k = 0;
    do {
      tmp[k++] = (char)('0' + (int)(v % 10u));
      v /= 10u;
    } while (v);
    while (k > 0 && p < end) *p++ = tmp[--k];
    *p = 0;
    return *this;
  }
  Fmt& i(int v) {
    if (v < 0) {
      if (p < end) *p++ = '-';
      *p = 0;
      return u((uint32_t)0 - (uint32_t)v);
    }
    return u((uint32_t)v);
  }
};

inline void put(FrameBuf& fb, int x, int y, uint8_t c) {
  if (inFrame(x, y)) fb.px[y * LW + x] = c;
}

void fillRect(FrameBuf& fb, int x, int y, int w, int h, uint8_t c) {
  int x0 = x < 0 ? 0 : x, y0 = y < 0 ? 0 : y;
  int x1 = x + w > LW ? LW : x + w, y1 = y + h > LH ? LH : y + h;
  for (int yy = y0; yy < y1; ++yy)
    for (int xx = x0; xx < x1; ++xx) fb.px[yy * LW + xx] = c;
}

// Bayer-dithered panel (alpha16 coverage) used behind menus and screens.
void ditherRect(FrameBuf& fb, int x, int y, int w, int h, uint8_t c, int a16) {
  int x0 = x < 0 ? 0 : x, y0 = y < 0 ? 0 : y;
  int x1 = x + w > LW ? LW : x + w, y1 = y + h > LH ? LH : y + h;
  for (int yy = y0; yy < y1; ++yy)
    for (int xx = x0; xx < x1; ++xx)
      if (bayerPass(xx, yy, a16)) fb.px[yy * LW + xx] = c;
}

void frameRect(FrameBuf& fb, int x, int y, int w, int h, uint8_t c) {
  for (int i = 0; i < w; ++i) { put(fb, x + i, y, c); put(fb, x + i, y + h - 1, c); }
  for (int j = 0; j < h; ++j) { put(fb, x, y + j, c); put(fb, x + w - 1, y + j, c); }
}

void drawGlyph(FrameBuf& fb, int x, int y, char ch, uint8_t color, int scale) {
  uint16_t g = fontGlyph(ch);
  if (!g) return;
  if (scale == 1 && x >= 0 && y >= 0 && x + FONT_W <= LW && y + FONT_H <= LH) {
    // common case: a 1x glyph fully inside the frame, no per-pixel clipping
    uint8_t* row = fb.px + y * LW + x;
    for (int r = 0; r < FONT_H; ++r, row += LW) {
      const int bits = (g >> (3 * (4 - r))) & 7;
      if (bits & 4) row[0] = color;
      if (bits & 2) row[1] = color;
      if (bits & 1) row[2] = color;
    }
    return;
  }
  for (int row = 0; row < FONT_H; ++row) {
    int bits = (g >> (3 * (4 - row))) & 7;
    if (!bits) continue;
    for (int col = 0; col < FONT_W; ++col) {
      if (!(bits & (4 >> col))) continue;
      if (scale == 1) {
        put(fb, x + col, y + row, color);
      } else {
        for (int sy = 0; sy < scale; ++sy)
          for (int sx = 0; sx < scale; ++sx) put(fb, x + col * scale + sx, y + row * scale + sy, color);
      }
    }
  }
}

// blink helper: true during the "on" half of a period of 1/hz seconds
inline bool blinkOn(float t, float hz) {
  float p = t * hz;
  return p - fm::floorF(p) < 0.5f;
}

void textRight(FrameBuf& fb, int xRight, int y, const char* s, uint8_t c, int scale = 1) {
  hudText(fb, xRight - hudTextWidth(s, scale) + 1, y, s, c, true, scale);
}

// Small integer / fixed formatting without floating-point printf (no libc float paths).
void fmtTenths(char* buf, int n, float v) {
  if (v < 0.0f) v = 0.0f;
  int t = (int)(v * 10.0f);
  Fmt(buf, n).i(t / 10).s(".").i(t % 10);
}

// ---------------------------------------------------------------- play HUD parts
void drawBoostBar(FrameBuf& fb, const HudData& h) {
  const int x = 2, y = LH - 10, w = 24;
  bool empty = h.boostEnergy < 0.10f;
  uint8_t frame = (empty && blinkOn(h.time, 4.0f)) ? pal::HUD_RED : pal::HUD_SHADOW;
  fillRect(fb, x - 1, y - 1, w + 2, 4, frame);
  int fill = (int)(sat(h.boostEnergy) * (float)w + 0.5f);
  fillRect(fb, x, y, fill, 2, pal::HUD_AMBER);
  char buf[24];
  Fmt(buf, (int)sizeof buf).s("SPD ").i((int)(h.speedKt + 0.5f));
  hudText(fb, x, LH - 6, buf, pal::HUD_WHITE);
}

void drawBadge(FrameBuf& fb, int x, int y, TiltBadge b, float t) {
  char s[2] = {'T', 0};
  uint8_t c = pal::HUD_GREEN;
  switch (b) {
    case TiltBadge::TiltOk: s[0] = 'T'; c = pal::HUD_GREEN; break;
    case TiltBadge::NeedsCal: s[0] = 'T'; c = pal::HUD_AMBER; break;
    case TiltBadge::Keys: s[0] = 'K'; c = pal::HUD_WHITE; break;
    case TiltBadge::NoImu: s[0] = 'K'; c = pal::HUD_CYAN; break;
    case TiltBadge::Guard: s[0] = '?'; c = blinkOn(t, 3.0f) ? pal::HUD_RED : pal::HUD_AMBER; break;
  }
  fillRect(fb, x - 1, y - 1, 5, 7, pal::HUD_SHADOW);
  hudText(fb, x, y, s, c, false);
}

void drawBattery(FrameBuf& fb, int x, int y, int pct, float t) {
  uint8_t c = blinkOn(t, 1.0f) ? pal::HUD_RED : pal::HUD_SHADOW;
  frameRect(fb, x, y, 7, 4, c);
  put(fb, x + 7, y + 1, c);
  put(fb, x + 7, y + 2, c);
  if (pct > 7) fillRect(fb, x + 1, y + 1, 1, 2, pal::HUD_RED);
}

// 5 px chevron pointing toward the off-screen ring, drawn 2 px thick with a shadow.
void drawChevron(FrameBuf& fb, int x, int y, int dir, uint8_t c) {
  // dir: 0 right, 1 left, 2 up, 3 down
  static const int8_t PTS[5][2] = {{-2, -2}, {-1, -1}, {0, 0}, {-1, 1}, {-2, 2}};
  for (int pass = 0; pass < 2; ++pass) {
    uint8_t col = pass == 0 ? pal::HUD_SHADOW : c;
    int o = pass == 0 ? 1 : 0;
    for (int i = 0; i < 5; ++i) {
      for (int k = 0; k < 2; ++k) {
        int a = PTS[i][0] - k, b = PTS[i][1];
        int px = 0, py = 0;
        switch (dir) {
          case 0: px = a; py = b; break;
          case 1: px = -a; py = b; break;
          case 2: px = b; py = -a; break;
          default: px = b; py = a; break;
        }
        put(fb, x + px + o, y + py + o, col);
      }
    }
  }
}

void drawPlay(FrameBuf& fb, const HudData& h) {
  char buf[24];
  // top-left: score + combo
  Fmt(buf, (int)sizeof buf).u(h.score);
  hudText(fb, 2, 2, buf, pal::HUD_WHITE);
  if (h.combo > 1) {
    Fmt(buf, (int)sizeof buf).s("X").i((int)h.combo);
    bool pop = h.comboPulse > 0.5f;
    hudText(fb, 2, pop ? 7 : 8, buf, pop ? pal::HUD_WHITE : pal::HUD_AMBER, true, 1);
  }
  // top-right: timer (Ring Run)
  if (h.timeLeft >= 0.0f) {
    fmtTenths(buf, sizeof buf, h.timeLeft);
    bool low = h.timeLeft < 5.0f;
    if (!low || blinkOn(h.time, 4.0f) || h.timeLeft <= 0.0f)
      textRight(fb, LW - 2, 2, buf, low ? pal::HUD_RED : pal::HUD_WHITE, 2);  // fail-state readout: big
  }
  if (h.batteryPct > 0 && h.batteryPct < 15)
    drawBattery(fb, LW - 11, h.timeLeft >= 0.0f ? 14 : 9, h.batteryPct, h.time);

  // bottom-left: boost bar + speed
  drawBoostBar(fb, h);

  // bottom-right: altitude, SKIM, tilt badge
  Fmt(buf, (int)sizeof buf).s("ALT ").i((int)(h.alt > 0.0f ? h.alt + 0.5f : 0.0f));
  textRight(fb, LW - 2, LH - 6, buf, pal::HUD_WHITE);
  drawBadge(fb, LW - 5, LH - 13, h.tilt, h.time);
  if (h.skimming) hudText(fb, LW - 24, LH - 12, "SKIM", pal::HUD_AMBER);

  // ring guide chevron
  if (h.guide) {
    int dir = 0;
    if (h.guideX <= 5) dir = 1;
    else if (h.guideX >= LW - 6) dir = 0;
    else if (h.guideY <= 12) dir = 2;
    else dir = 3;
    drawChevron(fb, h.guideX, h.guideY, dir, pal::HUD_AMBER);
    if (h.guideVertical != 0) {
      char s[2] = {h.guideVertical > 0 ? GLYPH_UP : GLYPH_DOWN, 0};
      int tx = dir == 1 ? h.guideX + 3 : h.guideX - 6;
      if (dir >= 2) tx = h.guideX + 4;
      hudText(fb, tx, h.guideY - 2, s, pal::HUD_CYAN);
    }
  }

  // GPWS
  if (h.pullUp && blinkOn(h.time, 4.0f)) hudTextCentered(fb, LW / 2, 14, "PULL UP", pal::HUD_RED);
}

void drawPopups(FrameBuf& fb, const HudData& h) {
  for (int i = 0; i < HUD_POPUPS; ++i) {
    const Popup& p = h.popups[i];
    if (!p.text[0] || p.age > 0.8f) continue;
    int y = p.y - (int)(6.0f * (p.age / 0.8f) + 0.5f);
    if (p.age > 0.6f && !blinkOn(p.age, 12.0f)) continue;  // flicker out
    hudTextCentered(fb, p.x, y, p.text, p.color);
  }
  if (h.centre[0] && h.centreAge < 1.2f) {
    bool show = h.centreAge < 0.9f || blinkOn(h.centreAge, 10.0f);
    if (show) {
      int len = (int)strlen(h.centre);
      int scale = (len * FONT_ADV * 2 <= LW - 8) ? 2 : 1;
      int y = h.pullUp ? 22 : 18;
      if (h.centreAge < 0.08f && scale == 2) y -= 1;  // tiny pop on appearance
      hudTextCentered(fb, LW / 2, y, h.centre, pal::HUD_AMBER, true, scale);
    }
  }
}

// ---------------------------------------------------------------- screens
void drawCalibrate(FrameBuf& fb, const HudData& h) {
  ditherRect(fb, 0, 0, LW, LH, pal::HUD_SHADOW, 8);
  const int cx = LW / 2, cy = 27, R = 13;
  // progress ring: 24 dots, lit clockwise from the top
  int lit = (int)(sat(h.calibProgress) * 24.0f + 0.5f);
  for (int i = 0; i < 24; ++i) {
    float a = (float)i * (TAU / 24.0f) - PI * 0.5f;
    int x = cx + (int)lroundf(cosf(a) * (float)R), y = cy + (int)lroundf(sinf(a) * (float)R);
    put(fb, x, y, i < lit ? pal::HUD_GREEN : pal::HUD_WHITE);
  }
  // crosshair + live tilt dot
  for (int i = -3; i <= 3; ++i) {
    if (i == 0) continue;
    if (i & 1) { put(fb, cx + i, cy, pal::HUD_CYAN); put(fb, cx, cy + i, pal::HUD_CYAN); }
  }
  int dx = cx + (int)lroundf(clampf(h.dotX, -1.0f, 1.0f) * 9.0f);
  int dy = cy - (int)lroundf(clampf(h.dotY, -1.0f, 1.0f) * 9.0f);
  fillRect(fb, dx - 1, dy - 1, 2, 2, pal::HUD_AMBER);
  hudTextCentered(fb, cx, 46, "HOLD COMFORTABLY", pal::HUD_WHITE);
  if (h.calibTimedOut) {
    if (blinkOn(h.time, 2.0f)) hudTextCentered(fb, cx, 6, "HOLD STILL", pal::HUD_RED);
  } else {
    hudTextCentered(fb, cx, 6, "CALIBRATING TILT", pal::HUD_CYAN);
  }
  hudTextCentered(fb, cx, 56, "ENTER SKIP", pal::HUD_AMBER);
}

void drawTitle(FrameBuf& fb, const HudData& h) {
  ditherRect(fb, 0, 3, LW, 22, pal::HUD_SHADOW, 10);
  hudTextCentered(fb, LW / 2, 5, "MOONSKIM", pal::HUD_WHITE, true, 2);
  char buf[24];
  Fmt(buf, (int)sizeof buf).s("BEST ").u(h.best);
  if (h.bestCombo > 1) {
    char b2[48];
    Fmt(b2, (int)sizeof b2).s(buf).s("  COMBO X").i((int)h.bestCombo);
    hudTextCentered(fb, LW / 2, 18, b2, pal::HUD_AMBER);
  } else {
    hudTextCentered(fb, LW / 2, 18, buf, pal::HUD_AMBER);
  }
  ditherRect(fb, 0, 44, LW, 24, pal::HUD_SHADOW, 10);
  hudTextCentered(fb, LW / 2, 45, "ENTER RING RUN", blinkOn(h.time, 1.2f) ? pal::HUD_WHITE : pal::HUD_AMBER);
  hudTextCentered(fb, LW / 2, 52, "2 FREE FLIGHT", pal::HUD_CYAN);
  hudTextCentered(fb, LW / 2, 59, "H KEYS", pal::HUD_WHITE);
  if (h.centre[0] && h.centreAge < 2.0f && blinkOn(h.centreAge, 2.0f))
    hudTextCentered(fb, LW / 2, 30, h.centre, pal::HUD_RED);
}

void drawCountdown(FrameBuf& fb, const HudData& h) {
  char buf[4];
  if (h.countdown > 0) {
    buf[0] = (char)('0' + h.countdown);
    buf[1] = 0;
  } else {
    buf[0] = 'G'; buf[1] = 'O'; buf[2] = '!'; buf[3] = 0;
  }
  hudTextCentered(fb, LW / 2, 16, buf, pal::HUD_AMBER, true, 2);
}

void drawPause(FrameBuf& fb, const HudData& h) {
  const int px = 16, pw = LW - 32;
  ditherRect(fb, px, 1, pw, LH - 2, pal::HUD_SHADOW, 8);
  frameRect(fb, px, 1, pw, LH - 2, pal::HUD_SHADOW);
  hudText(fb, px + 3, 3, "PAUSED", pal::HUD_AMBER);
  if (h.batteryPct > 0) {
    char b[12];
    Fmt(b, (int)sizeof b).s("BAT ").i((int)h.batteryPct).s("%");
    textRight(fb, px + pw - 3, 3, b, h.batteryPct < 15 ? pal::HUD_RED : pal::HUD_WHITE);
  }
  for (int i = 0; i < MENU_ITEMS; ++i) {
    int y = 10 + i * 6;
    bool sel = i == h.menuSel;
    const char* lab = h.menuLabels[i] ? h.menuLabels[i] : "";
    const char* val = h.menuValues[i] ? h.menuValues[i] : "";
    if (sel) {
      fillRect(fb, px + 2, y - 1, pw - 4, 7, pal::HUD_SHADOW);
      hudText(fb, px + 3, y, ">", pal::HUD_AMBER, false);
    }
    hudText(fb, px + 8, y, lab, sel ? pal::HUD_AMBER : pal::HUD_WHITE, !sel);
    if (val[0]) textRight(fb, px + pw - 4, y, val, sel ? pal::HUD_WHITE : pal::HUD_CYAN);
  }
}

void drawHelp(FrameBuf& fb, const HudData& h) {
  (void)h;
  ditherRect(fb, 0, 0, LW, LH, pal::HUD_SHADOW, 12);
  static const char* const LINES[] = {
    "TILT OR KEYS TO FLY",
    "A D , /    TURN",
    "W S ; .    CLIMB DIVE",
    "SPACE ALT  BOOST",
    "Z X  L '   BARREL ROLL",
    "ENTER ROLL   P PAUSE",
    "C CALIBRATE  T TILT",
    "I INVERT     V VIEW",
    "N TIME   M - = SOUND",
    "[ ] BRIGHT   F FPS",
  };
  hudTextCentered(fb, LW / 2, 1, "KEYS", pal::HUD_AMBER);
  for (int i = 0; i < 10; ++i)
    hudText(fb, 4, 8 + i * 6, LINES[i], i == 0 ? pal::HUD_CYAN : pal::HUD_WHITE);
}

void drawGameOver(FrameBuf& fb, const HudData& h) {
  ditherRect(fb, 10, 4, LW - 20, LH - 8, pal::HUD_SHADOW, 10);
  hudTextCentered(fb, LW / 2, 7, "GAME OVER", pal::HUD_RED, true, 2);
  char buf[40];
  Fmt(buf, (int)sizeof buf).s("SCORE ").u(h.lastScore);
  hudTextCentered(fb, LW / 2, 21, buf, pal::HUD_WHITE);
  Fmt(buf, (int)sizeof buf).s("BEST ").u(h.best);
  hudTextCentered(fb, LW / 2, 28, buf, pal::HUD_AMBER);
  Fmt(buf, (int)sizeof buf).s("RINGS ").i((int)h.lastRings).s("  COMBO X").i((int)h.bestCombo);
  hudTextCentered(fb, LW / 2, 35, buf, pal::HUD_CYAN);
  if (h.newBest && blinkOn(h.time, 3.0f)) hudTextCentered(fb, LW / 2, 44, "NEW BEST!", pal::HUD_GREEN, true, 1);
  hudTextCentered(fb, LW / 2, 55, "ENTER RETRY  BKSP MENU", pal::HUD_WHITE);
}

void drawStats(FrameBuf& fb, const HudData& h) {
  char buf[24];
  int ms10 = (int)(h.coreMs * 10.0f + 0.5f);
  Fmt(buf, (int)sizeof buf).i((int)(h.fps + 0.5f)).s(" ").i(ms10 / 10).s(".").i(ms10 % 10).s("MS");
  hudTextCentered(fb, LW / 2, 2, buf, pal::HUD_GREEN);
}

void drawImuDebug(FrameBuf& fb, const HudData& h) {
  const int cx = LW / 2, cy = 17, r = 7;
  ditherRect(fb, cx - r - 1, cy - r - 1, 2 * r + 3, 2 * r + 3, pal::HUD_SHADOW, 10);
  frameRect(fb, cx - r - 1, cy - r - 1, 2 * r + 3, 2 * r + 3, pal::HUD_CYAN);
  put(fb, cx, cy, pal::HUD_CYAN);
  int dx = cx + (int)lroundf(clampf(h.dotX, -1.0f, 1.0f) * (float)r);
  int dy = cy - (int)lroundf(clampf(h.dotY, -1.0f, 1.0f) * (float)r);
  put(fb, dx, dy, pal::HUD_AMBER);
  put(fb, dx + 1, dy, pal::HUD_AMBER);
  put(fb, dx, dy + 1, pal::HUD_AMBER);
  put(fb, dx + 1, dy + 1, pal::HUD_AMBER);
}

}  // namespace

// ================================================================ public API
void hudText(FrameBuf& fb, int x, int y, const char* s, uint8_t color, bool shadow, int scale) {
  if (!s) return;
  if (scale < 1) scale = 1;
  const int adv = FONT_ADV * scale;
  if (shadow) {
    int cx = x;
    for (const char* p = s; *p; ++p, cx += adv) drawGlyph(fb, cx + 1, y + 1, *p, pal::HUD_SHADOW, scale);
  }
  int cx = x;
  for (const char* p = s; *p; ++p, cx += adv) drawGlyph(fb, cx, y, *p, color, scale);
}

int hudTextWidth(const char* s, int scale) {
  if (!s || !*s) return 0;
  if (scale < 1) scale = 1;
  int n = (int)strlen(s);
  return n * FONT_ADV * scale - scale;
}

void hudTextCentered(FrameBuf& fb, int cx, int y, const char* s, uint8_t color, bool shadow,
                     int scale) {
  int w = hudTextWidth(s, scale);
  hudText(fb, cx - w / 2, y, s, color, shadow, scale);
}

void drawHud(FrameBuf& fb, const HudData& h) {
  if (h.showPlay) {
    drawPlay(fb, h);
  }
  switch (h.screen) {
    case HudScreen::None: break;
    case HudScreen::Calibrate: drawCalibrate(fb, h); break;
    case HudScreen::Title: drawTitle(fb, h); break;
    case HudScreen::Countdown: drawCountdown(fb, h); break;
    case HudScreen::Pause: drawPause(fb, h); break;
    case HudScreen::Help: drawHelp(fb, h); break;
    case HudScreen::GameOver: drawGameOver(fb, h); break;
  }
  if (h.showPlay && (h.screen == HudScreen::None || h.screen == HudScreen::Countdown))
    drawPopups(fb, h);
  if (h.imuDebug && h.screen != HudScreen::Calibrate) drawImuDebug(fb, h);
  if (h.stats) drawStats(fb, h);
}

}  // namespace kj
