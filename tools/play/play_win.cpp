// play_win.cpp - interactive Windows preview of MOONSKIM.
//
// Runs the SAME portable core (src/core) as the Cardputer firmware in a Win32 window:
// the 240x135 panel image (device-exact expand2x output) is scaled up with nearest-neighbour,
// the keyboard is mapped to the Cardputer key positions, the mouse stands in for the IMU
// (mouse offset from the window centre = tilt of the device), and the synth plays through
// waveOut at 16 kHz.
//
// Build: tools\play\build.bat  ->  tools\play\out\moonskim_pc.exe
// Usage: moonskim_pc.exe [--keys]   (--keys: no fake IMU, keyboard-only flying)
#define _CRT_SECURE_NO_WARNINGS
#define WIN32_LEAN_AND_MEAN
#define NOMINMAX
#include <windows.h>
#include <mmsystem.h>

#include <stdarg.h>
#include <stdio.h>
#include <string.h>

#include <string>

#include "game.h"
#include "expand.h"
#include "../sim/synth_imu.h"

#pragma comment(lib, "user32.lib")
#pragma comment(lib, "gdi32.lib")
#pragma comment(lib, "winmm.lib")

namespace {

constexpr int PW = kj::cfg::SCREEN_W;   // 240
constexpr int PH = kj::cfg::SCREEN_H;   // 135
constexpr uint32_t SEED = 0x4D4F4F4Eu;  // same world as the device ("MOON")
constexpr float MOUSE_ROLL_MAX = 40.0f;   // deg of fake device roll at the window edge
constexpr float MOUSE_PITCH_MAX = 30.0f;  // deg of fake device pitch at the window edge

kj::Game g_game;  // ~20 KB, static as on the device
uint16_t g_panel[PW * PH];
uint32_t g_bgra[PW * PH];
LARGE_INTEGER g_qpf, g_qp0;
std::string g_persistPath;
HWND g_hwnd = nullptr;
bool g_mouseTilt = true;  // F2 toggles
bool g_running = true;
uint64_t g_keysDown = 0, g_keysPressed = 0;
char g_serialCmd = 0;

uint32_t nowMicros() {
  LARGE_INTEGER t;
  QueryPerformanceCounter(&t);
  return (uint32_t)((uint64_t)(t.QuadPart - g_qp0.QuadPart) * 1000000ull / (uint64_t)g_qpf.QuadPart);
}

void logLine(const char* line) {
  fputs(line, stdout);
  fputc('\n', stdout);
}

void savePersist(const kj::Persist& p) {
  FILE* f = fopen(g_persistPath.c_str(), "wb");
  if (!f) return;
  fwrite(&p, sizeof(p), 1, f);
  fclose(f);
}

kj::Persist loadPersist() {
  kj::Persist p = kj::defaultPersist();
  FILE* f = fopen(g_persistPath.c_str(), "rb");
  if (!f) return p;
  kj::Persist q;
  if (fread(&q, sizeof(q), 1, f) == 1 && q.version == kj::PERSIST_VERSION) p = q;
  fclose(f);
  return p;
}

// Windows virtual key -> Cardputer physical key position (-1 = unmapped).
int vkToKp(WPARAM vk) {
  if (vk >= 'A' && vk <= 'Z') {
    static const uint8_t letters[26] = {
        kj::KP_A, kj::KP_B, kj::KP_C, kj::KP_D, kj::KP_E, kj::KP_F, kj::KP_G, kj::KP_H, kj::KP_I,
        kj::KP_J, kj::KP_K, kj::KP_L, kj::KP_M, kj::KP_N, kj::KP_O, kj::KP_P, kj::KP_Q, kj::KP_R,
        kj::KP_S, kj::KP_T, kj::KP_U, kj::KP_V, kj::KP_W, kj::KP_X, kj::KP_Y, kj::KP_Z};
    return letters[vk - 'A'];
  }
  if (vk >= '1' && vk <= '9') return kj::KP_1 + (int)(vk - '1');
  switch (vk) {
    case '0': return kj::KP_0;
    case VK_LEFT: return kj::KP_COMMA;   // same as the Cardputer arrow legends
    case VK_RIGHT: return kj::KP_SLASH;
    case VK_UP: return kj::KP_SEMI;
    case VK_DOWN: return kj::KP_DOT;
    case VK_OEM_COMMA: return kj::KP_COMMA;
    case VK_OEM_PERIOD: return kj::KP_DOT;
    case VK_OEM_2: return kj::KP_SLASH;
    case VK_OEM_1: return kj::KP_SEMI;
    case VK_OEM_7: return kj::KP_QUOTE;
    case VK_OEM_MINUS: return kj::KP_MINUS;
    case VK_OEM_PLUS: return kj::KP_EQUAL;
    case VK_OEM_4: return kj::KP_LBRACK;
    case VK_OEM_6: return kj::KP_RBRACK;
    case VK_OEM_3: return kj::KP_BACKTICK;
    case VK_OEM_5: return kj::KP_BSLASH;
    case VK_BACK: return kj::KP_BKSP;
    case VK_ESCAPE: return kj::KP_P;      // pause
    case VK_TAB: return kj::KP_TAB;
    case VK_RETURN: return kj::KP_ENTER;
    case VK_SPACE: return kj::KP_SPACE;
    case VK_SHIFT: return kj::KP_SHIFT;
    case VK_CONTROL: return kj::KP_CTRL;
    case VK_MENU: return kj::KP_ALT;
    default: return -1;
  }
}

// ------------------------------------------------------------------ audio (waveOut)
constexpr int AUD_BUFS = 6;
constexpr int AUD_LEN = 320;  // 20 ms each at 16 kHz -> ~120 ms queue
HWAVEOUT g_wave = nullptr;
WAVEHDR g_hdr[AUD_BUFS];
int16_t g_audio[AUD_BUFS][AUD_LEN];

void audioOpen() {
  WAVEFORMATEX wf = {};
  wf.wFormatTag = WAVE_FORMAT_PCM;
  wf.nChannels = 1;
  wf.nSamplesPerSec = kj::Synth::RATE;
  wf.wBitsPerSample = 16;
  wf.nBlockAlign = 2;
  wf.nAvgBytesPerSec = wf.nSamplesPerSec * wf.nBlockAlign;
  if (waveOutOpen(&g_wave, WAVE_MAPPER, &wf, 0, 0, CALLBACK_NULL) != MMSYSERR_NOERROR) {
    g_wave = nullptr;
    logLine("[PC] no audio device, running silent");
    return;
  }
  for (int i = 0; i < AUD_BUFS; ++i) {
    memset(&g_hdr[i], 0, sizeof(WAVEHDR));
    memset(g_audio[i], 0, sizeof(g_audio[i]));
    g_hdr[i].lpData = (LPSTR)g_audio[i];
    g_hdr[i].dwBufferLength = sizeof(g_audio[i]);
    waveOutPrepareHeader(g_wave, &g_hdr[i], sizeof(WAVEHDR));
    waveOutWrite(g_wave, &g_hdr[i], sizeof(WAVEHDR));  // prime with silence
  }
}

void audioPump() {
  if (!g_wave) return;
  for (int i = 0; i < AUD_BUFS; ++i) {
    if (g_hdr[i].dwFlags & WHDR_DONE) {
      g_game.synth().render(g_audio[i], AUD_LEN);
      waveOutWrite(g_wave, &g_hdr[i], sizeof(WAVEHDR));
    }
  }
}

void audioClose() {
  if (!g_wave) return;
  waveOutReset(g_wave);
  for (int i = 0; i < AUD_BUFS; ++i) waveOutUnprepareHeader(g_wave, &g_hdr[i], sizeof(WAVEHDR));
  waveOutClose(g_wave);
}

// ------------------------------------------------------------------ window
void updateTitle() {
  char t[160];
  snprintf(t, sizeof(t), "MOONSKIM - PC preview   [F2] mouse tilt: %s   [F1] controls",
           g_mouseTilt ? "ON" : "OFF");
  SetWindowTextA(g_hwnd, t);
}

void printControls() {
  logLine("");
  logLine("=== MOONSKIM PC preview controls ===");
  logLine("  Mouse            = tilt the Cardputer (left/right = bank+turn, up = dive, down = climb)");
  logLine("                     keep the mouse still near the centre while it says HOLD STILL");
  logLine("  Arrows / WASD    = turn and climb/dive (keyboard flying)");
  logLine("  Space / Alt      = thruster boost");
  logLine("  Z / X            = barrel roll left / right     Enter = confirm / auto-roll");
  logLine("  Q = loop   1 / 2 = Ring Run / Free Flight on the title screen");
  logLine("  C = recalibrate tilt   T = tilt on/off   N = day/sunset/night   V = camera");
  logLine("  F = fps/stats   H = help overlay   M = mute   Esc/P = pause   Backspace = back");
  logLine("  F2 = mouse tilt on/off (window title shows the state)   F1 = print this again");
  logLine("");
}

LRESULT CALLBACK wndProc(HWND h, UINT msg, WPARAM wp, LPARAM lp) {
  switch (msg) {
    case WM_KEYDOWN:
    case WM_SYSKEYDOWN: {
      if (wp == VK_F1) { printControls(); return 0; }
      if (wp == VK_F2) { g_mouseTilt = !g_mouseTilt; updateTitle(); return 0; }
      if (wp == VK_F4 && (GetKeyState(VK_MENU) & 0x8000)) { g_running = false; return 0; }
      const int kp = vkToKp(wp);
      if (kp >= 0) {
        const uint64_t b = kj::keyBit(kp);
        if (!(g_keysDown & b)) g_keysPressed |= b;
        g_keysDown |= b;
      }
      return 0;  // swallow Alt so it does not open the system menu
    }
    case WM_KEYUP:
    case WM_SYSKEYUP: {
      const int kp = vkToKp(wp);
      if (kp >= 0) g_keysDown &= ~kj::keyBit(kp);
      return 0;
    }
    case WM_SYSCHAR:
      return 0;
    case WM_KILLFOCUS:
      g_keysDown = 0;
      return 0;
    case WM_PAINT: {
      PAINTSTRUCT ps;
      BeginPaint(h, &ps);
      EndPaint(h, &ps);
      return 0;
    }
    case WM_ERASEBKGND:
      return 1;
    case WM_CLOSE:
      g_running = false;
      return 0;
    default:
      return DefWindowProcA(h, msg, wp, lp);
  }
}

void blit() {
  // device-exact panel buffer (byte-swapped RGB565) -> BGRA
  for (int i = 0; i < PW * PH; ++i) {
    const uint16_t v = g_panel[i];
    const uint16_t c = (uint16_t)((v >> 8) | (v << 8));
    const uint32_t r = (c >> 11) & 31, g = (c >> 5) & 63, b = c & 31;
    g_bgra[i] = ((r << 3 | r >> 2) << 16) | ((g << 2 | g >> 4) << 8) | (b << 3 | b >> 2);
  }
  RECT rc;
  GetClientRect(g_hwnd, &rc);
  const int cw = rc.right, ch = rc.bottom;
  // keep the 240:135 aspect, letterbox the rest
  int w = cw, hh = cw * PH / PW;
  if (hh > ch) { hh = ch; w = ch * PW / PH; }
  const int x0 = (cw - w) / 2, y0 = (ch - hh) / 2;
  BITMAPINFO bi = {};
  bi.bmiHeader.biSize = sizeof(BITMAPINFOHEADER);
  bi.bmiHeader.biWidth = PW;
  bi.bmiHeader.biHeight = -PH;  // top-down
  bi.bmiHeader.biPlanes = 1;
  bi.bmiHeader.biBitCount = 32;
  bi.bmiHeader.biCompression = BI_RGB;
  HDC dc = GetDC(g_hwnd);
  SetStretchBltMode(dc, COLORONCOLOR);
  if (x0 > 0) { PatBlt(dc, 0, 0, x0, ch, BLACKNESS); PatBlt(dc, x0 + w, 0, cw - x0 - w, ch, BLACKNESS); }
  if (y0 > 0) { PatBlt(dc, 0, 0, cw, y0, BLACKNESS); PatBlt(dc, 0, y0 + hh, cw, ch - y0 - hh, BLACKNESS); }
  StretchDIBits(dc, x0, y0, w, hh, 0, 0, PW, PH, g_bgra, &bi, DIB_RGB_COLORS, SRCCOPY);
  ReleaseDC(g_hwnd, dc);
}

// Mouse offset from the window centre in [-1, 1] (0,0 when the cursor is outside / tilt off).
void mouseStick(float& nx, float& ny) {
  nx = ny = 0.0f;
  if (!g_mouseTilt || GetForegroundWindow() != g_hwnd) return;
  POINT p;
  GetCursorPos(&p);
  ScreenToClient(g_hwnd, &p);
  RECT rc;
  GetClientRect(g_hwnd, &rc);
  if (p.x < 0 || p.y < 0 || p.x >= rc.right || p.y >= rc.bottom) return;
  nx = (2.0f * p.x / (float)rc.right) - 1.0f;
  ny = (2.0f * p.y / (float)rc.bottom) - 1.0f;
}

}  // namespace

int main(int argc, char** argv) {
  bool useImu = true;
  for (int i = 1; i < argc; ++i)
    if (!strcmp(argv[i], "--keys")) useImu = false;

  setvbuf(stdout, nullptr, _IONBF, 0);
  QueryPerformanceFrequency(&g_qpf);
  QueryPerformanceCounter(&g_qp0);
  timeBeginPeriod(1);

  char exe[MAX_PATH];
  GetModuleFileNameA(nullptr, exe, MAX_PATH);
  g_persistPath = exe;
  g_persistPath = g_persistPath.substr(0, g_persistPath.find_last_of("\\/") + 1) + "moonskim_save.bin";

  WNDCLASSA wc = {};
  wc.lpfnWndProc = wndProc;
  wc.hInstance = GetModuleHandleA(nullptr);
  wc.hCursor = LoadCursor(nullptr, IDC_CROSS);
  wc.lpszClassName = "MoonskimPC";
  RegisterClassA(&wc);
  RECT r = {0, 0, PW * 4, PH * 4};
  AdjustWindowRect(&r, WS_OVERLAPPEDWINDOW, FALSE);
  g_hwnd = CreateWindowA("MoonskimPC", "MOONSKIM", WS_OVERLAPPEDWINDOW | WS_VISIBLE, CW_USEDEFAULT,
                         CW_USEDEFAULT, r.right - r.left, r.bottom - r.top, nullptr, nullptr,
                         wc.hInstance, nullptr);
  if (!g_hwnd) {
    logLine("[PC] cannot create window");
    return 1;
  }
  updateTitle();
  printControls();

  const kj::PlatformApi api = {nowMicros, logLine, savePersist};
  g_game.init(api, loadPersist(), SEED, useImu);
  sim::SynthImu imu;
  imu.reset(SEED ^ 0x1D5EEDu);
  imu.setNoise(true);
  audioOpen();

  const uint32_t frameUs = 1000000u / 60u;
  uint32_t last = nowMicros(), next = last;
  while (g_running) {
    MSG m;
    while (PeekMessageA(&m, nullptr, 0, 0, PM_REMOVE)) {
      if (m.message == WM_QUIT) g_running = false;
      TranslateMessage(&m);
      DispatchMessageA(&m);
    }
    if (!g_running) break;

    const uint32_t now = nowMicros();
    kj::InputFrame in = {};
    in.dt = (float)(now - last) * 1e-6f;
    last = now;
    in.keysDown = g_keysDown;
    in.keysPressed = g_keysPressed;
    g_keysPressed = 0;
    in.imuPresent = useImu;
    if (useImu) {
      float nx, ny;
      mouseStick(nx, ny);
      imu.setTilt(nx * MOUSE_ROLL_MAX, -ny * MOUSE_PITCH_MAX, 0.05f);  // mouse up = top edge away = dive
      in.imu = imu.step(in.dt, now);
    } else {
      in.imu = kj::ImuSample{0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, now, false};
    }
    in.serialCmd = g_serialCmd;
    g_serialCmd = 0;
    in.batteryPct = 87;
    in.batteryMv = 4050;

    g_game.update(in);
    g_game.render();
    audioPump();
    kj::expand2x(g_game.frame().px, g_game.palette().lut2x(), g_panel);
    blit();
    g_game.profiler().endFrame(frameUs);

    next += frameUs;
    const uint32_t t = nowMicros();
    if ((int32_t)(next - t) > 1500) Sleep((DWORD)((next - t - 1000) / 1000));
    while ((int32_t)(next - nowMicros()) > 0) {
    }
    if ((int32_t)(nowMicros() - next) > 100000) next = nowMicros();  // resync after a stall
  }

  audioClose();
  timeEndPeriod(1);
  return 0;
}
