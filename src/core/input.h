// input.h - raw input frame delivered by the platform, key positions, action keymap.
// OWNER: group D (keymap, actions) + group A (InputFrame filling). SHARED CONTRACT.
//
// Keys are identified by PHYSICAL POSITION kp = y*14 + x from Keyboard.keyList()
// (x 0..13, y 0..3), never by character, so Shift/Ctrl/Fn can never change meaning.
// Layout (Cardputer-Adv):
//   y0: `  1  2  3  4  5  6  7  8  9  0  -  =  BKSP
//   y1: TAB q  w  e  r  t  y  u  i  o  p  [  ]  BSLASH
//   y2: FN SHIFT a s  d  f  g  h  j  k  l  ;  '  ENTER
//   y3: CTRL OPT ALT z x  c  v  b  n  m  ,  .  /  SPACE
#pragma once
#include <stdint.h>

namespace kj {

enum KeyPos : uint8_t {
  KP_BACKTICK = 0, KP_1 = 1, KP_2 = 2, KP_3 = 3, KP_4 = 4, KP_5 = 5, KP_6 = 6, KP_7 = 7, KP_8 = 8,
  KP_9 = 9, KP_0 = 10, KP_MINUS = 11, KP_EQUAL = 12, KP_BKSP = 13,
  KP_TAB = 14, KP_Q = 15, KP_W = 16, KP_E = 17, KP_R = 18, KP_T = 19, KP_Y = 20, KP_U = 21,
  KP_I = 22, KP_O = 23, KP_P = 24, KP_LBRACK = 25, KP_RBRACK = 26, KP_BSLASH = 27,
  KP_FN = 28, KP_SHIFT = 29, KP_A = 30, KP_S = 31, KP_D = 32, KP_F = 33, KP_G = 34, KP_H = 35,
  KP_J = 36, KP_K = 37, KP_L = 38, KP_SEMI = 39, KP_QUOTE = 40, KP_ENTER = 41,
  KP_CTRL = 42, KP_OPT = 43, KP_ALT = 44, KP_Z = 45, KP_X = 46, KP_C = 47, KP_V = 48, KP_B = 49,
  KP_N = 50, KP_M = 51, KP_COMMA = 52, KP_DOT = 53, KP_SLASH = 54, KP_SPACE = 55,
  KP_COUNT = 56
};
inline uint64_t keyBit(int kp) { return 1ull << kp; }

// Names used by the simulator scripts and the serial key echo ('k').
constexpr const char* KEY_NAMES[KP_COUNT] = {
  "BACKTICK", "1", "2", "3", "4", "5", "6", "7", "8", "9", "0", "MINUS", "EQUAL", "BKSP",
  "TAB", "Q", "W", "E", "R", "T", "Y", "U", "I", "O", "P", "LBRACK", "RBRACK", "BSLASH",
  "FN", "SHIFT", "A", "S", "D", "F", "G", "H", "J", "K", "L", "SEMI", "QUOTE", "ENTER",
  "CTRL", "OPT", "ALT", "Z", "X", "C", "V", "B", "N", "M", "COMMA", "DOT", "SLASH", "SPACE"};

// IMU sample in the DEVICE frame (platform already applied cfg::IMU_SRC_*/IMU_SGN_*):
//   X right (toward Enter), Y away from the user (toward the screen edge), Z out of the keys.
//   Lying flat: az ~ +1 g. Accel in g, gyro in deg/s (right-hand rule).
struct ImuSample {
  float ax, ay, az;
  float gx, gy, gz;
  uint32_t usec;   // sensor timestamp (M5 getImuData().usec)
  bool fresh;      // false if M5.Imu.update() reported no new sample this frame
};

struct InputFrame {
  float dt;                // real frame time, s (platform measures, game clamps to cfg::DT_MAX)
  uint64_t keysDown;       // bit kp set = key currently held
  uint64_t keysPressed;    // bit kp set = key went down since the previous InputFrame
                           // (accumulated across all FIFO drains, so taps < 1 frame are kept)
  bool btnA;               // G0 button held
  bool btnAPressed;        // G0 went down since previous frame
  bool imuPresent;         // board is ADV and M5.Imu reported a sensor
  ImuSample imu;
  char serialCmd;          // forwarded single-char serial command for the game, 0 if none
  uint8_t batteryPct;      // 0..100, refreshed every cfg::BATTERY_PERIOD_S
  uint16_t batteryMv;
};

// ------------------------------------------------------------------ actions
enum Act : uint32_t {
  A_LEFT = 1u << 0, A_RIGHT = 1u << 1, A_UP = 1u << 2 /*climb*/, A_DOWN = 1u << 3 /*dive*/,
  A_BOOST = 1u << 4, A_ROLL_L = 1u << 5, A_ROLL_R = 1u << 6, A_ROLL_AUTO = 1u << 7,
  A_CONFIRM = 1u << 8, A_PAUSE = 1u << 9, A_BACK = 1u << 10, A_RECAL = 1u << 11,
  A_TILT = 1u << 12, A_INVERT = 1u << 13, A_TOD = 1u << 14, A_VIEW = 1u << 15,
  A_FPS = 1u << 16, A_IMUDBG = 1u << 17, A_HELP = 1u << 18, A_MUTE = 1u << 19,
  A_VOL_DN = 1u << 20, A_VOL_UP = 1u << 21, A_BRI_DN = 1u << 22, A_BRI_UP = 1u << 23,
  A_MODE1 = 1u << 24, A_MODE2 = 1u << 25, A_LOOP = 1u << 26
};

struct KeyBind {
  uint8_t kp;
  uint32_t acts;
};

// THE keymap (decided; see DESIGN.md section 9). G0 (BtnA) additionally maps to A_PAUSE.
constexpr KeyBind KEYMAP[] = {
  {KP_COMMA, A_LEFT},  {KP_A, A_LEFT},   {KP_SLASH, A_RIGHT}, {KP_D, A_RIGHT},
  {KP_SEMI, A_UP},     {KP_W, A_UP},     {KP_DOT, A_DOWN},    {KP_S, A_DOWN},
  {KP_SPACE, A_BOOST}, {KP_ALT, A_BOOST},
  {KP_Z, A_ROLL_L},    {KP_L, A_ROLL_L}, {KP_X, A_ROLL_R},    {KP_QUOTE, A_ROLL_R},
  {KP_ENTER, A_CONFIRM | A_ROLL_AUTO},
  {KP_P, A_PAUSE},     {KP_BACKTICK, A_PAUSE}, {KP_BKSP, A_BACK},
  {KP_C, A_RECAL},     {KP_T, A_TILT},   {KP_I, A_INVERT},    {KP_N, A_TOD},
  {KP_V, A_VIEW},      {KP_F, A_FPS},    {KP_G, A_IMUDBG},    {KP_H, A_HELP},
  {KP_M, A_MUTE},      {KP_MINUS, A_VOL_DN}, {KP_EQUAL, A_VOL_UP},
  {KP_LBRACK, A_BRI_DN}, {KP_RBRACK, A_BRI_UP},
  {KP_1, A_MODE1},     {KP_2, A_MODE2},  {KP_Q, A_LOOP},
};
constexpr int KEYMAP_COUNT = (int)(sizeof(KEYMAP) / sizeof(KEYMAP[0]));

inline uint32_t keysToActs(uint64_t keys) {
  uint32_t a = 0;
  for (int i = 0; i < KEYMAP_COUNT; ++i)
    if (keys & keyBit(KEYMAP[i].kp)) a |= KEYMAP[i].acts;
  return a;
}

}  // namespace kj
