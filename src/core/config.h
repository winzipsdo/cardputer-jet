// config.h - EVERY tuning constant of MOONSKIM lives here.
// OWNER: shared. Each block names the group that tunes it; a group may change the
// VALUES of its own block freely, but adding/removing/renaming a constant used by
// another group needs agreement. Units: u = world unit (= 1 jet voxel), s, rad, px =
// LOGICAL pixel (120x68 frame) unless stated otherwise.
#pragma once
#include <stdint.h>

// ---------------------------------------------------------------- build switches
#ifndef JET_SPI_MHZ
#define JET_SPI_MHZ 40          // 40 (safe, default) or 80 (env cardputer-adv-spi80)
#endif
#ifndef JET_FPS_CAP
#define JET_FPS_CAP 60          // 0 = uncapped (bench env)
#endif
#ifndef JET_BOOT_BENCH
#define JET_BOOT_BENCH 0        // 1 = run the 20 s autopilot bench right after boot
#endif
#ifndef JET_AUDIO
#define JET_AUDIO 1             // 0 = silent build (no Speaker use at all)
#endif
#ifndef JET_DEBUG
#define JET_DEBUG 0             // 1 = IMU stream + key echo on at boot
#endif
#define JET_FW_NAME    "moonskim"
#define JET_FW_VERSION "0.1.0"

namespace kj {
namespace cfg {

// ================================================================ screen (A)
constexpr int SCREEN_W = 240, SCREEN_H = 135;  // panel, landscape (M5GFX 0.3.0 rotation 0)
constexpr int LW = 120, LH = 68;               // logical frame, 2x nearest expand
constexpr int LPIX = LW * LH;                  // 8160
constexpr uint8_t BRIGHT_DEFAULT = 150, BRIGHT_STEP = 24, BRIGHT_MIN = 24;
constexpr uint32_t FRAME_US = 16667;           // 60 fps pacing period
constexpr float DT_MAX = 0.05f;                // frame dt clamp
constexpr float SIM_STEP = 1.0f / 120.0f;      // fixed sub-step for flight
constexpr int   SIM_MAX_SUBSTEPS = 6;

// ================================================================ world data (B)
constexpr float CELL_U = 4.0f;          // world units per terrain cell
constexpr int   CELL_SHIFT = 2;         // log2(CELL_U)
constexpr int   WORLD_CELLS = 2048;     // world wraps every 2048 cells
constexpr float WORLD_U = 8192.0f;      // = WORLD_CELLS * CELL_U
constexpr int   STAMP_N = 64;           // island stamp = 64x64 cells (256 u)
constexpr int   STAMP_COUNT = 12;
constexpr int   MIP_N = 32;             // per-stamp 2x2 max mip (8 u cells)
constexpr int   MACRO_N = 32;           // macro table 32x32 stamps = 2048 cells
constexpr uint8_t MACRO_OCEAN = 0xFF;   // macro entry: open deep ocean
constexpr uint8_t MACRO_TRANSPOSE = 0x10;  // macro entry bit: swap lx/ly
constexpr float MACRO_ISLAND_FRACTION = 0.45f;
constexpr float STAMP_LAND_MIN = 0.30f, STAMP_LAND_MAX = 0.45f;  // land fraction per stamp
constexpr int   STAMP_BORDER = 4;       // border cells forced to DEEP
// Terrace level (cell high nibble) -> height in u. Levels 1-8 grass, 9-12 rock, 13-15 snow.
constexpr uint8_t HEIGHT_U[16] = {0, 3, 6, 9, 12, 15, 18, 21, 24, 28, 32, 36, 40, 45, 50, 56};
constexpr int   ZTOP = 56;              // tallest possible terrain (u)
constexpr int   BEACONS_MAX = 32;

// clouds (B)
constexpr int   CLOUD_N = 128;          // 128x128 byte map
constexpr float CLOUD_CELL_U = 8.0f;    // -> wraps every 1024 u (independent of terrain)
constexpr float CLOUD_BASE = 100.0f;    // slab base (u)
constexpr float CLOUD_LAYER_U = 3.0f;
constexpr int   CLOUD_MAX_LAYERS = 6;
constexpr float CLOUD_TOP = CLOUD_BASE + CLOUD_LAYER_U * CLOUD_MAX_LAYERS;  // 118
constexpr float CLOUD_COVER = 0.40f;
constexpr int   CLOUD_CLUSTERS = 48;
constexpr float CLOUD_DRIFT_X = 6.0f, CLOUD_DRIFT_Y = 2.5f;  // u/s

// ================================================================ world render (B)
constexpr float VIEW_DIST_BASE = 640.0f, VIEW_DIST_ALT = 2.0f, VIEW_DIST_MAX = 1000.0f;
constexpr float FOG_START_BASE = 120.0f, FOG_START_ALT = 1.5f;
constexpr int   FOG_LEVELS = 8;
constexpr float FOG_T[FOG_LEVELS] = {0.0f, 0.12f, 0.25f, 0.38f, 0.52f, 0.66f, 0.80f, 0.92f};
constexpr int   MARCH_MAX_STEPS = 180;
constexpr float MARCH_START_DEN = 0.15f;    // first depth step at den = 0.15
constexpr float MARCH_DV_MIN = 2.0f, MARCH_DV_MAXK = 2.5f;
constexpr float MIP_M_THRESHOLD = 0.125f;  // use mip when px-per-u m < this
constexpr float NEAR_FX_DIST = 300.0f;     // glints / surf flicker / moon path only nearer than this (zc)
constexpr int   SKY_BANDS = 8;
constexpr float SKY_SPAN_FRAC = 0.75f;     // bands span 0.75*LH rows above the horizon
constexpr int   STARS = 240;
constexpr int   MILKY_DUST = 300;
constexpr float SHOOTING_STAR_MIN_S = 8.0f, SHOOTING_STAR_MAX_S = 18.0f;
// Light: one azimuth for moon (night) and sun (sunset/day). Direction TOWARD the light
// is forward(MOON_AZ). Shadows fall the opposite way.
constexpr float LIGHT_AZ = -0.7853982f;             // forward(-pi/4) = (-0.707,-0.707)
constexpr float MOON_EL_TAN = 0.17f;                // ~10 deg above horizon
constexpr float SUN_EL_TAN_SUNSET = 0.08f;
constexpr float MOON_R_PX = 3.0f, SUN_R_PX = 6.0f;  // logical px
// Ground offset of a shadow per unit of height (x,y), per Tod {Night, Sunset, Day}.
constexpr float SHADOW_K[3] = {0.35f, 0.80f, 0.25f};  // offset = (SHADOW_K, SHADOW_K) * h

// ================================================================ camera (D)
constexpr float CAM_F0 = 88.0f;              // focal length, logical px (hFOV ~68.6 deg)
constexpr float CAM_DIST[3] = {88.0f, 120.0f, 150.0f};  // view presets: near(default)/far/high
constexpr float CAM_TILT[3] = {0.21f, 0.24f, 0.38f};    // base look-down angle (rad)
constexpr float CAM_TILT_CLIMB_K = 0.75f;    // tilt = base - K*sin(pitch)
constexpr float CAM_T_CLAMP = 1.1f;          // clamp on the angle before tan()
constexpr float CAM_T_EASE = 6.0f;
constexpr float CAM_CY_FRAC = 0.64f, CAM_CY_BOOST = 0.03f;
constexpr float CAM_BOOST_PULL = 0.16f, CAM_BOOST_EASE = 4.0f;
constexpr float CAM_FOV_PUNCH = 0.10f, CAM_FOV_SPRING_W = 14.0f, CAM_FOV_SPRING_Z = 0.55f;
constexpr float CAM_FOCUS_EASE = 3.5f, CAM_ALT_EASE = 3.0f, CAM_ALT_FEED = 0.8f;
constexpr float CAM_HEADING_EASE = 3.4f;
constexpr float CAM_FLOOR_MARGIN = 5.0f, CAM_FLOOR_PROBE = 8.0f;
constexpr float CAM_CLOUD_MARGIN = 3.0f, CAM_TFIX_EASE = 7.0f;
constexpr float SHAKE_DECAY = 6.0f, SHAKE_X_PX = 2.5f, SHAKE_Y_PX = 2.0f;
constexpr float CRASH_ORBIT_RATE = 0.6f, CRASH_ORBIT_DIST = 110.0f;
constexpr float CULL_DEN = 0.15f, JET_CULL_DEN = 0.2f;

// ================================================================ flight (D)
constexpr float JET_LEN_U = 34.0f, JET_SPAN_U = 26.0f;
constexpr float V_CRUISE = 68.0f;            // u/s (2 jet lengths per second)
constexpr float BOOST_MUL = 1.8f;
constexpr float BOOST_UP = 5.0f, BOOST_DOWN = 2.4f;
constexpr float BOOST_DRAIN = 0.22f, BOOST_REGEN = 0.10f, BOOST_REGEN_SKIM = 0.35f;
constexpr float BOOST_RING_BONUS = 0.25f, BOOST_MIN_ENGAGE = 0.10f;
constexpr float MAX_YAW = 1.30f, MAX_YAW_BOOST = 1.15f;  // rad/s
constexpr float YAW_EASE = 3.2f;
constexpr float BANK_MAX = 0.96f, BANK_EASE = 6.0f;      // 55 deg, visual only
constexpr float TURN_NOSE_UP = 0.08f;
constexpr float CLIMB_MAX = 1.05f, DIVE_MAX = 0.70f;     // rad
constexpr float CLIMB_EASE = 7.0f, CLIMB_RATE_UP = 1.6f, CLIMB_RATE_DOWN = 1.35f;
constexpr float ENERGY_EASE = 1.1f, CLIMB_BLEED = 0.22f, ENERGY_MAX = 1.15f;
constexpr float CEILING = 220.0f, CEIL_CUSHION = 20.0f;
constexpr float BOB_AMP = 1.0f, BOB_FREQ = 0.9f;
constexpr float SPAWN_ALT = 40.0f;
constexpr float SKIM_Z = 2.0f;                 // sea floor for the jet pivot
constexpr float SKIM_BAND = 8.0f;              // "skimming" below this height above water
constexpr float SEA_CRASH_VZ = -14.0f;         // u/s
constexpr float SEA_CRASH_PITCH = -0.35f;      // rad
constexpr float TERRAIN_CLEAR = 1.5f, NOSE_PROBE = 17.0f, NOSE_CLEAR = 1.0f;
constexpr float CLOSE_CALL_DIST = 5.0f, CLOSE_CALL_COOLDOWN = 2.0f;
constexpr float GPWS_LOOKAHEAD = 0.8f, GPWS_GAP = 4.0f, GPWS_SINK = 8.0f;
constexpr float EASY_FLOOR_CLEAR = 6.0f, EASY_FLOOR_UP = 9.0f, EASY_FLOOR_DOWN = 3.0f;
constexpr float ROLL_TIME = 0.80f, ROLL_DASH_U = 12.0f, ROLL_COOLDOWN = 0.25f;
constexpr float LOOP_TIME = 3.4f;
constexpr float G_EASE = 2.5f;
constexpr float INVULN_TIME = 2.0f;

// ================================================================ tilt / input (D)
// Sensitivity presets [Low, Med, High]: deadzone and full-scale angles in degrees.
constexpr float TILT_ROLL_DZ[3] = {3.0f, 3.0f, 2.5f};
constexpr float TILT_ROLL_FULL[3] = {32.0f, 24.0f, 19.0f};
constexpr float TILT_PITCH_DZ[3] = {4.0f, 4.0f, 3.5f};
constexpr float TILT_PITCH_FULL[3] = {28.0f, 22.0f, 17.0f};
constexpr float TILT_CURVE_LIN = 0.45f;         // out = u*(LIN + (1-LIN)*u)
constexpr float TILT_TAU_STILL = 0.25f, TILT_TAU_MOVING = 2.0f;   // accel correction time const
constexpr float TILT_ACC_TOL_FAST = 0.10f, TILT_ACC_TOL_SLOW = 0.40f;  // | |a|-1g |
constexpr float TILT_OUT_HZ = 10.0f;            // output low-pass
constexpr float TILT_CAL_STILL_DPS = 8.0f, TILT_CAL_STILL_S = 0.5f, TILT_CAL_TIMEOUT_S = 4.0f;
constexpr float TILT_RECAL_S = 0.5f;
constexpr float TILT_DRIFT_TAU = 20.0f, TILT_DRIFT_LIMIT_DEG = 12.0f;
constexpr float TILT_DRIFT_WINDOW_DEG = 8.0f, TILT_DRIFT_TURN = 0.3f, TILT_DRIFT_HOLD_S = 2.0f;
constexpr float TILT_STALE_S = 0.2f;
constexpr float TILT_GUARD_PITCH_DEG = 60.0f;   // beyond this: output frozen at 0, HUD "TILT?"
constexpr float KEY_RAMP_UP = 6.0f, KEY_RAMP_DOWN = 10.0f;
constexpr int8_t ROLL_SIGN_DEFAULT = 1, PITCH_SIGN_DEFAULT = 1;  // VERIFY ON DEVICE (serial r/p flips)
// Platform-side raw->device axis remap (applied in plat_input.cpp). Device frame:
// X right (toward Enter), Y away from user (toward screen edge), Z out of the keys.
constexpr int   IMU_SRC_X = 0, IMU_SRC_Y = 1, IMU_SRC_Z = 2;   // raw index feeding device x,y,z
constexpr float IMU_SGN_X = 1.0f, IMU_SGN_Y = 1.0f, IMU_SGN_Z = 1.0f;

// ================================================================ course / scoring (D)
constexpr int   LIVE_RINGS = 3;
constexpr float RING_R_START = 20.0f, RING_R_END = 14.0f;   // shrinks over RING_DIFF_RINGS
constexpr float RING_PERFECT_R = 6.5f;
constexpr int   RING_DIFF_RINGS = 40;
constexpr float RING_DIST_START = 300.0f, RING_DIST_END = 210.0f;
constexpr float RING_TURN_MIN0 = 0.26f, RING_TURN_MAX0 = 0.70f;   // |delta heading| at d=0 (rad)
constexpr float RING_TURN_MIN1 = 0.44f, RING_TURN_MAX1 = 0.96f;   // at d=1
constexpr float RING_ALT_LOW_MIN = 10.0f, RING_ALT_LOW_MAX = 20.0f;  // above surface
constexpr float RING_ALT_MID_MIN = 25.0f, RING_ALT_MID_MAX = 65.0f;
constexpr float RING_ALT_HIGH_MIN = 128.0f, RING_ALT_HIGH_MAX = 145.0f;
constexpr float RING_P_LOW = 0.50f, RING_P_MID = 0.35f;   // rest HIGH
constexpr int   RING_HIGH_FROM = 6;                       // no HIGH rings before ring 6
constexpr float RING_RIM_CLEAR = 8.0f;
constexpr float RING_MISS_DIST = 520.0f, RING_MISS_TIME = 25.0f;
constexpr float MAGNET_RANGE = 200.0f, MAGNET_CONE = 0.61f;     // rad (35 deg)
constexpr float MAGNET_TURN_K = 1.2f, MAGNET_TURN_MAX = 0.25f;
constexpr float MAGNET_CLIMB_K = 0.016f, MAGNET_CLIMB_MAX = 0.15f;
constexpr float ASSIST_GAIN[3] = {1.0f, 0.6f, 0.0f};            // Easy, Normal, Pro
constexpr float TIMER_START = 45.0f, TIMER_RING = 4.0f, TIMER_PERFECT = 1.5f;
constexpr float TIMER_MAX = 60.0f, TIMER_CRASH = 5.0f;
constexpr int   COMBO_MAX = 8;
constexpr int   PTS_RING = 100, PTS_PERFECT = 50, PTS_ROLL = 100, PTS_LOW = 50, PTS_BOOST = 25;
constexpr int   PTS_SKIM_TICK = 10;  constexpr float SKIM_TICK_S = 0.1f;
constexpr int   PTS_CLOSE = 25, PTS_LOOP = 200;
constexpr float LOW_RING_ALT = 20.0f;
constexpr float HITSTOP_PERFECT_S = 0.06f, HITSTOP_PERFECT_SCALE = 0.5f;
constexpr float CRASH_SLOWMO_S = 0.6f, CRASH_SLOWMO_SCALE = 0.3f, TIMESCALE_EASE = 8.0f;
constexpr float CRASH_SEQ_S = 2.2f, RESPAWN_BACK_U = 200.0f, RESPAWN_ALT = 50.0f;
constexpr float COUNTDOWN_S = 1.5f;
constexpr float ATTRACT_ROLL_MIN_S = 9.0f, ATTRACT_ROLL_MAX_S = 14.0f, ATTRACT_ALT = 55.0f;
constexpr float PERSIST_DEBOUNCE_S = 2.0f;

// ================================================================ jet model / render (C)
constexpr int   JET_GRID_X = 26, JET_GRID_Y = 34, JET_GRID_Z = 11;  // span x length x slices
constexpr int   JET_PIVOT_ROW = 19;       // b = PIVOT_ROW - row  (b > 0 toward the nose)
constexpr int   JET_PIVOT_SLICE = 4;      // s = slice - PIVOT_SLICE
// across a = (x - 12.5) u, stored doubled as a2 = 2x - 25 (odd integers)
constexpr int   JET_MAX_VOX = 1400;
constexpr float JET_SPLAT_BIG_M = 1.6f;   // m >= this: each sub-sample writes 2x2 px
constexpr float JET_LIT_DOT = 0.55f, JET_DIM_DOT = -0.15f, JET_DARK_DOT = -0.55f;
constexpr float JET_SPEC_DOT = 0.92f;
constexpr float NOZZLE_ROW = 33.0f, NOZZLE_SLICE = 4.0f, NOZZLE_R = 2.4f;
constexpr float FLAME_LEN_CRUISE = 6.0f, FLAME_LEN_BOOST = 14.0f;  // u: len = C + B*boost
constexpr float FLAME_R = 2.0f;
constexpr float FLAME_CURL = 0.36f;
constexpr float SHOCK_T[2] = {0.28f, 0.52f};
constexpr float STROBE_PERIOD = 1.2f;
constexpr float NAV_BLINK_PERIOD = 1.0f, NAV_BLINK_DUTY = 0.7f;
constexpr float JET_SHADOW_MAX_H = 60.0f;   // no shadow above this height over ground
constexpr float ASSEMBLE_STAGGER = 0.05f, ASSEMBLE_FALL = 0.30f, ASSEMBLE_DROP_PX = 20.0f;

// ================================================================ effects (C)
constexpr int   PARTICLES = 128;
constexpr int   TRAIL_POINTS = 48;          // per wingtip
constexpr float TRAIL_SPACING_U = 3.0f, TRAIL_LIFE_S = 2.4f;
constexpr int   WAKE_MARKS = 32;
constexpr float WAKE_SPACING_U = 2.4f, WAKE_BEHIND_U = 6.0f, WAKE_LIFE_S = 2.6f;
constexpr float WAKE_START_H = 24.0f, WAKE_FULL_H = 7.0f, WAKE_SPEED_FULL = 50.0f;
constexpr float SMOKE_PERIOD = 0.034f, SMOKE_PERIOD_BOOST = 0.012f, SMOKE_LIFE = 1.0f;
constexpr float SPRAY_RATE_BASE = 40.0f, SPRAY_RATE_K = 70.0f;
constexpr int   SPEEDLINES = 20;
constexpr float SPEEDLINE_AHEAD = 250.0f;
constexpr float MIST_MIN = 0.55f, MIST_MAX = 0.75f;

// ================================================================ audio (D synth, A pump)
constexpr int   AUDIO_RATE = 16000;
constexpr int   AUDIO_BLOCK = 384;          // samples per buffer (24 ms)
constexpr int   AUDIO_BUFFERS = 3;
constexpr int   AUDIO_CHANNEL = 0;          // M5 Speaker virtual channel for the stream
constexpr uint8_t VOLUME_DEFAULT = 80, VOLUME_STEP = 16;

// ================================================================ logging (A)
constexpr float STAT_PERIOD_S = 1.0f;
constexpr float IMU_STREAM_HZ = 20.0f;
constexpr float BATTERY_PERIOD_S = 2.0f;
constexpr float SLOW_FRAME_MS = 20.0f;

}  // namespace cfg
}  // namespace kj
