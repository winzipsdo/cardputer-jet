// game.h - top-level game: state machine, input mapping, subsystem orchestration,
// render order, persistence, telemetry, debug hooks for the simulator.
// OWNER: group D (game.cpp). SHARED CONTRACT with group A (platform/main) and E (sim).
//
// There is exactly ONE Game instance, a static global (it embeds ~20 KB of state; the big
// world/frame arrays are static in their own .cpp files). Never allocate a Game on the stack.
//
// Per-frame call sequence (device main.cpp and the simulator use the SAME order):
//   plat input -> game.update(in) -> game.render() -> audio pump (game.synth())
//   -> present(game.frame().px, game.palette().lut2x()) -> game.profiler().endFrame()
#pragma once
#include <stdint.h>
#include "config.h"
#include "types.h"
#include "input.h"
#include "frame.h"
#include "perf.h"
#include "palette.h"
#include "world.h"
#include "camera.h"
#include "flight.h"
#include "tilt.h"
#include "course.h"
#include "jet_model.h"
#include "render_jet.h"
#include "fx.h"
#include "hud.h"
#include "audio.h"

namespace kj {

enum class GameState : uint8_t { Boot, Calibrate, Title, Countdown, Play, Pause, Crash, GameOver };
enum class Mode : uint8_t { RingRun = 0, Free = 1 };

constexpr uint32_t PERSIST_VERSION = 1;
// Saved in NVS namespace "moonskim" as one blob (key "p"). Platform owns the storage;
// the game owns the content and asks for saves through PlatformApi::save (debounced 2 s,
// never mid-flight except settings changes from the pause menu / serial).
struct Persist {
  uint32_t version;
  uint32_t best;          // best Ring Run score
  uint8_t bestCombo;      // best Free Flight combo
  uint8_t tod;            // Tod
  uint8_t tiltOn;         // 1 = tilt steering enabled
  uint8_t sens;           // Sens
  uint8_t invert;         // invert pitch
  uint8_t assist;         // Assist
  uint8_t volume;         // 0..255
  uint8_t bright;         // 0..255
  uint8_t view;           // camera preset 0..2
  uint8_t mute;
  uint8_t drift;          // neutral-pitch drift on
  int8_t rollSign, pitchSign;  // tilt sign flags (serial r/p)
  uint8_t pad[3];
  float nx, ny, nz;       // last calibrated neutral up-vector
};

inline Persist defaultPersist() {
  Persist p = {};
  p.version = PERSIST_VERSION;
  p.tod = (uint8_t)Tod::Night;
  p.tiltOn = 1;
  p.sens = (uint8_t)Sens::Med;
  p.assist = (uint8_t)Assist::Normal;
  p.volume = cfg::VOLUME_DEFAULT;
  p.bright = cfg::BRIGHT_DEFAULT;
  p.drift = 1;
  p.rollSign = cfg::ROLL_SIGN_DEFAULT;
  p.pitchSign = cfg::PITCH_SIGN_DEFAULT;
  p.nx = 0.0f; p.ny = 0.57f; p.nz = 0.82f;
  return p;
}

// Services the core needs from the platform (device or simulator).
struct PlatformApi {
  uint32_t (*micros)();                 // monotonic microseconds
  void (*log)(const char* line);        // one complete line, no trailing newline needed
  void (*save)(const Persist& p);       // persist now (game already debounced)
};

struct Telemetry {
  GameState state;
  Mode mode;
  uint32_t score;
  uint8_t combo;
  float timeLeft;
  uint16_t rings;
  Vec3 pos;
  float yaw, speed, agl, boost, boostEnergy;
  float tiltRoll, tiltPitch, turn, climb;   // tilt angles (deg) and final stick values
  bool tiltOn, tiltValid, calibrated;
  Vec3 up, neutral;                          // tilt filter state (for the [IMU] line)
  Tod tod;
};

class Game {
 public:
  void init(const PlatformApi& api, const Persist& p, uint32_t seed, bool imuPresent);
  void update(const InputFrame& in);
  void render();

  const FrameBuf& frame() const { return fb_; }
  const Palette& palette() const { return pal_; }
  Synth& synth() { return synth_; }
  Profiler& profiler() { return prof_; }
  Telemetry telemetry() const;
  const Persist& persist() const { return persist_; }

  // Settings the platform applies when they change (brightness PWM, speaker volume).
  uint8_t brightness() const { return persist_.bright; }
  uint8_t volume() const { return persist_.volume; }
  bool muted() const { return persist_.mute != 0; }
  bool imuStream() const { return imuStream_; }   // [IMU] serial stream requested ('g' key / serial 'i')

  // Single-char serial commands handled by the core (see DESIGN.md section 14):
  //  'r' flip roll sign, 'p' flip pitch sign, 'c' recalibrate, 't' tilt on/off, 'n' time of day,
  //  'a' autopilot on/off, 'b' 20 s bench, 'x' crash+respawn, 'X' reset best, 'f' stats overlay,
  //  'i' IMU stream on/off, 'm' dump jet model ASCII. Returns false if not a core command.
  bool command(char c);

  // ---- simulator / debug hooks
  void debugStart(Mode m);                                   // skip calibration+countdown
  void debugTeleport(float x, float y, float z, float yawRad);
  void setTod(Tod t);
  void setAutopilot(bool on);
  void setView(int preset);
  // Named values for sim asserts: "alt","agl","speed","score","combo","crashed","rings",
  // "horizon","state","boost","time_left","jet_w" (jet screen bbox width). false if unknown.
  bool debugVar(const char* name, float& out) const;
  const World& world() const { return world_; }
  const JetState& jet() const { return flight_.s(); }
  const View& view() const { return cam_.view(); }

 private:
  // subsystems
  PlatformApi api_ = {nullptr, nullptr, nullptr};
  Persist persist_ = {};
  World world_;
  Palette pal_;
  JetModel jet_;
  Flight flight_;
  ChaseCamera cam_;
  TiltController tilt_;
  Course course_;
  Fx fx_;
  Synth synth_;
  Profiler prof_;
  FrameBuf fb_ = {nullptr, nullptr};
  HudData hud_ = {};
  // state
  GameState state_ = GameState::Boot;
  Mode mode_ = Mode::RingRun;
  uint32_t seed_ = 1;
  float clock_ = 0.0f, stateT_ = 0.0f, timeScale_ = 1.0f, hitStop_ = 0.0f, simAcc_ = 0.0f;
  float shake_ = 0.0f, flash_ = 0.0f, mist_ = 0.0f;
  uint32_t flashRgb_ = 0xFFF4E0u;
  uint32_t score_ = 0;
  uint8_t combo_ = 1, bestComboRun_ = 1;
  float timeLeft_ = 0.0f, skimTick_ = 0.0f;
  float keyTurn_ = 0.0f, keyClimb_ = 0.0f;
  uint64_t prevKeys_ = 0;
  uint32_t acts_ = 0, actsPressed_ = 0;
  bool imuPresent_ = false, imuStream_ = false, autopilot_ = false, statsOn_ = false;
  bool helpOn_ = false, pausedFromPlay_ = false, newBest_ = false;
  float saveTimer_ = -1.0f;
  int8_t menuSel_ = 0;
  Vec3 prevPos_ = {0.0f, 0.0f, 0.0f};
  Vec3 wreck_ = {0.0f, 0.0f, 0.0f};
  float pullUpBeep_ = 0.0f;
  uint32_t frameNo_ = 0;
  float benchT_ = -1.0f;
  FlightEvents lastEv_ = {};
  Rect jetBounds_ = {0, 0, 0, 0};
  uint32_t jetBoundsFrame_ = 0xFFFFFFF0u;  // frameNo_ when jetBounds_ was last set

  // ---- private helpers and extra state (group D; not part of the contract)
  void enterState(GameState s);
  void startRun(Mode m, bool skipCountdown);
  void placeAttractJet();
  void respawn();
  void crash(CrashCause c);
  void gameOver();
  void updateAttract(float rdt);
  void updatePlay(float rdt);
  void updatePause();
  void handleGlobalKeys();
  void handleEvents(const FlightEvents& ev, bool scoring);
  void handleRing(const RingEvent& e);
  CrashCause runFlight(float sdt, FlightInput fi, FlightEvents& ev, Assist assist);
  FlightInput playerInput(float rdt);
  FlightInput autopilotInput(float rdt, bool useRings);
  void updateMist(float rdt, bool inCloud);
  void onCalibrationDone();
  void startRecal();
  void applyTiltConfig();
  void requestSave();
  void saveNow();
  void addPopup(const char* text, int x, int y, uint8_t color);
  void setCentre(const char* text);
  void buildHud();
  void buildMenu();
  void menuAdjust(int item, int dir);
  void benchTick(float rdt);
  void startBench();
  JetPose pose() const;
  void logf(const char* fmt, ...) const;
  float lastDt_ = 0.0f, assembleT_ = 1.0f, fade_ = 0.0f, slowmoT_ = 0.0f, attractT_ = 0.0f;
  float imuStaleT_ = 0.0f, apRollT_ = 10.0f, stickTurn_ = 0.0f, stickClimb_ = 0.0f;
  bool staleLogged_ = false, calWas_ = false, recal_ = false, calSkipped_ = false;
  bool bootCal_ = false, holdStillMsg_ = false, pendingLoop_ = false, benchPrevAuto_ = false;
  bool countdownCal_ = false;               // the running calibration is the countdown re-zero
  Vec3 cdPrevN_ = {0.0f, 0.57f, 0.82f};     // neutral before the countdown re-zero
  float recalT_ = 0.0f;                     // time since the in-play 'c' recalibration began
  int8_t lastAutoRoll_ = 1, pendingRoll_ = 0, countdownDigit_ = 3;
  uint16_t ringsRun_ = 0;
  uint32_t crashes_ = 0, skimTouches_ = 0, runCount_ = 0;
  uint8_t batteryPct_ = 0;
  uint32_t benchFrames_ = 0, benchWinFrames_ = 0;
  float benchWinT_ = 0.0f, benchFpsMin_ = 0.0f, benchAvg_ = 0.0f;
  char menuVal_[MENU_ITEMS][10] = {};
};

}  // namespace kj
