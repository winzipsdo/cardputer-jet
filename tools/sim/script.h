// script.h - simulator scenario scripts (.jsc): parser, per-frame input scheduler, asserts.
// OWNER: group E (DESIGN.md section 18).
//
// Grammar: one command per line, "<t> <cmd> [args]". '#' starts a comment. Times are seconds
// of simulated time (60 frames per second); "+dt" means "dt after the previous command".
// Times must not decrease. A command runs in the first frame whose time >= t.
//
//   seed N                     (t = 0) world seed, decimal or 0x hex
//   tod night|sunset|day       time of day
//   view 0|1|2                 camera preset
//   state title|ring|free      ring/free call Game::debugStart (calibration skipped);
//                              title BLOCKS until the game reaches TITLE (waits for the boot
//                              calibration, taps ENTER/P/BKSP as needed; 20 s timeout)
//   key NAME down|up           keyboard, NAME from kj::KEY_NAMES (case-insensitive), or G0
//   tap NAME                   key down for exactly one frame
//   hold NAME secs             key down now, released secs later
//   tilt rollDeg pitchDeg [rampSec]   synthetic IMU, relative to the hold pose
//   teleport x y z yawDeg      Game::debugTeleport; any value may be ~ (current) or ~N (current+N)
//   autopilot on|off
//   shot NAME                  expand2x + LUT -> <out>/<script>/<NAME>.png at --scale
//   sheet NAME cols            contact sheet of the shots since the previous sheet
//   assert VAR OP VALUE        OP: < <= > >= == != (== / != use tolerance 1e-4*max(1,|VALUE|));
//                              VALUE: number, on/off, or a state name (TITLE, PLAY, ...)
//   bench FRAMES               BLOCKS for FRAMES frames, then prints the timing table
//   end                        stop the run
// Blocking commands (state title, bench, wait): when one finishes later than its scheduled
// time, every later command is shifted by that delay, so "+dt" times after a blocking command
// are relative to the moment it finished.
// Simulator extensions (documented in tools/sim/sim_main.cpp usage text):
//   persist KEY VALUE          (t = 0) initial Persist field: tod sens invert assist tilt volume
//                              bright view mute drift rollsign pitchsign best
//   imu present|absent         (t = 0) board has an IMU (InputFrame.imuPresent)
//   imu noise on|off           synthetic IMU noise
//   pose holdDeg [rampSec]     move the hold pose (default 35 deg toward the face)
//   cmd C                      forward one serial command character to the game
//   spawn SALT [ALT]           World::findSpawn(SALT) -> teleport there at ALT (default 40)
//   seek land ALT [DIST] [cloudy]          highest land within 1600 u; jet DIST (default 160)
//                                          before it on the current heading, at ALT
//   seek cloud ALT|under|in|above [DIST]   densest cloud within 800 u (DIST default 0)
//   seek sea ALT                           nearest open water with 600 u of sea ahead
//   jetpose yawDeg rollDeg pitchDeg [boost] | jetpose off
//                              "studio" frames: world + sky + shadow + jet re-rendered by the
//                              sim with this pose relative to the game's jet (no HUD)
//   resetmax                   restart the max_* / min_* trackers
//   echo TEXT...               print TEXT and the current sim time
//   wait VAR OP VALUE [timeoutSec]   BLOCKS until the assert-style condition holds (checked
//                              after every render; default timeout 30 s, runtime error after)
// Frame phases: every due input/state command runs BEFORE the frame's update+render; shot,
// sheet, assert, wait, echo and end run AFTER it. An input command listed after an output
// command with the same time therefore lands one frame later.
// Variables (assert/wait): Sim::getVar in sim_main.cpp - t frame state mode crashes empty_px
// fps_est esp_est esp_wld esp_jet esp_fx c1_est steps samples fills voxels vox_writes flame_px
// particles line_px max_<counter> max_esp_* max_c1_est max_empty_px min_fps_est turn climb
// tilt_roll tilt_pitch tilt_valid tilt_on calibrated x y z vz yaw(deg) yaw_rate pitch bank
// roll_angle g boost_energy invuln in_cloud cloud_thick imu_roll imu_pitch imu_hold; any other
// name goes to Game::debugVar (alt agl speed score combo crashed rings horizon boost time_left
// jet_w). Errors are reported as "<file>:<line>: message"; exit code 2 = parse error,
// 1 = assert failure or runtime error.
#pragma once
#include <stdint.h>
#include <deque>
#include <string>
#include <utility>
#include <vector>

namespace sim {

enum class CmdType : uint8_t {
  Seed, Persist, ImuPresent, ImuNoise, Tod, View, State, Key, Tap, Hold, Tilt, Pose, Teleport,
  Autopilot, SerialCmd, Bench, Spawn, Seek, JetPose, ResetMax, Echo, Shot, Sheet, Assert, Wait, End
};

enum class AssertOp : uint8_t { LT, LE, GT, GE, EQ, NE };

struct Cmd {
  double t = 0.0;
  int line = 0;
  CmdType type = CmdType::End;
  std::string s0, s1;      // names / sub-commands / text
  float f[4] = {0.0f, 0.0f, 0.0f, 0.0f};
  uint8_t rel[4] = {0, 0, 0, 0};   // teleport: 1 = '~' (relative to current)
  int nf = 0;              // number of numeric args given
  int i0 = 0;              // key position / state / tod / view / cols / frames / flags
  AssertOp op = AssertOp::EQ;
};

struct PersistOverride {
  std::string key;
  int value;
};

const char* stateName(int gameState);       // "BOOT" ... "GAMEOVER"
int stateFromName(const char* name);        // -1 if unknown (case-insensitive)
int keyFromName(const char* name);          // 0..55, KEY_G0 (56) for the G0 button, -1 unknown
constexpr int KEY_G0 = 56;

class Script {
 public:
  bool load(const char* path, std::string& err);
  bool parse(const char* text, const char* fileName, std::string& err);

  const std::vector<Cmd>& cmds() const { return cmds_; }
  const std::string& name() const { return name_; }   // file base name (output folder)
  const std::string& file() const { return file_; }
  bool hasSeed() const { return hasSeed_; }
  uint32_t seed() const { return seed_; }
  bool imuPresent() const { return imuPresent_; }
  const std::vector<PersistOverride>& persist() const { return persist_; }

 private:
  bool parseLine(const std::string& line, int lineNo, double& prevT, std::string& err);
  std::vector<Cmd> cmds_;
  std::string name_, file_;
  bool hasSeed_ = false;
  uint32_t seed_ = 0;
  bool imuPresent_ = true;
  std::vector<PersistOverride> persist_;
};

// Services the scheduler needs from the simulator.
class SimHost {
 public:
  virtual ~SimHost() {}
  // Commands that act on the game / renderer / outputs (everything except keys, serial,
  // blocking goals and asserts). Return false + err on failure (reported with the line).
  virtual bool execCommand(const Cmd& c, std::string& err) = 0;
  virtual int gameState() const = 0;            // (int)kj::GameState
  virtual bool getVar(const char* name, float& out) const = 0;
  virtual void benchBegin(int frames) = 0;
  virtual void benchEnd() = 0;
  virtual void log(const char* line) = 0;
};

// Keyboard / button / serial state handed to the game for one frame.
struct FrameKeys {
  uint64_t down = 0, pressed = 0;
  bool btnA = false, btnAPressed = false;
  char serialCmd = 0;
};

class Scheduler {
 public:
  void begin(const Script& s);
  // Start of frame `frame` (time frame/60): key releases, then every due pre-render command.
  void preFrame(int frame, SimHost& host, FrameKeys& out);
  // After render + expand: shots, sheets, asserts, end; bench countdown.
  void postFrame(int frame, SimHost& host);
  bool done() const { return ended_ || failed_ || (pc_ >= cmds_.size() && !blocked()); }
  bool failed() const { return failed_; }               // runtime error (already reported)
  int assertsRun() const { return assertsRun_; }
  int assertsFailed() const { return assertsFailed_; }
  const std::string& file() const { return file_; }

 private:
  static double frameTime(int frame) { return (double)frame / 60.0; }
  static bool isPost(CmdType t);
  bool blocked() const { return titleGoal_ || benchLeft_ > 0 || waitGoal_; }
  bool due(const Cmd& c, double T) const { return c.t + shift_ <= T + 1e-6; }
  void finishGoal(int frame);
  bool evalCond(const Cmd& c, SimHost& host, float& v, bool& ok);
  void execPre(const Cmd& c, int frame, SimHost& host);
  void execPost(const Cmd& c, int frame, SimHost& host);
  void stepWaitGoal(int frame, SimHost& host);
  void runtimeError(const Cmd& c, const std::string& msg, SimHost& host);
  void pressKey(int kp, int releaseFrame);
  void stepTitleGoal(int frame, SimHost& host);

  std::vector<Cmd> cmds_;
  std::string file_;
  size_t pc_ = 0;
  uint64_t down_ = 0, prevDown_ = 0;
  std::vector<std::pair<int, int>> releases_;   // (key, frame at which it is released)
  std::deque<char> serial_;
  bool titleGoal_ = false;
  int titleLine_ = 0, titleStart_ = 0, navNext_ = 0;
  int benchLeft_ = 0;
  bool waitGoal_ = false;
  Cmd waitCmd_;
  int waitStart_ = 0;
  double goalT_ = 0.0;   // scheduled (script) time of the active blocking command
  double shift_ = 0.0;   // accumulated delay added to every later command time
  bool ended_ = false, failed_ = false;
  int assertsRun_ = 0, assertsFailed_ = 0;
};

}  // namespace sim
