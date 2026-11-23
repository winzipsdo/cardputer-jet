// script.cpp - see script.h. OWNER: group E.
#ifndef _CRT_SECURE_NO_WARNINGS
#define _CRT_SECURE_NO_WARNINGS
#endif
#include "script.h"

#include <ctype.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <fstream>
#include <sstream>

#include "game.h"
#include "input.h"

namespace sim {

namespace {

const char* const STATE_NAMES[] = {"BOOT", "CALIBRATE", "TITLE", "COUNTDOWN",
                                   "PLAY", "PAUSE",     "CRASH", "GAMEOVER"};
constexpr int STATE_COUNT = (int)(sizeof(STATE_NAMES) / sizeof(STATE_NAMES[0]));
static_assert((int)kj::GameState::Boot == 0 && (int)kj::GameState::Calibrate == 1 &&
                  (int)kj::GameState::Title == 2 && (int)kj::GameState::Countdown == 3 &&
                  (int)kj::GameState::Play == 4 && (int)kj::GameState::Pause == 5 &&
                  (int)kj::GameState::Crash == 6 && (int)kj::GameState::GameOver == 7,
              "STATE_NAMES must follow kj::GameState");
static_assert(KEY_G0 >= kj::KP_COUNT && KEY_G0 < 64, "G0 needs a free bit");

const double TIME_EPS = 1e-6;
const int TITLE_TIMEOUT_FRAMES = 20 * 60;
const int CALIBRATE_WAIT_FRAMES = 6 * 60;
const int NAV_GAP_FRAMES = 6;
const uint64_t KEYS_MASK = (1ull << kj::KP_COUNT) - 1ull;

bool ieq(const char* a, const char* b) {
  for (; *a && *b; ++a, ++b)
    if (tolower((unsigned char)*a) != tolower((unsigned char)*b)) return false;
  return *a == *b;
}

std::string lower(std::string s) {
  for (char& c : s) c = (char)tolower((unsigned char)c);
  return s;
}

bool parseNum(const std::string& s, double& out) {
  if (s.empty()) return false;
  char* end = nullptr;
  out = strtod(s.c_str(), &end);
  return end && *end == '\0' && isfinite(out);
}

bool parseFloat(const std::string& s, float& out) {
  double d;
  if (!parseNum(s, d)) return false;
  out = (float)d;
  return true;
}

bool parseInt(const std::string& s, long& out) {
  if (s.empty()) return false;
  char* end = nullptr;
  out = strtol(s.c_str(), &end, 0);
  return end && *end == '\0';
}

bool validName(const std::string& s) {
  if (s.empty() || s.size() > 64) return false;
  for (char c : s)
    if (!(isalnum((unsigned char)c) || c == '_' || c == '-')) return false;
  return true;
}

std::string baseName(const std::string& path) {
  size_t p = path.find_last_of("/\\");
  std::string b = p == std::string::npos ? path : path.substr(p + 1);
  size_t d = b.find_last_of('.');
  if (d != std::string::npos && d > 0) b = b.substr(0, d);
  return b;
}

const char* const PERSIST_KEYS[] = {"tod",  "sens", "invert", "assist",   "tilt",      "volume", "bright",
                                    "view", "mute", "drift",  "rollsign", "pitchsign", "best"};

}  // namespace

const char* stateName(int s) { return (s >= 0 && s < STATE_COUNT) ? STATE_NAMES[s] : "?"; }

int stateFromName(const char* name) {
  for (int i = 0; i < STATE_COUNT; ++i)
    if (ieq(name, STATE_NAMES[i])) return i;
  if (ieq(name, "GAME_OVER")) return (int)kj::GameState::GameOver;
  return -1;
}

int keyFromName(const char* name) {
  for (int i = 0; i < kj::KP_COUNT; ++i)
    if (ieq(name, kj::KEY_NAMES[i])) return i;
  if (ieq(name, "G0") || ieq(name, "BTNA")) return KEY_G0;
  struct Alias { const char* s; int kp; };
  static const Alias ALIASES[] = {
      {"`", kj::KP_BACKTICK}, {"-", kj::KP_MINUS}, {"=", kj::KP_EQUAL}, {"[", kj::KP_LBRACK},
      {"]", kj::KP_RBRACK},   {"\\", kj::KP_BSLASH}, {";", kj::KP_SEMI}, {"'", kj::KP_QUOTE},
      {",", kj::KP_COMMA},    {".", kj::KP_DOT},   {"/", kj::KP_SLASH}, {"BACKSPACE", kj::KP_BKSP},
      {"RETURN", kj::KP_ENTER}};
  for (const Alias& a : ALIASES)
    if (ieq(name, a.s)) return a.kp;
  return -1;
}

// ------------------------------------------------------------------ Script

bool Script::load(const char* path, std::string& err) {
  std::ifstream in(path, std::ios::in | std::ios::binary);
  if (!in) {
    err = std::string(path) + ": cannot open script";
    return false;
  }
  std::stringstream ss;
  ss << in.rdbuf();
  return parse(ss.str().c_str(), path, err);
}

bool Script::parse(const char* text, const char* fileName, std::string& err) {
  cmds_.clear();
  persist_.clear();
  hasSeed_ = false;
  seed_ = 0;
  imuPresent_ = true;
  file_ = fileName ? fileName : "script";
  name_ = baseName(file_);
  if (!validName(name_)) {
    // keep the output folder name safe
    std::string clean;
    for (char c : name_) clean += (isalnum((unsigned char)c) || c == '_' || c == '-') ? c : '_';
    name_ = clean.empty() ? "script" : clean;
  }
  double prevT = 0.0;
  int lineNo = 0;
  const char* p = text;
  while (*p) {
    const char* e = p;
    while (*e && *e != '\n') ++e;
    std::string line(p, (size_t)(e - p));
    ++lineNo;
    if (!parseLine(line, lineNo, prevT, err)) return false;
    p = *e ? e + 1 : e;
  }
  return true;
}

bool Script::parseLine(const std::string& rawLine, int lineNo, double& prevT, std::string& err) {
  std::string line = rawLine;
  size_t hash = line.find('#');
  if (hash != std::string::npos) line.resize(hash);
  std::vector<std::string> tok;
  {
    std::istringstream is(line);
    std::string t;
    while (is >> t) tok.push_back(t);
  }
  if (tok.empty()) return true;

  char prefix[300];
  snprintf(prefix, sizeof(prefix), "%s:%d: ", file_.c_str(), lineNo);
  auto fail = [&](const std::string& m) {
    err = std::string(prefix) + m;
    return false;
  };

  // ---- time
  double t = 0.0;
  if (tok[0][0] == '+') {
    double dt;
    if (!parseNum(tok[0].substr(1), dt) || dt < 0.0) return fail("bad relative time '" + tok[0] + "'");
    t = prevT + dt;
  } else if (!parseNum(tok[0], t) || t < 0.0) {
    return fail("expected a time in seconds, got '" + tok[0] + "'");
  }
  if (t + TIME_EPS < prevT) return fail("time goes backwards (" + tok[0] + ")");
  prevT = t;
  if (tok.size() < 2) return fail("missing command");

  Cmd c;
  c.t = t;
  c.line = lineNo;
  const std::string cmd = lower(tok[1]);
  const size_t nargs = tok.size() - 2;
  auto arg = [&](size_t i) -> const std::string& { return tok[i + 2]; };
  auto needArgs = [&](size_t lo, size_t hi) {
    return nargs >= lo && nargs <= hi;
  };
  auto usage = [&](const char* u) { return fail(std::string("usage: ") + u); };
  auto atZero = [&]() { return t <= TIME_EPS; };

  if (cmd == "seed") {
    long v;
    if (!needArgs(1, 1)) return usage("seed N");
    if (!atZero()) return fail("seed is only allowed at t = 0");
    if (!parseInt(arg(0), v)) {
      // allow values above LONG_MAX (e.g. 0xDEADBEEF)
      char* end = nullptr;
      unsigned long long u = strtoull(arg(0).c_str(), &end, 0);
      if (!end || *end != '\0' || u > 0xFFFFFFFFull) return fail("bad seed '" + arg(0) + "'");
      seed_ = (uint32_t)u;
    } else {
      seed_ = (uint32_t)v;
    }
    hasSeed_ = true;
    c.type = CmdType::Seed;
  } else if (cmd == "persist") {
    long v;
    if (!needArgs(2, 2)) return usage("persist KEY VALUE");
    if (!atZero()) return fail("persist is only allowed at t = 0");
    const std::string k = lower(arg(0));
    bool known = false;
    for (const char* pk : PERSIST_KEYS) known = known || k == pk;
    if (!known) return fail("unknown persist key '" + arg(0) + "'");
    if (!parseInt(arg(1), v)) return fail("bad persist value '" + arg(1) + "'");
    persist_.push_back(PersistOverride{k, (int)v});
    c.type = CmdType::Persist;
  } else if (cmd == "imu") {
    if (nargs == 1 && (ieq(arg(0).c_str(), "present") || ieq(arg(0).c_str(), "absent"))) {
      if (!atZero()) return fail("imu present|absent is only allowed at t = 0");
      imuPresent_ = ieq(arg(0).c_str(), "present");
      c.type = CmdType::ImuPresent;
    } else if (nargs == 2 && ieq(arg(0).c_str(), "noise") &&
               (ieq(arg(1).c_str(), "on") || ieq(arg(1).c_str(), "off"))) {
      c.type = CmdType::ImuNoise;
      c.i0 = ieq(arg(1).c_str(), "on") ? 1 : 0;
    } else {
      return usage("imu present|absent  or  imu noise on|off");
    }
  } else if (cmd == "tod") {
    if (!needArgs(1, 1)) return usage("tod night|sunset|day");
    const std::string v = lower(arg(0));
    if (v == "night") c.i0 = (int)kj::Tod::Night;
    else if (v == "sunset") c.i0 = (int)kj::Tod::Sunset;
    else if (v == "day") c.i0 = (int)kj::Tod::Day;
    else return usage("tod night|sunset|day");
    c.type = CmdType::Tod;
  } else if (cmd == "view") {
    long v;
    if (!needArgs(1, 1) || !parseInt(arg(0), v) || v < 0 || v > 2) return usage("view 0|1|2");
    c.type = CmdType::View;
    c.i0 = (int)v;
  } else if (cmd == "state") {
    if (!needArgs(1, 1)) return usage("state title|ring|free");
    const std::string v = lower(arg(0));
    if (v == "title") c.i0 = 0;
    else if (v == "ring") c.i0 = 1;
    else if (v == "free") c.i0 = 2;
    else return usage("state title|ring|free");
    c.type = CmdType::State;
    c.s0 = v;
  } else if (cmd == "key") {
    if (!needArgs(2, 2)) return usage("key NAME down|up");
    c.i0 = keyFromName(arg(0).c_str());
    if (c.i0 < 0) return fail("unknown key '" + arg(0) + "' (see kj::KEY_NAMES)");
    const std::string v = lower(arg(1));
    if (v != "down" && v != "up") return usage("key NAME down|up");
    c.s1 = v;
    c.s0 = arg(0);
    c.type = CmdType::Key;
  } else if (cmd == "tap") {
    if (!needArgs(1, 1)) return usage("tap NAME");
    c.i0 = keyFromName(arg(0).c_str());
    if (c.i0 < 0) return fail("unknown key '" + arg(0) + "' (see kj::KEY_NAMES)");
    c.s0 = arg(0);
    c.type = CmdType::Tap;
  } else if (cmd == "hold") {
    if (!needArgs(2, 2)) return usage("hold NAME secs");
    c.i0 = keyFromName(arg(0).c_str());
    if (c.i0 < 0) return fail("unknown key '" + arg(0) + "' (see kj::KEY_NAMES)");
    if (!parseFloat(arg(1), c.f[0]) || c.f[0] < 0.0f) return usage("hold NAME secs");
    c.s0 = arg(0);
    c.nf = 1;
    c.type = CmdType::Hold;
  } else if (cmd == "tilt") {
    if (!needArgs(2, 3)) return usage("tilt rollDeg pitchDeg [rampSec]");
    for (size_t i = 0; i < nargs; ++i)
      if (!parseFloat(arg(i), c.f[i])) return usage("tilt rollDeg pitchDeg [rampSec]");
    if (nargs == 3 && c.f[2] < 0.0f) return fail("ramp must be >= 0");
    c.nf = (int)nargs;
    c.type = CmdType::Tilt;
  } else if (cmd == "pose") {
    if (!needArgs(1, 2)) return usage("pose holdDeg [rampSec]");
    for (size_t i = 0; i < nargs; ++i)
      if (!parseFloat(arg(i), c.f[i])) return usage("pose holdDeg [rampSec]");
    c.nf = (int)nargs;
    c.type = CmdType::Pose;
  } else if (cmd == "teleport") {
    if (!needArgs(4, 4)) return usage("teleport x y z yawDeg   (each may be ~ or ~N)");
    for (size_t i = 0; i < 4; ++i) {
      std::string a = arg(i);
      if (a[0] == '~') {
        c.rel[i] = 1;
        a = a.substr(1);
        if (a.empty()) {
          c.f[i] = 0.0f;
          continue;
        }
      }
      if (!parseFloat(a, c.f[i])) return fail("bad teleport value '" + arg(i) + "'");
    }
    c.nf = 4;
    c.type = CmdType::Teleport;
  } else if (cmd == "autopilot") {
    if (!needArgs(1, 1)) return usage("autopilot on|off");
    const std::string v = lower(arg(0));
    if (v != "on" && v != "off") return usage("autopilot on|off");
    c.i0 = v == "on" ? 1 : 0;
    c.type = CmdType::Autopilot;
  } else if (cmd == "cmd") {
    if (!needArgs(1, 1) || arg(0).size() != 1) return usage("cmd C   (one character)");
    c.s0 = arg(0);
    c.type = CmdType::SerialCmd;
  } else if (cmd == "bench") {
    long v;
    if (!needArgs(1, 1) || !parseInt(arg(0), v) || v < 1 || v > 1000000) return usage("bench FRAMES");
    c.i0 = (int)v;
    c.type = CmdType::Bench;
  } else if (cmd == "spawn") {
    long v;
    if (!needArgs(1, 2) || !parseInt(arg(0), v)) return usage("spawn SALT [ALT]");
    c.i0 = (int)v;
    c.f[0] = 40.0f;
    if (nargs == 2 && !parseFloat(arg(1), c.f[0])) return usage("spawn SALT [ALT]");
    c.nf = (int)nargs - 1;
    c.type = CmdType::Spawn;
  } else if (cmd == "seek") {
    if (nargs < 2) return usage("seek land|cloud|sea ALT [DIST] [cloudy]");
    const std::string kind = lower(arg(0));
    c.s0 = kind;
    c.type = CmdType::Seek;
    if (kind == "land") {
      if (!needArgs(2, 4) || !parseFloat(arg(1), c.f[0])) return usage("seek land ALT [DIST] [cloudy]");
      c.f[1] = 160.0f;
      for (size_t i = 2; i < nargs; ++i) {
        if (ieq(arg(i).c_str(), "cloudy")) c.i0 = 1;
        else if (!parseFloat(arg(i), c.f[1])) return usage("seek land ALT [DIST] [cloudy]");
      }
    } else if (kind == "cloud") {
      if (!needArgs(2, 3)) return usage("seek cloud ALT|under|in|above [DIST]");
      const std::string a = lower(arg(1));
      if (a == "under") c.s1 = a;
      else if (a == "in") c.s1 = a;
      else if (a == "above") c.s1 = a;
      else if (!parseFloat(arg(1), c.f[0])) return usage("seek cloud ALT|under|in|above [DIST]");
      c.f[1] = 0.0f;
      if (nargs == 3 && !parseFloat(arg(2), c.f[1])) return usage("seek cloud ALT|under|in|above [DIST]");
    } else if (kind == "sea") {
      if (!needArgs(2, 2) || !parseFloat(arg(1), c.f[0])) return usage("seek sea ALT");
    } else {
      return usage("seek land|cloud|sea ...");
    }
  } else if (cmd == "jetpose") {
    if (nargs == 1 && ieq(arg(0).c_str(), "off")) {
      c.i0 = 0;
    } else {
      if (!needArgs(3, 4)) return usage("jetpose yawDeg rollDeg pitchDeg [boost] | jetpose off");
      for (size_t i = 0; i < nargs; ++i)
        if (!parseFloat(arg(i), c.f[i])) return usage("jetpose yawDeg rollDeg pitchDeg [boost]");
      c.i0 = 1;
      c.nf = (int)nargs;
    }
    c.type = CmdType::JetPose;
  } else if (cmd == "resetmax") {
    if (!needArgs(0, 0)) return usage("resetmax");
    c.type = CmdType::ResetMax;
  } else if (cmd == "echo") {
    std::string text;
    for (size_t i = 0; i < nargs; ++i) {
      if (i) text += ' ';
      text += arg(i);
    }
    c.s0 = text;
    c.type = CmdType::Echo;
  } else if (cmd == "shot") {
    if (!needArgs(1, 1) || !validName(arg(0))) return usage("shot NAME   (letters, digits, _ and -)");
    c.s0 = arg(0);
    c.type = CmdType::Shot;
  } else if (cmd == "sheet") {
    long v;
    if (!needArgs(2, 2) || !validName(arg(0)) || !parseInt(arg(1), v) || v < 1 || v > 64)
      return usage("sheet NAME cols   (cols 1..64)");
    c.s0 = arg(0);
    c.i0 = (int)v;
    c.type = CmdType::Sheet;
  } else if (cmd == "assert" || cmd == "wait") {
    const bool isWait = cmd == "wait";
    if (isWait ? !needArgs(3, 4) : !needArgs(3, 3))
      return usage(isWait ? "wait VAR OP VALUE [timeoutSec]" : "assert VAR OP VALUE");
    c.f[1] = 30.0f;
    if (isWait && nargs == 4 && (!parseFloat(arg(3), c.f[1]) || c.f[1] <= 0.0f))
      return usage("wait VAR OP VALUE [timeoutSec]   (timeout > 0)");
    c.s0 = lower(arg(0));
    const std::string& op = arg(1);
    if (op == "<") c.op = AssertOp::LT;
    else if (op == "<=") c.op = AssertOp::LE;
    else if (op == ">") c.op = AssertOp::GT;
    else if (op == ">=") c.op = AssertOp::GE;
    else if (op == "==" || op == "=") c.op = AssertOp::EQ;
    else if (op == "!=") c.op = AssertOp::NE;
    else return fail("unknown assert operator '" + op + "' (< <= > >= == !=)");
    const std::string& v = arg(2);
    int st = stateFromName(v.c_str());
    if (st >= 0) c.f[0] = (float)st;
    else if (ieq(v.c_str(), "on") || ieq(v.c_str(), "true")) c.f[0] = 1.0f;
    else if (ieq(v.c_str(), "off") || ieq(v.c_str(), "false")) c.f[0] = 0.0f;
    else if (!parseFloat(v, c.f[0])) return fail("bad assert value '" + v + "'");
    c.s1 = op + " " + v;
    c.type = isWait ? CmdType::Wait : CmdType::Assert;
  } else if (cmd == "end") {
    if (!needArgs(0, 0)) return usage("end");
    c.type = CmdType::End;
  } else {
    return fail("unknown command '" + tok[1] + "'");
  }
  cmds_.push_back(c);
  return true;
}

// ------------------------------------------------------------------ Scheduler

void Scheduler::begin(const Script& s) {
  cmds_ = s.cmds();
  file_ = s.file();
  pc_ = 0;
  down_ = prevDown_ = 0;
  releases_.clear();
  serial_.clear();
  titleGoal_ = false;
  titleLine_ = titleStart_ = navNext_ = 0;
  benchLeft_ = 0;
  waitGoal_ = false;
  waitStart_ = 0;
  goalT_ = shift_ = 0.0;
  ended_ = failed_ = false;
  assertsRun_ = assertsFailed_ = 0;
}

bool Scheduler::isPost(CmdType t) {
  return t == CmdType::Shot || t == CmdType::Sheet || t == CmdType::Assert || t == CmdType::End ||
         t == CmdType::Echo || t == CmdType::Wait;
}

void Scheduler::finishGoal(int frame) {
  const double late = frameTime(frame) - (goalT_ + shift_);
  if (late > 0.0) shift_ += late;
}

bool Scheduler::evalCond(const Cmd& c, SimHost& host, float& v, bool& ok) {
  v = 0.0f;
  ok = false;
  if (!host.getVar(c.s0.c_str(), v)) {
    runtimeError(c,
                 std::string(c.type == CmdType::Wait ? "wait" : "assert") + ": unknown variable '" + c.s0 + "'",
                 host);
    return false;
  }
  const float b = c.f[0];
  const float tol = 1e-4f * (fabsf(b) > 1.0f ? fabsf(b) : 1.0f);
  switch (c.op) {
    case AssertOp::LT: ok = v < b; break;
    case AssertOp::LE: ok = v <= b; break;
    case AssertOp::GT: ok = v > b; break;
    case AssertOp::GE: ok = v >= b; break;
    case AssertOp::EQ: ok = fabsf(v - b) <= tol; break;
    case AssertOp::NE: ok = fabsf(v - b) > tol; break;
  }
  return true;
}

void Scheduler::runtimeError(const Cmd& c, const std::string& msg, SimHost& host) {
  char buf[600];
  snprintf(buf, sizeof(buf), "[SCRIPT] ERROR %s:%d: %s", file_.c_str(), c.line, msg.c_str());
  host.log(buf);
  failed_ = true;
}

void Scheduler::pressKey(int kp, int releaseFrame) {
  down_ |= 1ull << kp;
  for (auto& r : releases_)
    if (r.first == kp) {
      r.second = releaseFrame;
      return;
    }
  releases_.push_back(std::make_pair(kp, releaseFrame));
}

void Scheduler::stepTitleGoal(int frame, SimHost& host) {
  const int st = host.gameState();
  if (st == (int)kj::GameState::Title) {
    titleGoal_ = false;
    finishGoal(frame);
    return;
  }
  if (frame - titleStart_ > TITLE_TIMEOUT_FRAMES) {
    Cmd c;
    c.line = titleLine_;
    titleGoal_ = false;
    runtimeError(c, std::string("state title: TITLE not reached within 20 s (state ") + stateName(st) + ")",
                 host);
    return;
  }
  if (frame < navNext_) return;
  int kp = -1;
  switch ((kj::GameState)st) {
    case kj::GameState::Boot: break;  // wait for world generation / first render
    case kj::GameState::Calibrate:
      // let the boot calibration finish by itself (the synthetic IMU is still); skip if stuck
      if (frame - titleStart_ >= CALIBRATE_WAIT_FRAMES) kp = kj::KP_ENTER;
      break;
    case kj::GameState::Countdown:
    case kj::GameState::Play:
    case kj::GameState::Crash: kp = kj::KP_P; break;
    case kj::GameState::Pause: kp = kj::KP_BKSP; break;
    case kj::GameState::GameOver: kp = kj::KP_BKSP; break;  // ENTER = retry now
    default: break;
  }
  if (kp >= 0 && !(down_ & (1ull << kp))) {
    pressKey(kp, frame + 1);
    navNext_ = frame + NAV_GAP_FRAMES;
  }
}

void Scheduler::execPre(const Cmd& c, int frame, SimHost& host) {
  std::string err;
  switch (c.type) {
    case CmdType::Seed:
    case CmdType::Persist:
    case CmdType::ImuPresent:
      break;  // applied before Game::init
    case CmdType::Key:
      if (c.s1 == "down") {
        down_ |= 1ull << c.i0;
      } else {
        down_ &= ~(1ull << c.i0);
        for (size_t i = 0; i < releases_.size(); ++i)
          if (releases_[i].first == c.i0) {
            releases_.erase(releases_.begin() + (ptrdiff_t)i);
            break;
          }
      }
      break;
    case CmdType::Tap:
      pressKey(c.i0, frame + 1);
      break;
    case CmdType::Hold: {
      const int frames = (int)lround((double)c.f[0] * 60.0);
      pressKey(c.i0, frame + (frames < 1 ? 1 : frames));
      break;
    }
    case CmdType::SerialCmd:
      serial_.push_back(c.s0[0]);
      break;
    case CmdType::State:
      if (c.i0 == 0) {
        titleGoal_ = true;
        titleLine_ = c.line;
        titleStart_ = frame;
        navNext_ = frame;
        goalT_ = c.t;
        stepTitleGoal(frame, host);
      } else if (!host.execCommand(c, err)) {
        runtimeError(c, err, host);
      }
      break;
    case CmdType::Bench:
      benchLeft_ = c.i0;
      goalT_ = c.t + (double)(c.i0 - 1) / 60.0;  // a bench on schedule adds no extra shift
      host.benchBegin(c.i0);
      break;
    default:
      if (!host.execCommand(c, err)) runtimeError(c, err, host);
      break;
  }
}

void Scheduler::execPost(const Cmd& c, int frame, SimHost& host) {
  std::string err;
  if (c.type == CmdType::End) {
    ended_ = true;
    return;
  }
  if (c.type != CmdType::Assert && c.type != CmdType::Wait) {
    if (!host.execCommand(c, err)) runtimeError(c, err, host);
    return;
  }
  float v = 0.0f;
  bool ok = false;
  if (!evalCond(c, host, v, ok)) return;
  char buf[600];
  if (c.type == CmdType::Wait) {
    if (ok) {
      snprintf(buf, sizeof(buf), "[WAIT] ok   %s:%d: %s %s at once (t=%.3f, got %.4f)", file_.c_str(), c.line,
               c.s0.c_str(), c.s1.c_str(), frameTime(frame), (double)v);
      host.log(buf);
      return;
    }
    waitGoal_ = true;
    waitCmd_ = c;
    waitStart_ = frame;
    goalT_ = c.t;
    return;
  }
  ++assertsRun_;
  if (!ok) ++assertsFailed_;
  snprintf(buf, sizeof(buf), "[ASSERT] %s %s:%d t=%.3f: %s %s (got %.4f)", ok ? "ok  " : "FAIL", file_.c_str(),
           c.line, frameTime(frame), c.s0.c_str(), c.s1.c_str(), (double)v);
  host.log(buf);
}

void Scheduler::stepWaitGoal(int frame, SimHost& host) {
  float v = 0.0f;
  bool ok = false;
  if (!evalCond(waitCmd_, host, v, ok)) {
    waitGoal_ = false;
    return;
  }
  char buf[600];
  const double waited = (double)(frame - waitStart_) / 60.0;
  if (ok) {
    waitGoal_ = false;
    finishGoal(frame);
    snprintf(buf, sizeof(buf), "[WAIT] ok   %s:%d: %s %s after %.3f s (t=%.3f, got %.4f)", file_.c_str(),
             waitCmd_.line, waitCmd_.s0.c_str(), waitCmd_.s1.c_str(), waited, frameTime(frame), (double)v);
    host.log(buf);
  } else if (waited >= (double)waitCmd_.f[1]) {
    waitGoal_ = false;
    snprintf(buf, sizeof(buf), "wait: %s %s not reached within %.1f s (last value %.4f)", waitCmd_.s0.c_str(),
             waitCmd_.s1.c_str(), (double)waitCmd_.f[1], (double)v);
    runtimeError(waitCmd_, buf, host);
  }
}

void Scheduler::preFrame(int frame, SimHost& host, FrameKeys& out) {
  const double T = frameTime(frame);
  for (size_t i = 0; i < releases_.size();) {
    if (releases_[i].second <= frame) {
      down_ &= ~(1ull << releases_[i].first);
      releases_.erase(releases_.begin() + (ptrdiff_t)i);
    } else {
      ++i;
    }
  }
  if (titleGoal_ && !failed_) stepTitleGoal(frame, host);
  while (!blocked() && !ended_ && !failed_ && pc_ < cmds_.size()) {
    const Cmd& c = cmds_[pc_];
    if (!due(c, T) || isPost(c.type)) break;
    ++pc_;
    execPre(c, frame, host);
  }
  const uint64_t g0 = 1ull << KEY_G0;
  out.down = down_ & KEYS_MASK;
  out.pressed = (down_ & ~prevDown_) & KEYS_MASK;
  out.btnA = (down_ & g0) != 0;
  out.btnAPressed = (down_ & g0) != 0 && (prevDown_ & g0) == 0;
  out.serialCmd = 0;
  if (!serial_.empty()) {
    out.serialCmd = serial_.front();
    serial_.pop_front();
  }
  prevDown_ = down_;
}

void Scheduler::postFrame(int frame, SimHost& host) {
  if (benchLeft_ > 0 && --benchLeft_ == 0) {
    host.benchEnd();
    finishGoal(frame);
  }
  if (waitGoal_ && !failed_) stepWaitGoal(frame, host);
  const double T = frameTime(frame);
  while (!blocked() && !ended_ && !failed_ && pc_ < cmds_.size()) {
    const Cmd& c = cmds_[pc_];
    if (!due(c, T) || !isPost(c.type)) break;
    ++pc_;
    execPost(c, frame, host);
  }
}

}  // namespace sim
