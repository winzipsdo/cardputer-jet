// perf.cpp - per-stage profiler and work counters (see perf.h).
// OWNER: group A. Portable C++17: identical on the device and in the simulator.
//
// Window semantics (used by the [STAT] line, the device bench and the slow-frame check):
//   * begin/end/add accumulate into the CURRENT frame's per-stage microseconds.
//   * endFrame() folds the current frame into the window sums (and maxima) and zeroes it.
//   * avgMs(s) = window sum of stage s / frames in the window, so avgMs(s) * frames is the
//     exact window sum (main.cpp derives per-frame values from consecutive sums).
//   * reset() starts a new window; the current (in-progress) frame is not touched.
#include "perf.h"

#include <stdarg.h>
#include <stdio.h>

#include "config.h"

namespace kj {

namespace {

#if defined(__GNUC__) || defined(__clang__)
__attribute__((format(printf, 4, 5)))
#endif
int appendf(char* buf, int n, int pos, const char* fmt, ...) {
  if (buf == nullptr || n <= 0) return 0;
  if (pos >= n - 1) return n - 1;
  va_list ap;
  va_start(ap, fmt);
  const int w = vsnprintf(buf + pos, (size_t)(n - pos), fmt, ap);
  va_end(ap);
  if (w < 0) {
    buf[pos] = '\0';
    return pos;
  }
  const int np = pos + w;
  return np > n - 1 ? n - 1 : np;
}

inline float msF(uint64_t us, uint32_t frames) {
  return frames ? (float)us / ((float)frames * 1000.0f) : 0.0f;
}

inline unsigned perFrame(uint32_t sum, uint32_t frames) {
  return frames ? (unsigned)(sum / frames) : 0u;
}

}  // namespace

void Profiler::init(ClockUs clock) {
  clock_ = clock;
  for (int i = 0; i < ST_COUNT; ++i) {
    t0_[i] = 0;
    frameUs_[i] = 0;
  }
  cur_.clear();
  reset();
}

void Profiler::begin(Stage s) {
  if (clock_ != nullptr && s < ST_COUNT) t0_[s] = clock_();
}

void Profiler::end(Stage s) {
  if (clock_ != nullptr && s < ST_COUNT) frameUs_[s] += clock_() - t0_[s];
}

void Profiler::add(Stage s, uint32_t us) {
  if (s < ST_COUNT) frameUs_[s] += us;
}

void Profiler::endFrame(uint32_t frameUs) {
  uint32_t core = 0;
  for (int i = 0; i < ST_COUNT; ++i) {
    if (i != ST_DMAWAIT) core += frameUs_[i];
    sumUs_[i] += frameUs_[i];
    frameUs_[i] = 0;
  }
  if (core > maxCoreUs_) maxCoreUs_ = core;
  if (frameUs > maxFrameUs_) maxFrameUs_ = frameUs;
  windowUs_ += frameUs;
  ++frames_;
  sum_.steps += cur_.steps;
  sum_.samples += cur_.samples;
  sum_.fills += cur_.fills;
  sum_.voxels += cur_.voxels;
  sum_.voxWrites += cur_.voxWrites;
  sum_.flamePx += cur_.flamePx;
  sum_.particles += cur_.particles;
  sum_.linePx += cur_.linePx;
  // cur_ is NOT cleared here: Game::render() clears it at the start of every frame, and the
  // platform / simulator may still read this frame's counters after endFrame().
}

bool Profiler::windowReady() const {
  const uint64_t periodUs = (uint64_t)(cfg::STAT_PERIOD_S * 1000000.0f);
  return frames_ > 0 && windowUs_ >= periodUs;
}

int Profiler::format(char* buf, int n) const {
  if (buf == nullptr || n <= 0) return 0;
  buf[0] = '\0';
  const uint32_t f = frames_;
  int p = 0;
  p = appendf(buf, n, p, "fps=%.1f ft=%.1f/%.1f c1=%.2f/%.2f", (double)fps(),
              (double)msF(windowUs_, f), (double)maxFrameUs_ / 1000.0, (double)coreMsAvg(),
              (double)maxCoreUs_ / 1000.0);
  for (int i = 0; i < ST_COUNT; ++i)
    p = appendf(buf, n, p, " %s=%.2f", STAGE_NAMES[i], (double)msF(sumUs_[i], f));
  p = appendf(buf, n, p, " | smp=%u stp=%u fill=%u vox=%u vw=%u fl=%u prt=%u lp=%u",
              perFrame(sum_.samples, f), perFrame(sum_.steps, f), perFrame(sum_.fills, f),
              perFrame(sum_.voxels, f), perFrame(sum_.voxWrites, f), perFrame(sum_.flamePx, f),
              perFrame(sum_.particles, f), perFrame(sum_.linePx, f));
  return p;
}

float Profiler::estEspMs(const CostModel& cm) const {
  if (frames_ == 0 || cm.cyclesPerUs <= 0.0f) return 0.0f;
  const float cycles = (float)sum_.samples * cm.cSample + (float)sum_.fills * cm.cFill +
                       (float)sum_.steps * cm.cStep + (float)sum_.voxels * cm.cVoxel +
                       (float)sum_.voxWrites * cm.cVoxWrite + (float)sum_.flamePx * cm.cFlame +
                       (float)sum_.particles * cm.cParticle + (float)sum_.linePx * cm.cLinePx;
  return cycles / (cm.cyclesPerUs * 1000.0f * (float)frames_);
}

void Profiler::reset() {
  for (int i = 0; i < ST_COUNT; ++i) sumUs_[i] = 0;
  maxCoreUs_ = 0;
  maxFrameUs_ = 0;
  windowUs_ = 0;
  frames_ = 0;
  sum_.clear();
}

float Profiler::fps() const {
  if (frames_ == 0 || windowUs_ == 0) return 0.0f;
  return (float)frames_ * 1000000.0f / (float)windowUs_;
}

float Profiler::avgMs(Stage s) const {
  if (s >= ST_COUNT) return 0.0f;
  return msF(sumUs_[s], frames_);
}

float Profiler::coreMsAvg() const {
  uint64_t core = 0;
  for (int i = 0; i < ST_COUNT; ++i)
    if (i != ST_DMAWAIT) core += sumUs_[i];
  return msF(core, frames_);
}

float Profiler::frameMsMax() const { return (float)maxFrameUs_ / 1000.0f; }

}  // namespace kj
