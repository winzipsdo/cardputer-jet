// audio_synth.cpp - portable 16 kHz mono synth (DESIGN.md section 16). OWNER: group D.
// Continuous layers (engine saws, roar, wind, spray) ramp linearly across each block; the
// one-shot voices are evaluated in 32-sample chunks. No allocation, float only.
#include "audio.h"
#include <math.h>
#include <string.h>
#include "fastmath_d.h"

namespace kj {

namespace {

constexpr float INV_RATE = 1.0f / 16000.0f;
constexpr float TWO_PI = 6.28318531f;
constexpr int CHUNK = 32;

// Fast sine of a phase in [0,1) (parabolic approximation with one refinement step).
inline float fsin(float ph) {
  float p = 2.0f * ph - 1.0f;                 // [-1, 1)
  float y = -4.0f * p * (1.0f - fabsf(p));    // ~ sin(2 pi ph)
  return 0.225f * (y * fabsf(y) - y) + y;
}

// One-pole low-pass coefficient for cutoff fc (bilinear-ish rational approximation).
constexpr float lpCoef(float fc) {
  return (TWO_PI * fc * INV_RATE) / (1.0f + TWO_PI * fc * INV_RATE);
}

struct VoiceParams {
  float f;        // Hz of the tonal oscillator
  float tone;     // tonal amplitude
  float oct;      // octave partner amplitude
  float noise;    // noise amplitude
  float fc;       // noise filter cutoff (Hz)
  uint8_t wave;   // 0 sine, 1 square, 2 saw
  uint8_t nmode;  // 0 low-pass, 1 band, 2 high-pass
};

// Float <-> bits without memcpy (the ESP32 build turns a 4-byte memcpy into a real call).
union FloatBits {
  float f;
  uint32_t u;
};

// e^x without libm for the envelope range (|x| < 80, relative error < 2e-4): 2^(x log2 e)
// with the integer part in the exponent bits and a degree-5 polynomial for the fraction.
inline float fexp(float x) {
  float y = x * 1.44269504f;
  if (y < -125.0f) return 0.0f;
  if (y > 120.0f) y = 120.0f;
  int i = (int)y;                 // truncates toward 0 ...
  if ((float)i > y) --i;          // ... -> floor
  const float f = y - (float)i;   // [0, 1)
  const float p =
      1.0f + f * (0.693147181f + f * (0.240226507f + f * (0.0555041087f +
                                                           f * (0.00961812911f + f * 0.00133335581f))));
  FloatBits fb;
  fb.u = (uint32_t)(i + 127) << 23;
  return p * fb.f;
}

// sin(pi * t / T) for t in [0, T] (invT = 1 / T) (half-wave swell) via the parabolic fsin.
inline float halfSin(float t, float invT) { return fsin(0.5f * t * invT); }

inline float atk(float t) { return t < 0.004f ? t * 250.0f : 1.0f; }

// Envelope/pitch of each one-shot at time t (s). false = finished. Out of line (flash): one
// call per voice per chunk.
JET_NOINLINE bool voiceParams(Sfx sfx, float t, float p, VoiceParams& o) {
  o.f = 440.0f; o.tone = 0.0f; o.oct = 0.0f; o.noise = 0.0f; o.fc = 1000.0f;
  o.wave = 0; o.nmode = 0;
  switch (sfx) {
    case Sfx::Chime: {
      if (t > 0.7f) return false;
      float e = fexp(-t * (1.0f / 0.22f)) * atk(t);
      o.f = 1046.5f * p;
      o.tone = 0.32f * e;
      o.oct = 0.40f * e * fexp(-t * (1.0f / 0.10f));
      return true;
    }
    case Sfx::Perfect: {
      if (t > 1.0f) return false;
      float e = fexp(-t * (1.0f / 0.30f)) * atk(t);
      o.f = (t < 0.09f ? 1046.5f : 1568.0f) * p;
      o.tone = 0.30f * e;
      o.oct = 0.45f * e;
      return true;
    }
    case Sfx::Miss: {
      if (t > 0.34f) return false;
      o.wave = 1;
      o.f = t < 0.15f ? 392.0f : 294.0f;
      o.tone = 0.16f * atk(t) * (t < 0.15f ? 1.0f : fexp(-(t - 0.15f) * (1.0f / 0.08f)));
      return true;
    }
    case Sfx::Roll: {
      if (t > 0.4f) return false;
      o.nmode = 1;
      o.fc = 600.0f * fexp(t * (1.6094f / 0.4f));  // 600 -> 3000 Hz
      o.noise = 1.2f * halfSin(t, 1.0f / 0.4f);
      return true;
    }
    case Sfx::Ignite: {
      if (t > 0.4f) return false;
      o.wave = 2;
      o.f = 140.0f + 700.0f * t;
      o.tone = 0.26f * fexp(-t * (1.0f / 0.12f)) * atk(t);
      o.noise = 0.55f * fexp(-t * (1.0f / 0.05f));
      o.fc = 1800.0f;
      return true;
    }
    case Sfx::Slap: {
      if (t > 0.14f) return false;
      o.nmode = 2;
      o.fc = 1200.0f;
      o.noise = 0.65f * fexp(-t * (1.0f / 0.035f));
      o.f = 260.0f * p;
      o.tone = 0.25f * fexp(-t * (1.0f / 0.03f)) * atk(t);
      return true;
    }
    case Sfx::Whoosh: {
      if (t > 0.7f) return false;
      o.nmode = 1;
      o.fc = 400.0f + 2400.0f * fexp(-2.5f * t);
      o.noise = 1.3f * halfSin(t, 1.0f / 0.7f);
      return true;
    }
    case Sfx::Crash: {
      if (t > 1.8f) return false;
      o.fc = 300.0f + 3500.0f * fexp(-t * (1.0f / 0.25f));
      o.noise = 0.95f * fexp(-t * (1.0f / 0.5f)) * atk(t);
      o.wave = 2;
      o.f = 45.0f + 90.0f * fexp(-t * (1.0f / 0.35f));
      o.tone = 0.40f * fexp(-t * (1.0f / 0.6f));
      return true;
    }
    case Sfx::PullUp: {
      if (t > 0.08f) return false;
      o.wave = 1;
      o.f = 1000.0f;
      o.tone = 0.22f * atk(t);
      return true;
    }
    case Sfx::Tick: {
      if (t > 0.03f) return false;
      o.f = 2000.0f;
      o.tone = 0.28f * (1.0f - t * (1.0f / 0.03f));
      return true;
    }
    case Sfx::Blip: {
      if (t > 0.05f) return false;
      o.wave = 1;
      o.f = 1200.0f * p;
      o.tone = 0.15f * atk(t) * (1.0f - t * (1.0f / 0.05f));
      return true;
    }
    case Sfx::Go: {
      if (t > 0.4f) return false;
      o.wave = 1;
      o.f = 880.0f * p;
      o.tone = 0.17f * fexp(-t * (1.0f / 0.18f)) * atk(t);
      o.oct = 0.6f * o.tone;
      return true;
    }
    default:
      return false;
  }
}

}  // namespace

void Synth::init(uint32_t seed) {
  noise_ = seed ? seed : 0x1u;
  for (int i = 0; i < VOICES; ++i) voices_[i] = Voice{};
  ph1_ = ph2_ = lp1_ = lp2_ = bp1_ = bp2_ = hp_ = 0.0f;
  wd1_ = wd2_ = hpIn_ = 0.0f;
  burstT_ = 0.0f;
  burstAmp_ = 0.5f;
  cloud_ = 0.0f;
  eng_ = 0.0f;
  cur_ = target_;
}

void Synth::trigger(Sfx s, float pitch) {
  if ((int)s >= (int)Sfx::Count) return;
  if (!(pitch > 0.05f) || pitch > 8.0f) pitch = 1.0f;
  int slot = -1;
  // short repeating cues re-use their own voice instead of stacking
  if (s == Sfx::PullUp || s == Sfx::Tick || s == Sfx::Blip) {
    for (int i = 0; i < VOICES; ++i)
      if (voices_[i].on && voices_[i].sfx == (uint8_t)s) { slot = i; break; }
  }
  if (slot < 0)
    for (int i = 0; i < VOICES; ++i)
      if (!voices_[i].on) { slot = i; break; }
  if (slot < 0) {  // steal the oldest
    slot = 0;
    for (int i = 1; i < VOICES; ++i)
      if (voices_[i].t > voices_[slot].t) slot = i;
  }
  Voice& v = voices_[slot];
  v.sfx = (uint8_t)s;
  v.t = 0.0f;
  v.pitch = pitch;
  v.phase = 0.0f;
  v.state = 0.0f;
  v.s2 = 0.0f;
  v.on = true;
}

// ---------------------------------------------------------------------------------------
// Render. Cost notes (ESP32-S3: float divide, expf and sinf are slow library calls):
//  * per sample there is no divide and no libm call (the soft clip uses a Newton reciprocal);
//  * the filter / oscillator state lives in locals for the whole block;
//  * the shared white noise is generated once per 32-sample chunk into a small buffer;
//  * every active one-shot voice is rendered chunk-wise by a loop specialised for its
//    waveform / noise mode, so the per-sample mix has no switch and no per-voice branches;
//  * voice envelopes (no libm) are evaluated once per chunk, block ramps step per chunk.
namespace {

// 1 / d for d >= 1 (soft-clip denominator): bit-trick seed + 3 Newton steps (rel. err ~1e-7).
inline float recipGe1(float d) {
  FloatBits b;
  b.f = d;
  b.u = 0x7EF311C3u - b.u;
  float r = b.f;
  r = r * (2.0f - d * r);
  r = r * (2.0f - d * r);
  r = r * (2.0f - d * r);
  return r;
}

template <int W>
inline float waveT(float ph) {
  if constexpr (W == 1) return ph < 0.5f ? 0.6f : -0.6f;  // square
  else if constexpr (W == 2) return 1.4f * ph - 0.7f;  // saw
  else return fsin(ph);                                // sine
}

// Tonal oscillator (+ optional octave partner) of one voice, added into acc[0..len).
// The voice loops stay out of line (flash): they run 32 samples per call, so their I-cache
// misses are amortised, and Synth::render itself stays small enough for IRAM.
template <int W>
JET_NOINLINE void voiceTone(float* acc, int len, float& phase, float inc, float tone, float oct) {
  float ph = phase;
  if (oct > 0.0f) {
    for (int k = 0; k < len; ++k) {
      ph += inc;
      if (ph >= 1.0f) ph -= 1.0f;
      float p2 = ph * 2.0f;
      if (p2 >= 1.0f) p2 -= 1.0f;
      acc[k] += waveT<W>(ph) * tone + waveT<W>(p2) * oct;
    }
  } else {
    for (int k = 0; k < len; ++k) {
      ph += inc;
      if (ph >= 1.0f) ph -= 1.0f;
      acc[k] += waveT<W>(ph) * tone;
    }
  }
  phase = ph;
}

// Filtered-noise layer of one voice (nmode 0 low-pass, 1 band, 2 high-pass), added into acc.
JET_NOINLINE void voiceNoise(float* acc, const float* nzb, int len, const VoiceParams& p, float& state,
                float& s2) {
  const float a1 = lpCoef(p.fc), g = p.noise;
  float st = state;
  if (p.nmode == 0) {
    for (int k = 0; k < len; ++k) {
      st += a1 * (nzb[k] - st);
      acc[k] += st * g;
    }
  } else if (p.nmode == 1) {
    const float a2 = lpCoef(p.fc * 0.5f), g2 = 2.0f * g;
    float b = s2;
    for (int k = 0; k < len; ++k) {
      const float nz = nzb[k];
      st += a1 * (nz - st);
      b += a2 * (nz - b);
      acc[k] += (st - b) * g2;
    }
    s2 = b;
  } else {
    for (int k = 0; k < len; ++k) {
      const float nz = nzb[k];
      st += a1 * (nz - st);
      acc[k] += (nz - st) * g;
    }
  }
  state = st;
}

}  // namespace

JET_D_HOT void Synth::render(int16_t* out, int n) {
  if (!out || n <= 0) return;
  if (mute_) {
    memset(out, 0, sizeof(int16_t) * (size_t)n);
    for (int i = 0; i < VOICES; ++i) voices_[i].on = false;
    cur_ = target_;
    return;
  }
  const float invN = 1.0f / (float)n;

  // ---- block-level parameter ramps (start = cur_, end = target_)
  const AudioParams a0 = cur_, a1 = target_;
  const float engT = a1.engineOn ? 1.0f : 0.0f;
  const float eng0 = eng_;
  eng_ += (engT - eng_) * 0.45f;
  const float eng1 = eng_;
  const float cl0 = cloud_;
  cloud_ += ((a1.inCloud ? 1.0f : 0.0f) - cloud_) * 0.35f;
  const float cl1 = cloud_;

  // The engine ramps its phase increment (= f / RATE) directly.
  auto engInc = [](const AudioParams& a) {
    return (140.0f + 80.0f * a.speedRatio + 90.0f * a.boost) * INV_RATE;
  };
  const float inc1s = engInc(a0);
  const float dInc1 = (engInc(a1) - inc1s) * invN;
  const float lpas = lpCoef(700.0f + 1500.0f * a0.boost + 1000.0f * cl0);
  const float dlpa = (lpCoef(700.0f + 1500.0f * a1.boost + 1000.0f * cl1) - lpas) * invN;
  // Layer gains with their output trims folded in (engine x2, roar and wind x2.5).
  const float gEngs = 0.36f * eng0;
  const float dgEng = (0.36f * eng1 - gEngs) * invN;
  const float gRoars = 2.5f * (0.08f + 0.30f * a0.boost) * eng0;
  const float dgRoar = (2.5f * (0.08f + 0.30f * a1.boost) * eng1 - gRoars) * invN;
  auto windG = [](const AudioParams& a) {
    float w = a.wind < 0.0f ? 0.0f : (a.wind > 1.6f ? 1.6f : a.wind);
    return 0.3f * w * w;  // 2.5 * 0.12 * w^2
  };
  const float gWinds = windG(a0) * eng0;
  const float dgWind = (windG(a1) * eng1 - gWinds) * invN;
  const float gSprays = 0.45f * (a0.spray < 0.0f ? 0.0f : a0.spray);
  const float dgSpray = (0.45f * (a1.spray < 0.0f ? 0.0f : a1.spray) - gSprays) * invN;
  cur_ = target_;

  constexpr float aR1 = lpCoef(900.0f), aR2 = lpCoef(2200.0f);   // roar band ~1.5 kHz
  constexpr float aW1 = lpCoef(2200.0f), aW2 = lpCoef(4200.0f);  // wind band ~3 kHz
  constexpr float aS = lpCoef(2500.0f);                          // spray high-pass split

  // ---- state in locals for the whole block
  uint32_t ns = noise_;
  float ph1 = ph1_, ph2 = ph2_, lp1 = lp1_, lp2 = lp2_, bp1 = bp1_, bp2 = bp2_, hp = hp_;
  float wd1 = wd1_, wd2 = wd2_, hpIn = hpIn_;

  float nzb[CHUNK];
  float acc[CHUNK];

  for (int base = 0; base < n; base += CHUNK) {
    const int len = (n - base) < CHUNK ? (n - base) : CHUNK;
    const float chunkDt = (float)len * INV_RATE;
    // The block ramps advance per 32-sample chunk (2 ms steps, inaudible) and are evaluated at
    // the chunk centre: fewer live registers in the sample loop than per-sample ramps.
    const float mid = (float)base + 0.5f * (float)len;
    const float inc1 = inc1s + dInc1 * mid, lpa = lpas + dlpa * mid;
    const float gEng = gEngs + dgEng * mid, gRoar = gRoars + dgRoar * mid;
    const float gWind = gWinds + dgWind * mid, gSpray = gSprays + dgSpray * mid;
    const float inc2 = inc1 * 1.007f;

    // spray bursts: random 30-60 ms segments
    burstT_ -= chunkDt;
    if (burstT_ <= 0.0f) {
      ns ^= ns << 13; ns ^= ns >> 17; ns ^= ns << 5;
      float r1 = (float)(ns >> 8) * (1.0f / 16777216.0f);
      ns ^= ns << 13; ns ^= ns >> 17; ns ^= ns << 5;
      float r2 = (float)(ns >> 8) * (1.0f / 16777216.0f);
      burstT_ = 0.03f + 0.03f * r1;
      burstAmp_ = 0.2f + 0.8f * r2;
    }
    const float burstAmp = burstAmp_;

    // shared white noise in [-1, 1)
    for (int k = 0; k < len; ++k) {
      ns ^= ns << 13; ns ^= ns >> 17; ns ^= ns << 5;
      nzb[k] = (float)(int32_t)ns * (1.0f / 2147483648.0f);
      acc[k] = 0.0f;
    }

    // one-shot voices: parameters once per chunk, then one specialised loop per layer
    for (int i = 0; i < VOICES; ++i) {
      Voice& v = voices_[i];
      if (!v.on) continue;
      VoiceParams p;
      if (!voiceParams((Sfx)v.sfx, v.t, v.pitch, p)) {
        v.on = false;
        continue;
      }
      v.t += chunkDt;
      const float inc = p.f * INV_RATE;
      if (p.tone > 0.0f) {
        if (p.wave == 1) voiceTone<1>(acc, len, v.phase, inc, p.tone, p.oct);
        else if (p.wave == 2) voiceTone<2>(acc, len, v.phase, inc, p.tone, p.oct);
        else voiceTone<0>(acc, len, v.phase, inc, p.tone, p.oct);
      } else {
        // the oscillator phase keeps running while the tone is silent
        const float ph = v.phase + inc * (float)len;
        v.phase = ph - fm::floorF(ph);
      }
      if (p.noise > 0.0f) voiceNoise(acc, nzb, len, p, v.state, v.s2);
    }

    // continuous layers + mix + soft clip
    int16_t* o = out + base;
    for (int k = 0; k < len; ++k) {
      const float nz = nzb[k];

      // engine: two detuned saws through a 2-stage low-pass
      ph1 += inc1;
      if (ph1 >= 1.0f) ph1 -= 1.0f;
      ph2 += inc2;
      if (ph2 >= 1.0f) ph2 -= 1.0f;
      const float saw = (ph1 - 0.5f) + (ph2 - 0.5f);
      lp1 += lpa * (saw - lp1);
      lp2 += lpa * (lp1 - lp2);
      float x = lp2 * gEng;

      // roar: noise band-passed around 1.5 kHz
      bp1 += aR1 * (nz - bp1);
      bp2 += aR2 * ((nz - bp1) - bp2);
      x += bp2 * gRoar;

      // wind: noise band-passed around 3 kHz
      wd1 += aW1 * (nz - wd1);
      wd2 += aW2 * ((nz - wd1) - wd2);
      x += wd2 * gWind;

      // spray: high-passed noise with random bursts
      hp += aS * (nz - hp);
      hpIn += 0.01f * (burstAmp - hpIn);
      x += (nz - hp) * gSpray * hpIn;

      x += acc[k];

      // soft clip x / (1 + |x|) and output
      const float y = x * recipGe1(1.0f + fabsf(x));
      o[k] = (int16_t)(y * 30000.0f);
    }
  }

  noise_ = ns;
  ph1_ = ph1; ph2_ = ph2; lp1_ = lp1; lp2_ = lp2; bp1_ = bp1; bp2_ = bp2; hp_ = hp;
  wd1_ = wd1; wd2_ = wd2; hpIn_ = hpIn;
}

}  // namespace kj
