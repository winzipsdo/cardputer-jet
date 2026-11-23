// render_sky.cpp - sky gradient bands, stars with twinkle, Milky Way, moon / sun with halo,
// shooting stars. Fills every pixel the world march left EMPTY. OWNER: group B. DESIGN.md 7.4.
//
// Stars sit on the sky sphere at infinity: screen x = Cx + f tan(dAz), y = horizon -
// f tan(el)/cos(dAz). The 240-entry star table is drawn in four 90-degree azimuth copies with
// permuted elevations/classes (the field never repeats inside one frame: hFOV < 90 deg), which
// gives the visible sky band a dense, twinkling field without a bigger table.
//
// Cost notes (ESP32-S3): the gradient / haze fill works on 32-bit words (a word that is still
// all EMPTY is one store; a partly drawn word is merged branch-free); the moon / sun use a disc
// stencil precomputed at skyInit (no per-pixel sqrtf); the Milky Way glow evaluates its
// per-pixel test only inside the row interval where the band can be (one sqrtf per row). All
// three give bit-identical pixels to the straightforward per-pixel versions.
#include "render_world.h"

#include <math.h>
#include <string.h>

#include "compat.h"
#include "config.h"
#include "mathx.h"
#include "palette.h"

namespace kj {

namespace {

constexpr int NS = cfg::STARS;
constexpr int ND = cfg::MILKY_DUST;
constexpr int AZ_BITS = 12, AZ_N = 1 << AZ_BITS;  // 12-bit azimuth turn
constexpr int BUCKET_SHIFT = 6, NBUCKET = AZ_N >> BUCKET_SHIFT;  // 64 buckets
constexpr int STAR_COPIES = 4;
constexpr float Q14 = 16384.0f, INV_Q14 = 1.0f / 16384.0f;

// info bits: 0-1 class (0 bright, 1 mid, 2 dim, 3 warm), 2 hero, 3 twinkler, 4-5 rate-1
struct Star {
  int16_t ca, sa;     // cos / sin of azimuth, Q14 (direction = forward(az))
  uint16_t tanEl;     // tan(elevation), Q14
  uint8_t info;
  uint8_t phase;      // 0..15
};
struct Dust {
  int16_t ca, sa;
  uint16_t tanEl;
  uint8_t kind;       // 0 = MILKY dust, 1 = dim star in the band
  uint8_t pad;
};

Star gStars[NS];        // sorted by azimuth
uint16_t gStarAz[NS];
uint16_t gStarBucket[NBUCKET + 1];
Dust gDust[ND];
uint16_t gDustAz[ND];
uint16_t gDustBucket[NBUCKET + 1];
float gMwN[3];          // Milky Way great-circle normal (world x, y, z)
float gMwG[3];          // direction of the bright core of the band
uint32_t gSkyRngState = 0x5EED5u;  // xorshift state (plain POD: no static constructor)
// cos / sin of cfg::LIGHT_AZ (moon / sun azimuth), set by skyInit and used by skyMoonScreen.
float gLightCa = 0.0f, gLightSa = 0.0f;
bool gSkyReady = false;

struct Shooter {
  float timer;          // seconds until the next one
  float age;            // < 0 = inactive
  float az, tanEl;      // start point on the sky
  float dx, dy;         // screen direction (unit)
};
Shooter gShoot = {10.0f, -1.0f, 0.0f, 0.0f, 0.0f, 0.0f};

// Moon / sun disc stencils: one code byte per (dx, dy) in [-R, R]^2, R = (int)(3.2 r) + 1.
// bits 5-7 kind, bits 0-4 Bayer alpha. Built by skyInit with the exact per-pixel formulas.
enum : uint8_t { DK_NONE = 0, DK_DISC = 1, DK_CRATER = 2, DK_IN = 3, DK_OUT = 4 };
constexpr int discR(float r) { return (int)(3.2f * r) + 1; }
constexpr int MOON_SR = discR(cfg::MOON_R_PX), SUN_SR = discR(cfg::SUN_R_PX);
uint8_t gMoonSt[(2 * MOON_SR + 1) * (2 * MOON_SR + 1)];
uint8_t gSunSt[(2 * SUN_SR + 1) * (2 * SUN_SR + 1)];

inline int az12(float az) {
  int a = (int)floorf(az * ((float)AZ_N / TAU));
  a %= AZ_N;
  return a < 0 ? a + AZ_N : a;
}

template <typename T>
void sortByAz(T* items, uint16_t* az, int n) {
  for (int i = 1; i < n; ++i) {
    T it = items[i];
    uint16_t a = az[i];
    int j = i - 1;
    while (j >= 0 && az[j] > a) {
      items[j + 1] = items[j];
      az[j + 1] = az[j];
      --j;
    }
    items[j + 1] = it;
    az[j + 1] = a;
  }
}

void buildBuckets(const uint16_t* az, int n, uint16_t* bucket) {
  int i = 0;
  for (int b = 0; b <= NBUCKET; ++b) {
    while (i < n && (az[i] >> BUCKET_SHIFT) < b) ++i;
    bucket[b] = (uint16_t)i;
  }
}

inline bool isSkyBand(uint8_t idx, int minBand) {
  return idx >= pal::SKY0 + minBand && idx < pal::SKY0 + 8;
}

inline void plotSky(uint8_t* px, int x, int y, uint8_t idx, int minBand) {
  if (!inFrame(x, y)) return;
  uint8_t& p = px[y * LW + x];
  if (isSkyBand(p, minBand)) p = idx;
}

// Projects a direction at infinity. Returns false when behind / too oblique.
inline bool projInf(const View& v, float ca, float sa, float tanEl, float& sx, float& sy) {
  const float cosD = ca * v.cpsi + sa * v.spsi;
  if (cosD < 0.25f) return false;
  const float sinD = sa * v.cpsi - ca * v.spsi;
  const float inv = v.f / cosD;
  sx = v.Cx + sinD * inv;
  sy = v.horizon - tanEl * inv;
  return true;
}

inline int floorToInt(float f) { return (int)(f + 1024.0f) - 1024; }

// projInf for a point that is only drawn when it lands in the frame: points that would land
// more than 2 px outside are rejected with multiplies before the divide (bit-identical result
// for every point that can be drawn; star crosses reach 1 px beyond the centre).
inline bool projInfFrame(const View& v, float ca, float sa, float tanEl, float& sx, float& sy) {
  const float cosD = ca * v.cpsi + sa * v.spsi;
  if (cosD < 0.25f) return false;
  const float sinD = sa * v.cpsi - ca * v.spsi;
  const float sF = sinD * v.f, tF = tanEl * v.f;
  if (sF < (-2.0f - v.Cx) * cosD || sF > ((float)LW + 2.0f - v.Cx) * cosD) return false;
  if (tF > (v.horizon + 2.0f) * cosD || tF < (v.horizon - (float)LH - 2.0f) * cosD) return false;
  const float inv = v.f / cosD;
  sx = v.Cx + sinD * inv;
  sy = v.horizon - tanEl * inv;
  return true;
}

// ------------------------------------------------------------------ moon / sun
// Disc of radius r px with a two-ring halo: disc (d <= 1 + 0.5/r radii; crater pixels or, for
// the sun, a warm limb), inner glow to 1.9 r (Bayer density (1.9 - d) * 14, else outer glow at
// 3/16), outer glow to 3.2 r (density <= 3/16).
void buildDiscStencil(uint8_t* st, float r, bool sun) {
  const int R = discR(r), N = 2 * R + 1;
  const float invR = 1.0f / r;
  for (int dy = -R; dy <= R; ++dy) {
    for (int dx = -R; dx <= R; ++dx) {
      const float d = sqrtf((float)(dx * dx + dy * dy)) * invR;  // distance in radii
      uint8_t code = DK_NONE << 5;
      if (d <= 1.0f + 0.5f * invR) {
        code = DK_DISC << 5;
        if (sun) {
          if (d > 1.0f - 1.2f * invR) code = DK_CRATER << 5;  // warm limb
        } else if ((dx == -1 && dy == -1) || (dx == 1 && dy == 0)) {
          code = DK_CRATER << 5;
        }
      } else if (d < 1.9f) {
        int a = (int)((1.9f - d) * 14.0f);
        if (a < 0) a = 0;
        if (a > 31) a = 31;
        code = (uint8_t)((DK_IN << 5) | a);
      } else if (d < 3.2f) {
        int a = (int)((3.2f - d) * 2.5f + 0.5f);
        if (a > 3) a = 3;
        if (a > 0) code = (uint8_t)((DK_OUT << 5) | a);
      }
      st[(dy + R) * N + dx + R] = code;
    }
  }
}

void drawDisc(uint8_t* px, int cx, int cy, bool sun) {
  const uint8_t* st = sun ? gSunSt : gMoonSt;
  const int R = sun ? SUN_SR : MOON_SR, N = 2 * R + 1;
  const int y0 = cy - R < 0 ? 0 : cy - R, y1 = cy + R > LH - 1 ? LH - 1 : cy + R;
  const int x0 = cx - R < 0 ? 0 : cx - R, x1 = cx + R > LW - 1 ? LW - 1 : cx + R;
  for (int y = y0; y <= y1; ++y) {
    const int base = (y - cy + R) * N + R - cx;  // st[base + x] = code at (x, y)
    uint8_t* row = px + y * LW;
    for (int x = x0; x <= x1; ++x) {
      const uint8_t code = st[base + x];
      if (!code) continue;
      uint8_t& p = row[x];
      if (!pal::isSky(p)) continue;
      const int kind = code >> 5, a = code & 31;
      uint8_t idx = 0;
      if (kind == DK_DISC) idx = pal::MOON_DISC;
      else if (kind == DK_CRATER) idx = pal::MOON_CRATER;
      else if (kind == DK_IN) idx = bayerPass(x, y, a) ? pal::MOON_GLOW_IN : bayerPass(x, y, 3) ? pal::MOON_GLOW_OUT : 0;
      else if (bayerPass(x, y, a)) idx = pal::MOON_GLOW_OUT;
      if (idx) p = idx;
    }
  }
}

// ------------------------------------------------------------------ word-wise EMPTY fill
#if defined(__GNUC__) || defined(__clang__)
#define SKY_ALIGNED4(p) static_cast<uint8_t*>(__builtin_assume_aligned((p), 4))
#else
#define SKY_ALIGNED4(p) (p)
#endif

// Writes pattern byte (x & 3) into every EMPTY (0) pixel of one row. pat holds the four bytes
// in memory order (little endian: byte k = x & 3 == k). row must be 4-byte aligned.
#if defined(__GNUC__) || defined(__clang__)
typedef uint32_t __attribute__((may_alias)) SkyU32;
#else
typedef uint32_t SkyU32;
#endif
// (Plain aliased word access: the xtensa toolchain does NOT inline a 4-byte memcpy, each one
// was a call to the ROM memcpy - about 2/3 of the whole sky stage.)
inline void fillRowEmpty(uint8_t* row, uint32_t pat) {
  SkyU32* r = reinterpret_cast<SkyU32*>(SKY_ALIGNED4(row));
  for (int i = 0; i < LW / 4; ++i) {
    uint32_t w = r[i];
    if (w == 0u) {
      r[i] = pat;
    } else {
      // high bit of each byte of z set iff that byte is 0 (EMPTY)
      const uint32_t z = ~(((w & 0x7F7F7F7Fu) + 0x7F7F7F7Fu) | w) & 0x80808080u;
      if (z) {
        const uint32_t m = (z - (z >> 7)) | z;  // 0xFF in the EMPTY bytes
        w |= pat & m;
        r[i] = w;
      }
    }
  }
}

// Same, one byte at a time (frame buffer not word aligned).
inline void fillRowEmptyBytes(uint8_t* row, uint32_t pat) {
  for (int x = 0; x < LW; ++x)
    if (row[x] == pal::EMPTY) row[x] = (uint8_t)(pat >> (8 * (x & 3)));
}

inline uint32_t pack4(uint8_t b0, uint8_t b1, uint8_t b2, uint8_t b3) {
  return (uint32_t)b0 | ((uint32_t)b1 << 8) | ((uint32_t)b2 << 16) | ((uint32_t)b3 << 24);
}

// ------------------------------------------------------------------ Milky Way glow
void drawMilkyGlow(uint8_t* px, const View& v) {
  if (v.horizon <= 0.0f) return;
  const float fwd[3] = {v.spsi, -v.cpsi, 0.0f};
  const float rgt[3] = {v.cpsi, v.spsi, 0.0f};
  const float invF = 1.0f / v.f;
  // dir(sx, sy) = fwd + rgt*(sx - Cx)/f + up*(horizon - sy)/f
  const float nF = gMwN[0] * fwd[0] + gMwN[1] * fwd[1];
  const float nR = gMwN[0] * rgt[0] + gMwN[1] * rgt[1];
  const float nU = gMwN[2];
  const float gF = gMwG[0] * fwd[0] + gMwG[1] * fwd[1];
  const float gR = gMwG[0] * rgt[0] + gMwG[1] * rgt[1];
  const float gU = gMwG[2];
  const float s2 = 0.01485f;  // sin^2(7 deg)
  int yEnd = (int)ceilf(v.horizon);
  if (yEnd > LH) yEnd = LH;
  // In a row the test r2 < lim is the quadratic Q(a) = A a^2 + B a + C < 0 (A = nR^2 - s2). When
  // A is clearly positive the band covers one interval of a; the exact per-pixel test below is
  // run only there, widened by 2 px and by a generous float-error margin, so the result is the
  // same as testing every pixel. Otherwise (band nearly level on screen) every pixel is tested.
  const float A = nR * nR - s2;
  const bool useInterval = A > 0.01f;
  const float inv2A = useInterval ? 0.5f / A : 0.0f;
  for (int y = 0; y < yEnd; ++y) {
    if ((y & 1) != 0) continue;  // bayer(x, y) < 4 only on even rows (BAYER4 rows 1 and 3 are >= 4)
    const float b = (v.horizon - ((float)y + 0.5f)) * invF;
    const float rowN = nF + nU * b, rowG = gF + gU * b;
    const float rowL = 1.0f + b * b;
    int xBeg = 0, xEnd = LW;
    if (useInterval) {
      const float B = 2.0f * rowN * nR, C = rowN * rowN - s2 * rowL;
      const float B2 = B * B, AC4 = 4.0f * A * C;
      const float disc = B2 - AC4 + 1.0e-5f * (B2 + fabsf(AC4));
      if (disc < 0.0f) continue;
      const float hw = sqrtf(disc) * inv2A, av = -B * inv2A;
      float xl = v.Cx - 0.5f + v.f * (av - hw) - 2.0f;
      float xh = v.Cx - 0.5f + v.f * (av + hw) + 2.0f;
      if (xh < 0.0f || xl > (float)(LW - 1)) continue;
      if (xl < 0.0f) xl = 0.0f;
      if (xh > (float)(LW - 1)) xh = (float)(LW - 1);
      xBeg = (int)xl;
      xEnd = (int)xh + 2;
      if (xEnd > LW) xEnd = LW;
    }
    for (int x = xBeg; x < xEnd; ++x) {
      const int by = bayer(x, y);
      if (by >= 4) continue;  // max density 4/16 (3 + core)
      uint8_t& p = px[y * LW + x];
      if (!isSkyBand(p, 1)) continue;  // fades out into the horizon haze
      const float a = ((float)x + 0.5f - v.Cx) * invF;
      const float dn = rowN + nR * a;
      const float len2 = rowL + a * a;
      const float r2 = dn * dn;
      const float lim = s2 * len2;
      if (r2 >= lim) continue;
      int dens = r2 < 0.2f * lim ? 3 : 2;
      const float g = rowG + gR * a;
      if (g > 0.0f && g * g > 0.8f * len2) ++dens;  // brighter core of the band
      if (p == pal::SKY0 + 1) --dens;              // lowest band: half strength
      if (by < dens) p = pal::MILKY;
    }
  }
}

}  // namespace

// ================================================================== init
void skyInit(uint32_t seed) {
  gLightCa = cosf(cfg::LIGHT_AZ);
  gLightSa = sinf(cfg::LIGHT_AZ);
  Rng rng(hash32(seed ^ 0x57A2F1E5u) | 1u);
  // ---- stars
  for (int i = 0; i < NS; ++i) {
    const float az = rng.range(0.0f, TAU);
    float elDeg;
    const float u = rng.uniform();
    if (i < 6) elDeg = rng.range(6.0f, 20.0f);           // heroes sit where they are seen
    else if (u < 0.04f) elDeg = rng.range(0.5f, 4.0f);
    else if (u < 0.64f) elDeg = rng.range(4.0f, 18.0f);   // the strip the cruise view shows
    else if (u < 0.93f) elDeg = rng.range(18.0f, 38.0f);
    else elDeg = rng.range(38.0f, 55.0f);
    Star s;
    s.ca = (int16_t)floorf(cosf(az) * (Q14 - 1.0f) + 0.5f);
    s.sa = (int16_t)floorf(sinf(az) * (Q14 - 1.0f) + 0.5f);
    s.tanEl = (uint16_t)(tanf(elDeg * DEG) * Q14);
    int cls;
    const float c = rng.uniform();
    if (i < 6) cls = 0;
    else if (c < 0.25f) cls = 2;
    else if (c < 0.35f) cls = 3;
    else if (c < 0.55f) cls = 0;
    else cls = 1;
    const int hero = i < 6 ? 1 : 0;
    const int twinkle = (hero || rng.irange(0, 2) == 0) ? 1 : 0;
    const int rate = rng.irange(0, 3);
    s.info = (uint8_t)(cls | (hero << 2) | (twinkle << 3) | (rate << 4));
    s.phase = (uint8_t)rng.irange(0, 15);
    gStars[i] = s;
    gStarAz[i] = (uint16_t)az12(az);
  }
  sortByAz(gStars, gStarAz, NS);
  buildBuckets(gStarAz, NS, gStarBucket);

  // ---- Milky Way: great circle through the horizon at az0 (20 degrees left of the moon), tilted
  // 62 degrees and rising away from the moon, so it stands beside the moon in the start view.
  const float az0 = cfg::LIGHT_AZ - 0.35f;
  const float tilt = 62.0f * DEG;
  const float u0[3] = {sinf(az0), -cosf(az0), 0.0f};                    // horizon crossing
  const float hp[3] = {sinf(az0 - 0.5f * PI), -cosf(az0 - 0.5f * PI), 0.0f};
  const float w0[3] = {cosf(tilt) * hp[0], cosf(tilt) * hp[1], sinf(tilt)};
  gMwN[0] = u0[1] * w0[2] - u0[2] * w0[1];
  gMwN[1] = u0[2] * w0[0] - u0[0] * w0[2];
  gMwN[2] = u0[0] * w0[1] - u0[1] * w0[0];
  {
    const float th = 0.25f;  // bright core about 13 degrees above the horizon
    gMwG[0] = cosf(th) * u0[0] + sinf(th) * w0[0];
    gMwG[1] = cosf(th) * u0[1] + sinf(th) * w0[1];
    gMwG[2] = cosf(th) * u0[2] + sinf(th) * w0[2];
  }
  for (int i = 0; i < ND; ++i) {
    // Favour the low part of the band (what the chase camera sees).
    float th = rng.range(0.03f, PI - 0.03f);
    if (rng.uniform() < 0.6f) th = rng.uniform() < 0.5f ? rng.range(0.03f, 0.9f) : rng.range(PI - 0.9f, PI - 0.03f);
    const float off = (rng.uniform() + rng.uniform() - 1.0f) * (7.0f * DEG);
    float d[3];
    for (int k = 0; k < 3; ++k) d[k] = cosf(th) * u0[k] + sinf(th) * w0[k] + off * gMwN[k];
    const float hl = sqrtf(d[0] * d[0] + d[1] * d[1]);
    if (d[2] < 0.01f * hl) d[2] = 0.01f * hl;
    const float az = atan2f(d[0], -d[1]);  // forward(az) = (sin az, -cos az)
    Dust ds;
    ds.ca = (int16_t)floorf(cosf(az) * (Q14 - 1.0f) + 0.5f);
    ds.sa = (int16_t)floorf(sinf(az) * (Q14 - 1.0f) + 0.5f);
    float te = d[2] / hl;
    if (te > 3.9f) te = 3.9f;
    ds.tanEl = (uint16_t)(te * Q14);
    ds.kind = (uint8_t)(rng.irange(0, 5) == 0 ? 1 : 0);
    ds.pad = 0;
    gDust[i] = ds;
    gDustAz[i] = (uint16_t)az12(az);
  }
  sortByAz(gDust, gDustAz, ND);
  buildBuckets(gDustAz, ND, gDustBucket);

  Rng srng(hash32(seed ^ 0x0DEC0DE5u) | 1u);
  gShoot.timer = srng.range(cfg::SHOOTING_STAR_MIN_S, cfg::SHOOTING_STAR_MAX_S);
  gSkyRngState = srng.s;
  gShoot.age = -1.0f;
  buildDiscStencil(gMoonSt, cfg::MOON_R_PX, false);
  buildDiscStencil(gSunSt, cfg::SUN_R_PX, true);
  gSkyReady = true;
}

bool skyMoonScreen(const View& v, Tod tod, int& x, int& y) {
  if (tod == Tod::Day) return false;
  const float elTan = tod == Tod::Night ? cfg::MOON_EL_TAN : cfg::SUN_EL_TAN_SUNSET;
  const float ca = gSkyReady ? gLightCa : cosf(cfg::LIGHT_AZ), sa = gSkyReady ? gLightSa : sinf(cfg::LIGHT_AZ);
  float sx, sy;
  if (!projInf(v, ca, sa, elTan, sx, sy)) return false;
  if (sx < -40.0f || sx > (float)LW + 40.0f || sy < -40.0f || sy > (float)LH + 40.0f) return false;
  x = floorToInt(sx);
  y = floorToInt(sy);
  return true;
}

// ================================================================== render
void renderSky(FrameBuf& fb, const View& v, Tod tod, float time, float dt, RenderStats& st) {
  (void)st;
  uint8_t* const px = fb.px;
  const float hz = v.horizon;

  // ---- gradient bands (rows above the horizon) and haze (rows at / below it)
  const float span = cfg::SKY_SPAN_FRAC * (float)LH;
  const float tPerRow = 7.0f / span;
  // The glitter path continues across the far haze band (beyond the march) to the horizon.
  int pathX = -1000, pathY = 0;
  const bool pathOn = tod != Tod::Day && skyMoonScreen(v, tod, pathX, pathY);
  const int t8 = (int)(time * 8.0f);
  const bool aligned = ((uintptr_t)px & 3u) == 0u;
  static_assert(LW % 4 == 0 && pal::EMPTY == 0, "word fill needs 4-px rows and EMPTY == 0");
  for (int y = 0; y < LH; ++y) {
    uint8_t* row = px + y * LW;
    uint32_t pat;
    if ((float)y >= hz) {
      if (pathOn) {
        // glitter path: |x - pathX| < halfW  <=>  |x - pathX| <= kmax (integers)
        float halfW = 3.0f + 0.25f * ((float)y - hz);
        if (halfW > 512.0f) halfW = 512.0f;  // wider than any |x - pathX| (pathX is within 40 px of the frame)
        const int ih = (int)halfW;  // halfW >= 3
        const int kmax = (float)ih < halfW ? ih : ih - 1;
        const int xa = pathX - kmax < 0 ? 0 : pathX - kmax;
        const int xb = pathX + kmax > LW - 1 ? LW - 1 : pathX + kmax;
        for (int x = xa; x <= xb; ++x) {
          if (row[x] != pal::EMPTY) continue;
          const uint32_t h = hash3i(x, y, t8) & 255u;
          row[x] = h < 18u ? pal::GLINT_A : h < 110u ? pal::MOONPATH : pal::HAZE;
        }
      }
      pat = pack4(pal::HAZE, pal::HAZE, pal::HAZE, pal::HAZE);
    } else {
      float t = (hz - (float)y) * tPerRow;
      if (t > 7.999f) t = 7.999f;
      const int band = (int)t;
      const int bandB = band < 7 ? band + 1 : 7;
      const int thr = (int)((t - (float)band - 0.7f) * 54.0f);  // > bayer -> use bandB
      const uint8_t iA = (uint8_t)(pal::SKY0 + band), iB = (uint8_t)(pal::SKY0 + bandB);
      pat = pack4(thr > bayer(0, y) ? iB : iA, thr > bayer(1, y) ? iB : iA, thr > bayer(2, y) ? iB : iA,
                  thr > bayer(3, y) ? iB : iA);
    }
    if (aligned) fillRowEmpty(row, pat);
    else fillRowEmptyBytes(row, pat);
  }

  if (tod == Tod::Day || hz <= 0.0f || !gSkyReady) {
    if (tod != Tod::Night) gShoot.age = -1.0f;
    return;
  }

  // ---- moon (night) / sun (sunset); only on sky pixels so clouds and islands hide it
  {
    int mx, my;
    if (skyMoonScreen(v, tod, mx, my)) {
      const bool sun = tod == Tod::Sunset;
      drawDisc(px, mx, my, sun);
    }
  }
  if (tod != Tod::Night) {
    gShoot.age = -1.0f;
    return;
  }

  // ---- visible azimuth window (12-bit turn units)
  const int psi12 = az12(v.psi);
  int halfW = (int)(atanf(((float)LW * 0.5f + 8.0f) / v.f) * ((float)AZ_N / TAU)) + 8;
  if (halfW > AZ_N / 4) halfW = AZ_N / 4;

  // ---- Milky Way: faint glow band + dust points
  drawMilkyGlow(px, v);
  {
    const int b0 = (psi12 - halfW + 2 * AZ_N) >> BUCKET_SHIFT, b1 = (psi12 + halfW + 2 * AZ_N) >> BUCKET_SHIFT;
    for (int bb = b0; bb <= b1; ++bb) {
      const int b = bb & (NBUCKET - 1);
      for (int i = gDustBucket[b]; i < gDustBucket[b + 1]; ++i) {
        const Dust& d = gDust[i];
        float sx, sy;
        if (!projInfFrame(v, (float)d.ca * INV_Q14, (float)d.sa * INV_Q14, (float)d.tanEl * INV_Q14, sx, sy)) continue;
        const int x = floorToInt(sx), y = floorToInt(sy);
        if (!inFrame(x, y)) continue;
        if (d.kind == 1) plotSky(px, x, y, pal::STAR_DIM, 1);
        else if (bayer(x, y) < 8) plotSky(px, x, y, pal::MILKY, 1);
      }
    }
  }

  // ---- stars: four azimuth copies with permuted elevation / class, never in the lowest 2 bands
  const int tw40 = (int)(time * 40.0f);
  for (int c = 0; c < STAR_COPIES; ++c) {
    const int base = psi12 - c * (AZ_N / STAR_COPIES) + 2 * AZ_N;
    const int b0 = (base - halfW) >> BUCKET_SHIFT, b1 = (base + halfW) >> BUCKET_SHIFT;
    for (int bb = b0; bb <= b1; ++bb) {
      const int b = bb & (NBUCKET - 1);
      for (int i = gStarBucket[b]; i < gStarBucket[b + 1]; ++i) {
        const Star& sp = gStars[i];
        const Star& sa = gStars[(i + 61 * c) % NS];  // attributes of this copy
        // Rotate the azimuth by c * 90 degrees: (cos, sin) -> (-sin, cos).
        float ca = (float)sp.ca * INV_Q14, sn = (float)sp.sa * INV_Q14;
        for (int r = 0; r < c; ++r) {
          const float tmp = ca;
          ca = -sn;
          sn = tmp;
        }
        float sx, sy;
        if (!projInfFrame(v, ca, sn, (float)sa.tanEl * INV_Q14, sx, sy)) continue;
        const int x = floorToInt(sx), y = floorToInt(sy);
        if (!inFrame(x, y)) continue;
        const int cls = sa.info & 3;
        const bool hero = (sa.info & 4) != 0 && c == 0;
        const bool twinkler = (sa.info & 8) != 0;
        const int rate = ((sa.info >> 4) & 3) + 1;
        const int ph = (tw40 * rate + sa.phase * 16) & 255;
        uint8_t idx = cls == 0 ? pal::STAR_BRIGHT : cls == 1 ? pal::STAR_MID : cls == 2 ? pal::STAR_DIM : pal::STAR_WARM;
        if (twinkler && ph < 90) {
          if (idx == pal::STAR_BRIGHT) idx = pal::STAR_MID;
          else if (idx == pal::STAR_MID || idx == pal::STAR_WARM) idx = pal::STAR_DIM;
          else idx = pal::STAR_MID;
        }
        if (hero) {
          idx = pal::STAR_BRIGHT;
          if (ph > 200) {  // twinkle peak: 5-px cross
            plotSky(px, x - 1, y, pal::STAR_MID, 2);
            plotSky(px, x + 1, y, pal::STAR_MID, 2);
            plotSky(px, x, y - 1, pal::STAR_MID, 2);
            plotSky(px, x, y + 1, pal::STAR_MID, 2);
          }
        }
        plotSky(px, x, y, idx, 2);
      }
    }
  }

  // ---- shooting star
  if (gShoot.age < 0.0f) {
    Rng gSkyRng(gSkyRngState);
    gShoot.timer -= dt;
    if (gShoot.timer <= 0.0f) {
      if (hz > 22.0f) {
        const float sx0 = gSkyRng.range(20.0f, 100.0f);
        const float sy0 = gSkyRng.range(2.0f, hz - 16.0f);
        const float dAz = atanf((sx0 - v.Cx) / v.f);
        gShoot.az = v.psi + dAz;
        gShoot.tanEl = (hz - sy0) * cosf(dAz) / v.f;
        float ddx = gSkyRng.range(0.8f, 1.0f) * (gSkyRng.chance(0.5f) ? 1.0f : -1.0f);
        float ddy = gSkyRng.range(0.35f, 0.6f);
        const float l = 1.0f / sqrtf(ddx * ddx + ddy * ddy);
        gShoot.dx = ddx * l;
        gShoot.dy = ddy * l;
        gShoot.age = 0.0f;
        gShoot.timer = gSkyRng.range(cfg::SHOOTING_STAR_MIN_S, cfg::SHOOTING_STAR_MAX_S);
      } else {
        gShoot.timer = 1.0f;
      }
    }
    gSkyRngState = gSkyRng.s;
  }
  if (gShoot.age >= 0.0f) {
    const float life = 0.5f;
    const float p = gShoot.age / life;
    float sx, sy;
    if (p <= 1.0f && projInf(v, cosf(gShoot.az), sinf(gShoot.az), gShoot.tanEl, sx, sy)) {
      const float len = 14.0f * p;
      const float hx = sx + gShoot.dx * len, hy = sy + gShoot.dy * len;
      int tail = (int)len;
      if (tail > 6) tail = 6;
      for (int k = tail; k >= 0; --k) {
        const uint8_t idx = k < 2 ? pal::STAR_BRIGHT : k < 4 ? pal::STAR_MID : pal::STAR_DIM;
        plotSky(px, floorToInt(hx - gShoot.dx * (float)k), floorToInt(hy - gShoot.dy * (float)k), idx, 0);
      }
    }
    gShoot.age += dt;
    if (gShoot.age > life) gShoot.age = -1.0f;
  }
}

}  // namespace kj
