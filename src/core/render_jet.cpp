// render_jet.cpp - voxel-splat renderer for the "Kestrel" jet: shadow, lit body, afterburner,
// outline, nav lights. OWNER: group C. Design: DESIGN.md sections 6.2 - 6.4.
//
// ESP32-S3 cost notes (budget for the whole jet stage ~0.7 ms at 240 MHz):
//  - No float divide, sqrtf, powf, sinf, fmodf, floorf or ceilf runs per voxel / sample /
//    pixel (they are soft-float or out-of-line flash calls on this toolchain); the few left
//    run once per frame in the setup functions, which live in flash (not JET_HOT).
//  - Perspective per voxel: m = mJ / (1 + x), x = q * dDepth / den_j, |x| <= ~0.33 for the
//    jet: 4th-order polynomial (error <= x^5, ~0.02 px at the extremes). The old 256-entry
//    S/den LUT cost 512 soft divides per frame.
//  - Exact back-face culling from each voxel's exposed faces (JetModel::renderKeys()): a voxel
//    is splatted only if one of its exposed faces has the eye on its outer side (about half of
//    the shell). Perspective-correct, so no pinholes appear at the silhouette.
//  - The 4 sub-samples (+ up-axis gap fill, or the 2x2 "big" splat) of a voxel only depend on
//    its sub-pixel position (offsets are at the pivot scale), so they are precomputed per frame
//    as 8 x 8 "stamps" of deduplicated pixel offsets: a voxel writes 2-3 px instead of 4-6.
//  - Shadow over flat ground (sea, plateaus: the common case) is a linear map per voxel plus
//    the same perspective polynomial: no per-sample cell lookup, no divide.
//  - Hot tables are non-const (DRAM), never flash rodata.
#include "render_jet.h"

#include <math.h>
#include "compat.h"
#include "config.h"
#include "draw.h"
#include "jet_model.h"
#include "mathx.h"
#include "palette.h"
#include "world.h"

namespace kj {

namespace {

// Model anchor points (model units relative to the pivot: a across, b forward, s up).
constexpr float NOZZLE_B = (float)cfg::JET_PIVOT_ROW - cfg::NOZZLE_ROW - 0.5f;   // -14.5 (exit face)
constexpr float NOZZLE_S = cfg::NOZZLE_SLICE - (float)cfg::JET_PIVOT_SLICE;     // 0
constexpr float TIP_A = 12.5f;                                                 // outermost column
constexpr float TIP_B = (float)cfg::JET_PIVOT_ROW - 27.0f;                     // nav-light row
constexpr float CLIMB_CURL = 0.30f;

constexpr int GZ = cfg::JET_GRID_Z;
// Upper bound of |voxel offset from the pivot| in u (12.5, 19, 6 -> 23.5).
constexpr float JET_RADIUS_U = 24.0f;
// The perspective polynomial is used while |x| <= this (else the exact per-voxel divide).
constexpr float X_POLY_MAX = 0.40f;

// ---- DRAM tables (non-const: .data / .bss, not flash rodata)
uint8_t s_col[JET_NORMAL_COUNT * JET_SLOT_COUNT];   // [normal * 19 + slot], 0 = culled
uint8_t s_xm[256], s_ym[256];                       // visible-face bits by raw a2 / b byte
float s_nrm[JET_NORMAL_COUNT][3];                   // unit normals (model axes)
float s_heatT[5] = {0.28f, 0.5f, 0.75f, 1.0f, 1.25f};   // flame heat thresholds -> FLAME0..5
constexpr int POW_N = 64;
float s_pow075[POW_N + 2];                          // x^0.75 at x = i / POW_N
float s_heightU[16];                                // cfg::HEIGHT_U as float
uint8_t s_cls[256];                                 // outline classes
uint8_t s_bayer[16] = {0, 8, 2, 10, 12, 4, 14, 6, 3, 11, 1, 9, 15, 7, 13, 5};   // == BAYER4
Vec3 s_light = {0.0f, 0.0f, 1.0f};
bool s_ready = false;

constexpr uint8_t CLS_SKIP = 1;   // jet range (30..49) or flame: never becomes outline
constexpr uint8_t CLS_BODY = 2;   // jet body (31..49): makes its 4-neighbours outline

JET_NOINLINE void initTables() {
  for (int i = 0; i < JET_NORMAL_COUNT; ++i) {
    const int8_t* d = JET_NORMAL_DIR[i];
    const float inv = 1.0f / sqrtf((float)(d[0] * d[0] + d[1] * d[1] + d[2] * d[2]));
    for (int j = 0; j < 3; ++j) s_nrm[i][j] = (float)d[j] * inv;
  }
  for (int i = 0; i <= POW_N + 1; ++i) {
    const float x = (float)i / (float)POW_N;
    s_pow075[i] = x >= 1.0f ? 1.0f : powf(x, 0.75f);
  }
  for (int i = 0; i < 16; ++i) s_heightU[i] = (float)cfg::HEIGHT_U[i];
  for (int i = 0; i < 256; ++i) {
    const uint8_t c = (uint8_t)i;
    uint8_t k = 0;
    if ((c >= pal::JET_FIRST && c <= pal::JET_LAST) || pal::isFlame(c)) k |= CLS_SKIP;
    if (pal::isJetBody(c)) k |= CLS_BODY;
    s_cls[i] = k;
  }
  s_light = normalize(forwardOf(cfg::LIGHT_AZ) + v3(0.0f, 0.0f, 1.1f));
  s_ready = true;
}

inline void ensureTables() {
  if (JET_UNLIKELY(!s_ready)) initTables();
}

// x^0.75 for x in [0, 1] (table + lerp; replaces powf in the flame loop).
inline float pow075(float x) {
  if (x <= 0.0f) return 0.0f;
  const float f = x * (float)POW_N;
  int i = (int)f;
  if (i > POW_N) i = POW_N;
  const float fr = f - (float)i;
  return s_pow075[i] + (s_pow075[i + 1] - s_pow075[i]) * fr;
}

// 1 / (1 + x) for |x| <= X_POLY_MAX: (1 - x)(1 + x^2) + x^4 = 1 - x + x^2 - x^3 + x^4.
inline float recip1p(float x) {
  const float x2 = x * x;
  return (1.0f - x) * (1.0f + x2) + x2 * x2;
}

float flameLength(const JetPose& p) {
  float flick = 0.82f + 0.18f * sinf(47.0f * p.time) * sinf(31.0f * p.time);
  return (cfg::FLAME_LEN_CRUISE + cfg::FLAME_LEN_BOOST * sat(p.boost)) * flick;
}

Vec3 flameCentre(const JetAxes& ax, const JetPose& p, const Vec3& noz, float t, float len) {
  float k = t * t / (len > 0.1f ? len : 0.1f);
  return noz - ax.fwd * t + ax.right * (k * cfg::FLAME_CURL * p.yawRateN) +
         ax.up * (k * CLIMB_CURL * p.climbRateN);
}

inline float bump(float u, float c, float invW) {
  float d = (u - c) * invW;
  d = d < 0.0f ? -d : d;
  return d < 1.0f ? 1.0f - d : 0.0f;
}

inline int fastRound(float v) { return (int)(v + 1024.5f) - 1024; }   // valid for v > -1024
inline int fastFloor(float v) { return (int)(v + 1024.0f) - 1024; }   // valid for v > -1024
inline int ceilPos(float v) {                                         // ceil for v >= 0
  int i = (int)v;
  return (float)i < v ? i + 1 : i;
}
inline float absf(float v) { return v < 0.0f ? -v : v; }

// Phase of t (>= 0) within period P (replaces fmodf; t stays far below 2^31 * P).
inline float phaseOf(float t, float P, float invP) { return t - P * (float)(int)(t * invP); }

// project() with the divide replaced by a Newton-refined reciprocal of the previous call's
// den (consecutive flame samples differ by ~0.5 u in depth: e ~ 0.006, error e^3 ~ 2e-7).
struct NearProj {
  const View& v;
  float r = 0.0f;
  bool have = false;
  explicit NearProj(const View& view) : v(view) {}
  bool project(const Vec3& p, ScreenPt& out, float minDen) {
    float dx = p.x - v.fx, dy = p.y - v.fy;
    const float W = 8192.0f, H = 4096.0f;
    if (dx >= H) dx -= W; else if (dx < -H) dx += W;
    if (dy >= H) dy -= W; else if (dy < -H) dy += W;
    const float across = dx * v.cpsi + dy * v.spsi;
    const float depth = dx * v.spsi - dy * v.cpsi;
    const float den = 1.0f + v.q * depth;
    if (den < minDen) return false;
    const float e = 1.0f - den * r;
    if (!have || e > 0.05f || e < -0.05f) {
      r = 1.0f / den;
      have = true;
    } else {
      r = r * (1.0f + e * (1.0f + e));
    }
    const float m = v.S * r;
    out.x = v.Cx + m * across;
    out.y = v.Cy - m * ((p.z - v.alt) + v.T * depth);
    out.m = m;
    out.zc = v.dist * den;
    out.dq = depthQ(out.zc);
    return true;
  }
};

struct NavPt {
  float x, y;
  uint8_t dq, color;
};

struct HaloPt {
  int16_t x, y;
  float rs;
  uint8_t dq;
};

// ---- splat stamps: per sub-pixel position (8 x 8 bins), the deduplicated pixel offsets a
// voxel writes. Offsets are relative to (floor(sx), floor(sy)) and lie in [-3, 4].
constexpr int ST_FRAC = 8;
constexpr int ST_CASES = ST_FRAC * ST_FRAC;
constexpr int ST_BIAS = 16;                  // px bias: trunc == floor for sx, sy > -ST_BIAS
constexpr float ST_OFF_MAX = 2.4f;           // |sub-sample offset| limit for the stamp path
constexpr int ST_OFF0 = 3 * LW + 3;          // off + ST_OFF0 = (dy + 3) * LW + (dx + 3) >= 0
constexpr int FILL_DQ_BIAS = 2;              // back fill is written 2 depth steps behind

// "Back fill" (fill[d], nf[d]): the 4 sub-samples of the voxel's neighbour away from the eye
// along the up axis, d = 0 below (eye above the voxel), 1 above, minus pixels already in the
// stamp. Splatted only for voxels whose neighbour there is solid (that face not exposed): it
// stands in for that neighbour, which back-face culling skips but which the old draw-everything
// renderer splatted (without it, slabs seen at a shallow angle show 1 px pinholes: the splat
// samples R and F, not U). It is depth-tested and stored FILL_DQ_BIAS behind the voxel, so it
// only fills pixels no voxel of the surface covers (real splats at the same depth win either
// way): the colours of the old renderer are kept.
struct StampHead {
  uint8_t n;          // stamp pixels in off[]
  uint8_t nf[2];      // back-fill pixels in fill[d]
  uint8_t pad;
  int16_t fill[2][4];
};
template <int N>
struct Stamp : StampHead {
  int16_t off[N];   // dy * LW + dx
};
Stamp<6> s_stS[ST_CASES];    // normal splat: 4 sub-samples + up to 2 up-axis gap-fill samples
Stamp<18> s_stB[ST_CASES];   // 2x2 "big" splat (m >= JET_SPLAT_BIG_M): 4 x 2x2 + up to 2

// Everything renderJet needs per frame, computed by jetFrameSetup() (flash, once per frame).
struct JetFrame {
  JetAxes ax;
  float Aj, Dj, Yj, denJ, mJ, zJ;
  uint8_t dqPivot;
  float Ra, Rd, Ry, Fa, Fd, Fy, Ua, Ud, Uy;   // view-space axes (R per a2 step)
  bool navOn, strobeOn, noseFirst;
  int order[GZ];
  bool hidden[GZ];
  float drop[GZ];
  uint8_t zm[GZ];                              // visible-face bits per slice
  float offX[4], offY[4], uX[2], uY[2];
  int nu;
  float fillX, fillY;                          // screen offset of the neighbour below (+U * -1)
  float asmb;
  // fast (stamp) path
  bool fast;
  float tBig;                                  // t >= tBig <=> m >= JET_SPLAT_BIG_M
  float xR, xF, aR, aF, yR, yF, cx8;           // per a2 / b coefficients (see renderJet)
  float xs[GZ], As[GZ], Ys[GZ], cy8[GZ];       // per-slice bases
  uint8_t fillDir[GZ];                         // 0: back fill below (eye above the slice), 1: above
  int fx8, fy8;                                // fixed-point window of "stamp fully inside"
  unsigned fxN8, fyN8;
  int box[4];                                  // body box x0, x1, y0, y1 (old rule, see setup)
};
JetFrame s_fr;

// One set of stamps: heads at st0 + c * stride bytes, pixel offsets at off0 + c * stride
// (non-template, so the per-frame code exists once in flash).
JET_NOINLINE void buildStamps(StampHead* st0, int16_t* off0, int stride, int maxN, bool big,
                              const float* ox, const float* oy, const float* ux, const float* uy,
                              int nu, float fx, float fy, unsigned dirs, int ext[4]) {
  // Rounding of (bin centre + offset) per axis, per sample: 8 bins x (4 + nu + 8 fill), then
  // combined per case. Samples ns + 4d + j are the back-fill sub-samples (d 0 below, 1 above).
  int8_t rx[14][ST_FRAC], ry[14][ST_FRAC];
  const int ns = 4 + nu;
  for (int j = 0; j < ns + 8; ++j) {
    if (j >= ns && !((dirs >> ((j - ns) >> 2)) & 1u)) continue;   // direction not in use
    float ofx, ofy;
    if (j < 4) { ofx = ox[j]; ofy = oy[j]; }
    else if (j < ns) { ofx = ux[j - 4]; ofy = uy[j - 4]; }
    else {
      const int jj = (j - ns) & 3;
      const float sg = j - ns < 4 ? 1.0f : -1.0f;
      ofx = ox[jj] + sg * fx; ofy = oy[jj] + sg * fy;
    }
    const bool fl = big && j < 4;   // big samples: floor, then a 2x2 block; others: round
    for (int c = 0; c < ST_FRAC; ++c) {
      const float f = ((float)c + 0.5f) * (1.0f / ST_FRAC);
      rx[j][c] = (int8_t)(fl ? fastFloor(f + ofx) : fastRound(f + ofx));
      ry[j][c] = (int8_t)(fl ? fastFloor(f + ofy) : fastRound(f + ofy));
      // pixel extents of every stamp (a 2x2 block reaches one further)
      const int e = fl ? 1 : 0;
      if (rx[j][c] < ext[0]) ext[0] = rx[j][c];
      if (rx[j][c] + e > ext[1]) ext[1] = rx[j][c] + e;
      if (ry[j][c] < ext[2]) ext[2] = ry[j][c];
      if (ry[j][c] + e > ext[3]) ext[3] = ry[j][c] + e;
    }
  }
  for (int c = 0; c < ST_CASES; ++c) {
    const int cxb = c & (ST_FRAC - 1), cyb = c / ST_FRAC;
    StampHead& sh = *(StampHead*)((uint8_t*)st0 + c * stride);
    int16_t* off = (int16_t*)((uint8_t*)off0 + c * stride);
    uint32_t seen[2] = {0u, 0u};
    int n = 0;
    auto add = [&](int dx, int dy) {
      const int bit = (dy + 3) * 8 + (dx + 3);   // dx, dy in [-3, 4]
      const uint32_t b = 1u << (bit & 31);
      if (seen[bit >> 5] & b) return;
      seen[bit >> 5] |= b;
      if (n >= maxN) return;
      off[n] = (int16_t)(dy * LW + dx);
      ++n;
    };
    for (int j = 0; j < ns; ++j) {
      const int x = rx[j][cxb], y = ry[j][cyb];
      add(x, y);
      if (big && j < 4) { add(x + 1, y); add(x, y + 1); add(x + 1, y + 1); }
    }
    sh.n = (uint8_t)n;
    for (int d = 0; d < 2; ++d) {
      uint32_t seenF[2] = {seen[0], seen[1]};
      int nf = 0;
      for (int j = 0; j < ((dirs >> d) & 1u ? 4 : 0); ++j) {   // only the directions in use
        const int dx = rx[ns + 4 * d + j][cxb], dy = ry[ns + 4 * d + j][cyb];
        const int bit = (dy + 3) * 8 + (dx + 3);
        const uint32_t b = 1u << (bit & 31);
        if (seenF[bit >> 5] & b) continue;
        seenF[bit >> 5] |= b;
        sh.fill[d][nf++] = (int16_t)(dy * LW + dx);
      }
      sh.nf[d] = (uint8_t)nf;
    }
  }
}

// Per-frame setup of renderJet (flash; once per frame). Returns false if the jet is culled.
JET_NOINLINE bool jetFrameSetup(const View& v, const JetModel& m, const JetPose& p, JetFrame& F) {
  ensureTables();
  const JetAxes ax = jetAxes(p);
  F.ax = ax;
  const float c = v.cpsi, s = v.spsi;
  // Pivot in view space.
  const float pdx = wrapd(p.pos.x - v.fx), pdy = wrapd(p.pos.y - v.fy);
  F.Aj = pdx * c + pdy * s;
  F.Dj = pdx * s - pdy * c;
  F.Yj = (p.pos.z - v.alt) + v.T * F.Dj;
  F.denJ = 1.0f + v.q * F.Dj;
  if (F.denJ < cfg::JET_CULL_DEN) return false;
  const float invDenJ = 1.0f / F.denJ;
  F.mJ = v.S * invDenJ;
  F.zJ = v.dist * F.denJ;
  F.dqPivot = depthQ(F.zJ);

  // Axes in view space (A across, D depth, Y screen-up incl. the lens-shift term).
  F.Ra = 0.5f * (ax.right.x * c + ax.right.y * s);   // per a2 step (half voxel)
  F.Rd = 0.5f * (ax.right.x * s - ax.right.y * c);
  F.Ry = 0.5f * ax.right.z + v.T * F.Rd;
  F.Fa = ax.fwd.x * c + ax.fwd.y * s;
  F.Fd = ax.fwd.x * s - ax.fwd.y * c;
  F.Fy = ax.fwd.z + v.T * F.Fd;
  F.Ua = ax.up.x * c + ax.up.y * s;
  F.Ud = ax.up.x * s - ax.up.y * c;
  F.Uy = ax.up.z + v.T * F.Ud;

  // Eye relation and lighting.
  const Vec3 e = v3(wrapd(v.eyeX - p.pos.x), wrapd(v.eyeY - p.pos.y), v.eyeZ - p.pos.z);
  const Vec3 toEye = normalize(e);
  const float eyeR = dot(e, ax.right), eyeUp = dot(e, ax.up), eyeFwd = dot(e, ax.fwd);
  const Vec3 L = s_light;
  const float tm = p.time < 0.0f ? 0.0f : p.time;
  F.navOn = phaseOf(tm, cfg::NAV_BLINK_PERIOD, 1.0f / cfg::NAV_BLINK_PERIOD) <
            cfg::NAV_BLINK_PERIOD * cfg::NAV_BLINK_DUTY;
  const float sph = phaseOf(tm, cfg::STROBE_PERIOD, 1.0f / cfg::STROBE_PERIOD);
  F.strobeOn = sph < 0.06f || (sph >= 0.16f && sph < 0.22f);
  const uint8_t glowCol = p.boost > 0.5f ? (uint8_t)(pal::FLAME0 + 1) : pal::JET_GLOW;

  for (int i = 0; i < JET_NORMAL_COUNT; ++i) {
    const int8_t* d = JET_NORMAL_DIR[i];
    const float* un = s_nrm[i];
    Vec3 nw = ax.right * un[0] + ax.fwd * un[1] + ax.up * un[2];
    float de = dot(nw, toEye);
    // Cull threshold: a quantised normal averages several exposed faces, so a bevel voxel
    // (e.g. a leading edge whose top face the eye sees) can point up to ~55 deg past the
    // perpendicular. -0.6 keeps those; true back faces are still dropped.
    bool vis = de >= -0.6f;
    // Plate normals (0,0,1) and (+-1,0,1) are two-sided: seen from below, light the underside.
    if (d[1] == 0 && d[2] == 1 && de < 0.0f) {
      nw = ax.right * un[0] + ax.fwd * un[1] - ax.up * un[2];
      vis = true;
    }
    uint8_t* col = s_col + i * JET_SLOT_COUNT;
    if (!vis) {
      for (int k = 0; k < JET_SLOT_COUNT; ++k) col[k] = 0;
      continue;
    }
    float dl = dot(nw, L);
    int shift = dl > cfg::JET_LIT_DOT ? 1 : (dl < cfg::JET_DARK_DOT ? -2 : (dl < cfg::JET_DIM_DOT ? -1 : 0));
    Vec3 rf = nw * (2.0f * dl) - L;        // reflect(-L, n)
    int spec = dot(rf, toEye) > cfg::JET_SPEC_DOT ? 1 : 0;
    for (int t = 0; t < 7; ++t) col[t] = (uint8_t)(pal::JET_BODY0 + clampi(t + shift + spec, 0, 6));
    for (int t = 0; t < 4; ++t) col[7 + t] = (uint8_t)(pal::JET_ACCENT0 + clampi(t + shift, 0, 3));
    for (int t = 0; t < 3; ++t) col[11 + t] = (uint8_t)(pal::JET_CANOPY0 + clampi(t + shift + spec, 0, 2));
    col[14] = pal::JET_NOZZLE;
    col[15] = glowCol;
    col[16] = F.navOn ? pal::NAV_RED : (uint8_t)(pal::JET_ACCENT0 + 1);
    col[17] = F.navOn ? pal::NAV_GREEN : (uint8_t)(pal::JET_BODY0 + 3);
    col[18] = F.strobeOn ? pal::STROBE : (uint8_t)(pal::JET_ACCENT0 + 2);
  }

  // Exact back-face culling: a voxel face is visible iff the eye lies on its outer side of the
  // face plane (perspective-correct, per voxel). +x face of the voxel at a = a2/2 is the plane
  // a + 0.5: visible iff eyeR > a + 0.5; and so on. Tables by raw (int8) coordinate byte.
  for (int a2 = -25; a2 <= 25; ++a2) {
    const float a = 0.5f * (float)a2;
    s_xm[(uint8_t)a2] = (uint8_t)((eyeR > a + 0.5f ? JF_PX : 0) | (eyeR < a - 0.5f ? JF_NX : 0));
  }
  for (int b = cfg::JET_PIVOT_ROW - (cfg::JET_GRID_Y - 1); b <= cfg::JET_PIVOT_ROW; ++b) {
    const float fb = (float)b;
    s_ym[(uint8_t)b] = (uint8_t)((eyeFwd > fb + 0.5f ? JF_PY : 0) | (eyeFwd < fb - 0.5f ? JF_NY : 0));
  }
  for (int k = 0; k < GZ; ++k) {
    const float sv = (float)(k - cfg::JET_PIVOT_SLICE);
    F.zm[k] = (uint8_t)((eyeUp > sv + 0.5f ? JF_PZ : 0) | (eyeUp < sv - 0.5f ? JF_NZ : 0));
  }

  // Slice order: farthest from the eye (in the jet's up axis) first.
  float key[GZ];
  const float euc = clampf(eyeUp, -(float)cfg::JET_PIVOT_SLICE, (float)(GZ - 1 - cfg::JET_PIVOT_SLICE));
  for (int k = 0; k < GZ; ++k) {
    float dk = (float)(k - cfg::JET_PIVOT_SLICE) - euc;
    float kk = dk < 0.0f ? -dk : dk;
    int j = k;
    while (j > 0 && key[j - 1] < kk) { key[j] = key[j - 1]; F.order[j] = F.order[j - 1]; --j; }
    key[j] = kk; F.order[j] = k;
  }
  F.noseFirst = eyeFwd < 0.0f;   // eye behind: the nose end is far

  // Assembly intro: per-slice drop (bottom first); not-yet-started slices are hidden.
  F.asmb = sat(p.assemble);
  for (int k = 0; k < GZ; ++k) {
    F.drop[k] = 0.0f; F.hidden[k] = false;
    if (F.asmb < 1.0f) {
      float total = cfg::ASSEMBLE_STAGGER * (float)GZ + cfg::ASSEMBLE_FALL;
      float a = (F.asmb * total - cfg::ASSEMBLE_STAGGER * (float)k) * (1.0f / cfg::ASSEMBLE_FALL);
      if (a <= 0.0f) { F.hidden[k] = true; continue; }
      a = sat(a);
      F.drop[k] = -cfg::ASSEMBLE_DROP_PX * (1.0f - a * a);   // falls in, accelerating
    }
  }

  // Sub-sample offsets (+-0.25 R +- 0.25 F) at the pivot scale, plus up-axis gap fill.
  {
    const float rA = 2.0f * F.Ra, rY = 2.0f * F.Ry;   // unit right
    int j = 0;
    for (int sa = -1; sa <= 1; sa += 2)
      for (int sb = -1; sb <= 1; sb += 2) {
        F.offX[j] = F.mJ * 0.25f * ((float)sa * rA + (float)sb * F.Fa);
        F.offY[j] = -F.mJ * 0.25f * ((float)sa * rY + (float)sb * F.Fy);
        ++j;
      }
  }
  F.uX[0] = F.uX[1] = F.uY[0] = F.uY[1] = 0.0f;
  F.nu = 0;
  F.fillX = -F.mJ * F.Ua;   // one voxel down the up axis, on screen
  F.fillY = F.mJ * F.Uy;
  {
    float usx = F.mJ * F.Ua, usy = -F.mJ * F.Uy;
    float ulen = sqrtf(usx * usx + usy * usy);
    if (ulen > 1.8f) {
      F.nu = 2;
      F.uX[0] = usx * (1.0f / 3.0f); F.uY[0] = usy * (1.0f / 3.0f);
      F.uX[1] = -F.uX[0]; F.uY[1] = -F.uY[0];
    } else if (ulen > 1.2f) {
      F.nu = 1;
      F.uX[0] = usx * 0.5f; F.uY[0] = usy * 0.5f;
    }
  }

  // ---- fast-path decision and its tables
  const float kq = v.q * invDenJ;
  const float xmax = JET_RADIUS_U * kq;
  float offMax = 0.0f;
  for (int j = 0; j < 4; ++j) {
    if (absf(F.offX[j]) > offMax) offMax = absf(F.offX[j]);
    if (absf(F.offY[j]) > offMax) offMax = absf(F.offY[j]);
  }
  for (int j = 0; j < 2; ++j) {
    if (absf(F.uX[j]) > offMax) offMax = absf(F.uX[j]);
    if (absf(F.uY[j]) > offMax) offMax = absf(F.uY[j]);
  }
  for (int j = 0; j < 4; ++j) {
    const float a = absf(F.offX[j]) + absf(F.fillX), b = absf(F.offY[j]) + absf(F.fillY);
    if (a > offMax) offMax = a;
    if (b > offMax) offMax = b;
  }
  // zc = dist * den = zJ * (1 + x) must stay in the linear part of depthQ (< 160 u).
  F.fast = xmax <= X_POLY_MAX && F.denJ * (1.0f - xmax) >= cfg::CULL_DEN && offMax <= ST_OFF_MAX &&
           F.zJ * (1.0f + xmax) < 159.0f;
  if (!F.fast) return true;

  // Back-fill direction per slice: the culled neighbours are on the side away from the eye.
  unsigned dirs = 0;
  for (int k = 0; k < GZ; ++k) {
    F.fillDir[k] = eyeUp > (float)(k - cfg::JET_PIVOT_SLICE) ? 0 : 1;
    if (!F.hidden[k]) dirs |= 1u << F.fillDir[k];
  }
  int ext[4] = {0, 0, 0, 0};   // dx min/max, dy min/max over all stamps
  buildStamps(&s_stS[0], s_stS[0].off, (int)sizeof(s_stS[0]), 6, false, F.offX, F.offY, F.uX,
              F.uY, F.nu, F.fillX, F.fillY, dirs, ext);
  // m >= JET_SPLAT_BIG_M <=> t >= tBig (m = mJ * t); only possible if mJ / (1 - xmax) >= it.
  const bool bigPossible = F.mJ >= cfg::JET_SPLAT_BIG_M * (1.0f - xmax);
  F.tBig = bigPossible ? cfg::JET_SPLAT_BIG_M / F.mJ : 1e30f;
  if (bigPossible)
    buildStamps(&s_stB[0], s_stB[0].off, (int)sizeof(s_stB[0]), 18, true, F.offX, F.offY, F.uX,
                F.uY, F.nu, F.fillX, F.fillY, dirs, ext);
  // Voxels whose whole stamp is inside the frame skip the per-pixel bounds checks.
  const int fxLo = -ext[0], fxN = LW - ext[1] - fxLo;
  const int fyLo = -ext[2], fyN = LH - ext[3] - fyLo;
  F.fx8 = (fxLo + ST_BIAS) * ST_FRAC;
  F.fy8 = (fyLo + ST_BIAS) * ST_FRAC;
  F.fxN8 = (unsigned)(fxN > 0 ? fxN * ST_FRAC : 0);
  F.fyN8 = (unsigned)(fyN > 0 ? fyN * ST_FRAC : 0);
  // Fixed point: xi = (sx + ST_BIAS) * 8 -> pixel = (xi >> 3) - ST_BIAS, bin = xi & 7.
  //   x  = q * dDepth / den_j          (linear in a2, b; per-slice base)
  //   Am = 8 * mJ * across, Ym = 8 * mJ * height (linear)
  //   sx8 = cx8 + Am / (1 + x), sy8 = cy8[k] - Ym / (1 + x), zc = zJ * (1 + x)
  const float K8 = (float)ST_FRAC * F.mJ;
  F.cx8 = (v.Cx + (float)ST_BIAS) * (float)ST_FRAC;
  F.aR = K8 * F.Ra; F.aF = K8 * F.Fa; F.yR = K8 * F.Ry; F.yF = K8 * F.Fy;
  F.xR = kq * F.Rd; F.xF = kq * F.Fd;
  for (int k = 0; k < GZ; ++k) {
    const float sv = (float)(k - cfg::JET_PIVOT_SLICE);
    F.xs[k] = kq * sv * F.Ud;
    F.As[k] = K8 * (F.Aj + sv * F.Ua);
    F.Ys[k] = K8 * (F.Yj + sv * F.Uy);
    F.cy8[k] = (v.Cy + F.drop[k] + (float)ST_BIAS) * (float)ST_FRAC;
  }
  // Body box from the projected corners of each slice's (a2, b) rectangle (perspective maps
  // the rectangle to a convex quad containing every voxel of the slice): conservative, and no
  // per-voxel min / max in the splat loop. Old rule: floor(min) - 1 .. ceil(max) + 2, widened
  // to the stamp extents.
  int xiMin = 0x7fffffff, xiMax = -0x7fffffff, yiMin = 0x7fffffff, yiMax = -0x7fffffff;
  for (int k = 0; k < GZ; ++k) {
    if (F.hidden[k] || m.sliceEnd(k) <= m.sliceBegin(k)) continue;
    const int8_t* se = m.sliceExtent(k);
    for (int corner = 0; corner < 4; ++corner) {
      const float fa2 = (float)se[corner & 1], fbv = (float)se[2 + (corner >> 1)];
      const float t = recip1p(F.xs[k] + fa2 * F.xR + fbv * F.xF);
      const float sx8 = F.cx8 + t * (F.As[k] + fa2 * F.aR + fbv * F.aF);
      const float sy8 = F.cy8[k] - t * (F.Ys[k] + fa2 * F.yR + fbv * F.yF);
      // clamp to the cull window (+-8 px) before converting (keeps the int conversion sane)
      const float lo = (float)((ST_BIAS - 9) * ST_FRAC);
      const int xi = (int)clampf(sx8, lo, (float)((LW + ST_BIAS + 9) * ST_FRAC));
      const int yi = (int)clampf(sy8, lo, (float)((LH + ST_BIAS + 9) * ST_FRAC));
      if (xi < xiMin) xiMin = xi;
      if (xi > xiMax) xiMax = xi;
      if (yi < yiMin) yiMin = yi;
      if (yi > yiMax) yiMax = yi;
    }
  }
  const int dxLo = ext[0] < -1 ? ext[0] : -1, dxHi = ext[1] > 2 ? ext[1] : 2;
  const int dyLo = ext[2] < -1 ? ext[2] : -1, dyHi = ext[3] > 2 ? ext[3] : 2;
  F.box[0] = (xiMin >> 3) - ST_BIAS + dxLo;
  F.box[1] = ((xiMax + ST_FRAC - 1) >> 3) - ST_BIAS + dxHi;
  F.box[2] = (yiMin >> 3) - ST_BIAS + dyLo;
  F.box[3] = ((yiMax + ST_FRAC - 1) >> 3) - ST_BIAS + dyHi;
  return true;
}

// General splat path (jet very close to the camera plane, extreme FOV): exact divide per voxel
// and direct sub-sample writes, as the original renderer. Not reached in normal play.
JET_NOINLINE void splatGeneral(FrameBuf& fb, const View& v, const JetModel& m, const JetFrame& F,
                               uint32_t& nVox, uint32_t& nWrites, NavPt* nav, int& nNav,
                               int box[4]) {
  uint8_t* px = fb.px;
  uint8_t* depth = fb.depth;
  const JetVox* vx = m.voxels();
  const uint16_t* keys = m.renderKeys();
  const float Cx = v.Cx, Cy = v.Cy, q = v.q, dist = v.dist;
  float minX = 1e9f, maxX = -1e9f, minY = 1e9f, maxY = -1e9f;
#define JET_PUT(X, Y)                                                       \
  do {                                                                      \
    int xx_ = (X), yy_ = (Y);                                               \
    if ((unsigned)xx_ < (unsigned)LW && (unsigned)yy_ < (unsigned)LH) {     \
      int q_ = yy_ * LW + xx_;                                              \
      if (dq <= depth[q_]) { px[q_] = col; depth[q_] = dq; }                \
    }                                                                       \
  } while (0)
  for (int oi = 0; oi < GZ; ++oi) {
    const int k = F.order[oi];
    if (F.hidden[k]) continue;
    const float sv = (float)(k - cfg::JET_PIVOT_SLICE);
    const float As = F.Aj + sv * F.Ua, Ds = F.Dj + sv * F.Ud, Ys = F.Yj + sv * F.Uy;
    const float dropK = F.drop[k];
    const unsigned zmk = F.zm[k];
    const int i0 = m.sliceBegin(k), i1 = m.sliceEnd(k);
    for (int ii = 0; ii < i1 - i0; ++ii) {
      const int i = F.noseFirst ? i0 + ii : i1 - 1 - ii;
      const JetVox& vv = vx[i];
      const unsigned key = keys[i];
      if (!(key & (s_xm[(uint8_t)vv.a2] | s_ym[(uint8_t)vv.b] | zmk))) continue;
      const uint8_t col = s_col[(key >> JET_KEY_CIDX_SHIFT) & 511u];
      if (!col) continue;
      const float fa2 = (float)vv.a2, fbv = (float)vv.b;
      const float A = As + fa2 * F.Ra + fbv * F.Fa;
      const float D = Ds + fa2 * F.Rd + fbv * F.Fd;
      const float Y = Ys + fa2 * F.Ry + fbv * F.Fy;
      const float den = 1.0f + q * D;
      if (den < cfg::CULL_DEN) continue;
      const float mm = v.S / den;
      const float sx = Cx + mm * A;
      const float sy = Cy - mm * Y + dropK;
      if (sx < -8.0f || sx > (float)LW + 8.0f || sy < -8.0f || sy > (float)LH + 8.0f) continue;
      const uint8_t dq = depthQ(dist * den);
      ++nVox;
      if (sx < minX) minX = sx;
      if (sx > maxX) maxX = sx;
      if (sy < minY) minY = sy;
      if (sy > maxY) maxY = sy;
      if (mm >= cfg::JET_SPLAT_BIG_M) {
        for (int j = 0; j < 4; ++j) {
          int x = fastFloor(sx + F.offX[j]), y = fastFloor(sy + F.offY[j]);
          JET_PUT(x, y); JET_PUT(x + 1, y); JET_PUT(x, y + 1); JET_PUT(x + 1, y + 1);
        }
        nWrites += 16;
      } else {
        for (int j = 0; j < 4; ++j) JET_PUT(fastRound(sx + F.offX[j]), fastRound(sy + F.offY[j]));
        nWrites += 4;
      }
      for (int j = 0; j < F.nu; ++j) JET_PUT(fastRound(sx + F.uX[j]), fastRound(sy + F.uY[j]));
      nWrites += (uint32_t)F.nu;
      if ((key & JET_KEY_NAV) && nNav < 4) {
        nav[nNav].x = sx; nav[nNav].y = sy; nav[nNav].dq = dq;
        nav[nNav].color = vv.mat == JM_NAV_RED ? (F.navOn ? pal::NAV_RED : (uint8_t)0)
                        : vv.mat == JM_NAV_GREEN ? (F.navOn ? pal::NAV_GREEN : (uint8_t)0)
                        : (F.strobeOn ? pal::STROBE : (uint8_t)0);
        ++nNav;
      }
    }
  }
#undef JET_PUT
  if (nVox) {
    box[0] = fastFloor(minX) - 1;
    box[1] = -fastFloor(-maxX) + 2;
    box[2] = fastFloor(minY) - 1;
    box[3] = -fastFloor(-maxY) + 2;
  }
}

// ---- shadow

// Per-frame constants of the shadow loop. Everything is linear in the voxel coordinates
// (a2, b, s) for a given ground height h; dh = h - hRef is 0 over flat ground.
struct ShadowCtx {
  uint8_t exitMask;
  unsigned nightMask;        // 8: skip Bayer >= 8 (night), 0: none
  bool checkDz;              // a voxel can be below the ground
  float dzR, dzF, dzU, dz0;  // dz = P.z - hRef
  float xR, xF, xU;          // x = q (Gd - Gd0) / denRef (perspective term)
  float aR, aF, aU, a0;      // Am = 8 mRef Ga (+ ST_BIAS / rounding folded into cx8)
  float yR, yF, yU, y0;      // Ym = 8 mRef ((hRef - alt) + T Gd)
  float xH, aH, yH;          // d(x, Am, Ym) / d(dh)
  float cx8, cy8;            // (C + ST_BIAS) * 8 + 4 (round to nearest)
  float zRef, tWide;         // zc = zRef (1 + x); m > 1.2 <=> t > tWide
  float minT1;               // 1 + x below this -> den < CULL_DEN
  // uneven ground: grid cell of a voxel = (int)(g? + ...)
  float gxR, gxF, gxU, gx0, gyR, gyF, gyU, gy0;
};
constexpr int HG_N = 16;     // height grid under the jet (cells), covers 2 * 24.5 u + margin
int8_t s_hgrid[HG_N][HG_N];  // ground height - hRef per cell, u (heights are whole u <= 56)

// Shadow sample loop. FLAT: one ground level under the whole jet (open sea, plateaus):
// polynomial perspective always valid (checked per frame), depth in the linear range.
// Otherwise the ground height comes from s_hgrid per sample and samples far from the
// reference fall back to the exact reciprocal.
template <bool FLAT>
inline int shadowLoop(const ShadowCtx& C, FrameBuf& fb, const JetModel& m, const Palette& pal) {
  constexpr int LO8 = (ST_BIAS - 4) * ST_FRAC, XS8 = (LW + 8) * ST_FRAC, YS8 = (LH + 8) * ST_FRAC;
  const JetVox* vxs = m.voxels();
  const uint16_t* keys = m.renderKeys();
  uint8_t* px = fb.px;
  const uint8_t* depth = fb.depth;
  const unsigned exitMask = C.exitMask, nightMask = C.nightMask;
  int samples = 0;
  for (int kk = 0; kk < GZ; ++kk) {
    const float sv = (float)(kk - cfg::JET_PIVOT_SLICE);
    const float xs = sv * C.xU, As = C.a0 + sv * C.aU, Ys = C.y0 + sv * C.yU;
    const float dzs = C.dz0 + sv * C.dzU;
    const float gxs = C.gx0 + sv * C.gxU, gys = C.gy0 + sv * C.gyU;
    const int i1 = m.sliceEnd(kk);
    for (int i = m.sliceBegin(kk); i < i1; ++i) {
      if (!(keys[i] & exitMask)) continue;
      const JetVox& vv = vxs[i];
      const float fa = (float)vv.a2, fbv = (float)vv.b;
      ++samples;
      float x = xs + fa * C.xR + fbv * C.xF;
      float Am = As + fa * C.aR + fbv * C.aF;
      float Ym = Ys + fa * C.yR + fbv * C.yF;
      float dz = dzs + fa * C.dzR + fbv * C.dzF;
      if constexpr (!FLAT) {
        int ix = (int)(gxs + fa * C.gxR + fbv * C.gxF), iy = (int)(gys + fa * C.gyR + fbv * C.gyF);
        ix = ix < 0 ? 0 : (ix > HG_N - 1 ? HG_N - 1 : ix);
        iy = iy < 0 ? 0 : (iy > HG_N - 1 ? HG_N - 1 : iy);
        const float dh = (float)s_hgrid[iy][ix];
        dz -= dh;
        x += dh * C.xH;
        Am += dh * C.aH;
        Ym += dh * C.yH;
      }
      if (C.checkDz && dz < 0.0f) continue;
      float t;
      int dq;
      if constexpr (FLAT) {
        t = recip1p(x);
        dq = (int)(C.zRef + C.zRef * x);
      } else {
        if (x < X_POLY_MAX && x > -X_POLY_MAX) {
          t = recip1p(x);
        } else {
          if (1.0f + x < C.minT1) continue;   // behind the camera plane
          t = 1.0f / (1.0f + x);
        }
        dq = (int)depthQ(C.zRef + C.zRef * x);
      }
      const int xi = (int)(C.cx8 + t * Am), yi = (int)(C.cy8 - t * Ym);
      if ((unsigned)(xi - LO8) >= (unsigned)XS8 || (unsigned)(yi - LO8) >= (unsigned)YS8) continue;
      const int xp = (xi >> 3) - ST_BIAS, yp = (yi >> 3) - ST_BIAS;
      if ((unsigned)yp >= (unsigned)LH) continue;
      const int wpx = t > C.tWide ? 2 : 1;
      const uint8_t* brow = s_bayer + ((yp & 3) << 2);
      for (int j = 0; j < wpx; ++j) {
        const int xx = xp + j;
        if ((unsigned)xx >= (unsigned)LW) continue;
        if (brow[xx & 3] & nightMask) continue;
        const int qq = yp * LW + xx;
        if ((unsigned)((int)depth[qq] - dq + 2) > 4u) continue;   // |depth - dq| <= 2
        px[qq] = pal.shade(px[qq]);
      }
    }
  }
  return samples;
}

// Uneven ground (coast, terraces): out of IRAM, the loop body stays in the cache.
JET_NOINLINE int shadowUneven(const ShadowCtx& C, FrameBuf& fb, const JetModel& m,
                              const Palette& pal) {
  return shadowLoop<false>(C, fb, m, pal);
}

}  // namespace

JetAxes jetAxes(const JetPose& p) {
  float sp = sinf(p.yaw), cp = cosf(p.yaw);
  float sr = sinf(p.roll), cr = cosf(p.roll);
  float st = sinf(p.pitch), ct = cosf(p.pitch);
  Vec3 f0 = v3(sp, -cp, 0.0f);
  Vec3 up0 = v3(cp * sr, sp * sr, cr);
  JetAxes a;
  a.right = v3(cp * cr, sp * cr, -sr);
  a.fwd = f0 * ct + up0 * st;
  a.up = up0 * ct - f0 * st;
  return a;
}

JetAnchors jetAnchors(const JetPose& p) { return jetAnchors(p, jetAxes(p)); }

JetAnchors jetAnchors(const JetPose& p, const JetAxes& ax) {
  JetAnchors a;
  a.tipL = wrapPos3(p.pos - ax.right * TIP_A + ax.fwd * TIP_B);
  a.tipR = wrapPos3(p.pos + ax.right * TIP_A + ax.fwd * TIP_B);
  Vec3 noz = p.pos + ax.fwd * NOZZLE_B + ax.up * NOZZLE_S;
  a.nozzle = wrapPos3(noz);
  a.flameLen = flameLength(p);
  a.flameTip = wrapPos3(flameCentre(ax, p, noz, a.flameLen, a.flameLen));
  return a;
}

// Shadow (DESIGN 6.4). The ground point of a voxel is G = P.xy + (k, k) * (P.z - h), i.e. a
// parallel projection along v = (k, k, -1). Every ray along v through the (closed) voxel body
// leaves it through an exposed face whose direction has a positive dot with v, so splatting
// only the voxels with such an "exit" face covers the whole footprint (about half the shell;
// previously every 2nd shell voxel, which left the coverage to chance).
// For a ground height h, G, its depth and its height are linear in the voxel coordinates:
// a sample is ~12 FP ops plus the perspective polynomial (no divide, no cell lookup). Over
// uneven ground the heights of the cells under the jet are fetched once into a 16 x 16 grid.
JET_HOT void renderJetShadow(FrameBuf& fb, const View& v, const JetModel& m, const JetPose& p,
                             const World& w, const Palette& pal, Tod tod, RenderStats* st) {
  if (!p.visible || m.count() == 0) return;
  ensureTables();
  // Wrap the pivot ONCE. Per-voxel points stay unwrapped (a few tens of u outside [0, WORLD_U)
  // at most): World::cell() wraps cell indices itself.
  const float bx = wrapPos(p.pos.x), by = wrapPos(p.pos.y);
  const float g0 = w.groundHeight(bx, by);
  if (p.pos.z - g0 > cfg::JET_SHADOW_MAX_H) return;
  const int ti = (int)tod;
  const float k = cfg::SHADOW_K[ti < 0 ? 0 : (ti > 2 ? 2 : ti)];
  const JetAxes ax = jetAxes(p);
  ShadowCtx C;

  // Exit faces for the projection direction v = (k, k, -1), in model axes.
  const Vec3 pv = v3(k, k, -1.0f);
  const float vxm = dot(pv, ax.right), vym = dot(pv, ax.fwd), vzm = dot(pv, ax.up);
  constexpr float EPS = 1e-3f;
  C.exitMask = (uint8_t)((vxm > EPS ? JF_PX : 0) | (vxm < -EPS ? JF_NX : 0) |
                         (vym > EPS ? JF_PY : 0) | (vym < -EPS ? JF_NY : 0) |
                         (vzm > EPS ? JF_PZ : 0) | (vzm < -EPS ? JF_NZ : 0));
  C.nightMask = tod == Tod::Night ? 8u : 0u;

  // Terrain under the jet's whole xy extent (|offset . x| <= 24 u): fetch the cell heights
  // (relative to the pivot's ground) and see whether it is one flat level.
  constexpr float INV_CELL = 1.0f / cfg::CELL_U;
  constexpr int CB = 1 << 20;   // bias: truncation == floor
  const float hRef = g0;   // whole u (cfg::HEIGHT_U)
  const int hRefI = (int)g0;
  const int cx0 = (int)((bx - 25.0f) * INV_CELL + (float)CB) - CB;
  const int cy0 = (int)((by - 25.0f) * INV_CELL + (float)CB) - CB;
  const float ex = 12.5f * absf(ax.right.x) + 19.0f * absf(ax.fwd.x) + 6.0f * absf(ax.up.x) + 1.0f;
  const float ey = 12.5f * absf(ax.right.y) + 19.0f * absf(ax.fwd.y) + 6.0f * absf(ax.up.y) + 1.0f;
  const int ix0 = (int)((bx - ex) * INV_CELL + (float)CB) - CB - cx0;
  const int ix1 = (int)((bx + ex) * INV_CELL + (float)CB) - CB - cx0;
  const int iy0 = (int)((by - ey) * INV_CELL + (float)CB) - CB - cy0;
  const int iy1 = (int)((by + ey) * INV_CELL + (float)CB) - CB - cy0;
  bool flat = true;
  for (int iy = iy0; iy <= iy1; ++iy)
    for (int ix = ix0; ix <= ix1; ++ix) {
      const int dh = (int)s_heightU[cellLevel(w.cell(cx0 + ix, cy0 + iy))] - hRefI;
      if (dh != 0) flat = false;
      if ((unsigned)ix < (unsigned)HG_N && (unsigned)iy < (unsigned)HG_N) s_hgrid[iy][ix] = (int8_t)dh;
    }

  const float c = v.cpsi, s = v.spsi;
  const float pdx = wrapd(bx - v.fx), pdy = wrapd(by - v.fy);
  const float Pa0 = pdx * c + pdy * s, Pd0 = pdx * s - pdy * c;
  const float kA = k * (c + s), kD = k * (s - c);   // ground offset per u of height, view space
  C.dz0 = p.pos.z - hRef;
  // Ground point of the pivot: reference of the perspective polynomial.
  const float Gd0 = Pd0 + kD * C.dz0;
  const float denRef = 1.0f + v.q * Gd0;
  if (denRef < 0.3f) return;   // shadow centre (nearly) behind the camera plane
  // |Gd - Gd0| <= (1 + |kD|) * 24 over the model (flat ground).
  const float xmax = (1.0f + absf(kD)) * JET_RADIUS_U * v.q / denRef;
  C.zRef = v.dist * denRef;
  flat = flat && xmax <= X_POLY_MAX && denRef * (1.0f - xmax) >= cfg::CULL_DEN &&
         C.zRef * (1.0f + xmax) < 159.0f;

  const float invRef = 1.0f / denRef;
  const float mRef = v.S * invRef;
  const float kq = v.q * invRef;
  const float K8 = (float)ST_FRAC * mRef;
  const Vec3 R = ax.right * 0.5f;
  C.dzR = R.z; C.dzF = ax.fwd.z; C.dzU = ax.up.z;
  // view-space derivatives of (Ga, Gd) per a2 / b / s (dz included)
  const float gaR = (R.x * c + R.y * s) + kA * C.dzR, gdR = (R.x * s - R.y * c) + kD * C.dzR;
  const float gaF = (ax.fwd.x * c + ax.fwd.y * s) + kA * C.dzF, gdF = (ax.fwd.x * s - ax.fwd.y * c) + kD * C.dzF;
  const float gaU = (ax.up.x * c + ax.up.y * s) + kA * C.dzU, gdU = (ax.up.x * s - ax.up.y * c) + kD * C.dzU;
  C.xR = kq * gdR; C.xF = kq * gdF; C.xU = kq * gdU;
  C.aR = K8 * gaR; C.aF = K8 * gaF; C.aU = K8 * gaU; C.a0 = K8 * (Pa0 + kA * C.dz0);
  C.yR = K8 * v.T * gdR; C.yF = K8 * v.T * gdF; C.yU = K8 * v.T * gdU;
  C.y0 = K8 * ((hRef - v.alt) + v.T * Gd0);
  // ground dh higher: G moves by -(k, k) dh, its height by +dh
  C.xH = -kq * kD; C.aH = -K8 * kA; C.yH = K8 * (1.0f - v.T * kD);
  C.cx8 = (v.Cx + (float)ST_BIAS) * (float)ST_FRAC + 4.0f;
  C.cy8 = (v.Cy + (float)ST_BIAS) * (float)ST_FRAC + 4.0f;
  C.tWide = 1.2f / mRef;
  C.minT1 = cfg::CULL_DEN * invRef;
  // dz < 0 (voxel below the ground) can only happen when the jet is within 24 u of it.
  C.checkDz = !flat || C.dz0 < JET_RADIUS_U;
  // grid cell of a voxel: (world x - cell x0 * CELL_U) / CELL_U
  C.gxR = R.x * INV_CELL; C.gxF = ax.fwd.x * INV_CELL; C.gxU = ax.up.x * INV_CELL;
  C.gyR = R.y * INV_CELL; C.gyF = ax.fwd.y * INV_CELL; C.gyU = ax.up.y * INV_CELL;
  C.gx0 = (bx - (float)cx0 * cfg::CELL_U) * INV_CELL;
  C.gy0 = (by - (float)cy0 * cfg::CELL_U) * INV_CELL;

  int samples;
  if (flat) samples = shadowLoop<true>(C, fb, m, pal);
  else samples = shadowUneven(C, fb, m, pal);
  // Cost model (sim): a shadow sample (~80 instructions) ~ 1/2 of a drawn body voxel.
  if (st != nullptr) st->voxels += (uint32_t)(samples / 2);
}

// Afterburner (DESIGN 6.3). Kept out of IRAM (~12 samples at cruise, ~40 under boost).
// Grows the box [bx0,bx1]x[by0,by1] (inclusive) over what it touched.
static JET_NOINLINE void drawFlame(FrameBuf& fb, const View& v, const JetPose& p, const JetAxes& ax,
                                   RenderStats& st, int& bx0, int& by0, int& bx1, int& by1,
                                   HaloPt* halo, int& nHalo) {
  nHalo = 0;
  const float boost = sat(p.boost);
  const float len = flameLength(p);
  if (len < 0.5f) return;
  const Vec3 noz = p.pos + ax.fwd * NOZZLE_B + ax.up * NOZZLE_S;
  const float rBase = cfg::FLAME_R * (1.0f + 0.25f * boost);
  const bool shock = boost > 0.6f;
  const float shockK = sat((boost - 0.6f) * 2.5f);
  const float invLen = 1.0f / len;
  // centre(t) = noz - fwd * t + curl * t^2   (== flameCentre)
  const Vec3 curl = ax.right * (cfg::FLAME_CURL * p.yawRateN * invLen) +
                    ax.up * (CLIMB_CURL * p.climbRateN * invLen);
  // sin(1.9 t - 40 time) stepped by rotation: t advances 0.5 -> angle advances 0.95 rad.
  constexpr float SD = 0.81341550f, CD = 0.58168309f;   // sin(0.95), cos(0.95)
  float sn = -sinf(40.0f * p.time), cs = cosf(40.0f * p.time);
  NearProj np(v);
  uint8_t* px = fb.px;
  uint8_t* depth = fb.depth;
  uint32_t touched = 0;
  uint8_t colOf[256];
  for (int it = 0;; ++it) {
    const float t = 0.5f * (float)it;
    if (t > len) break;
    const float wob = sn;
    {
      const float ns = sn * CD + cs * SD;
      cs = cs * CD - sn * SD;
      sn = ns;
    }
    const float u = t * invLen;
    const float r = rBase * pow075(1.0f - u) * (0.9f + 0.2f * wob);
    if (r < 0.15f) continue;
    ScreenPt sp;
    const Vec3 cpos = noz - ax.fwd * t + curl * (t * t);
    if (!np.project(cpos, sp, cfg::JET_CULL_DEN)) continue;
    if (sp.x < -16.0f || sp.x > (float)LW + 16.0f || sp.y < -16.0f || sp.y > (float)LH + 16.0f)
      continue;
    const float rs = r * sp.m;
    const int cx = fastRound(sp.x), cy = fastRound(sp.y);
    float base = 0.9f * u - 0.16f * boost;
    if (shock)  // cool the core between the shock diamonds so they read as bright knots
      base += shockK * 0.24f * (bump(u, 0.40f, 1.0f / 0.09f) + bump(u, 0.64f, 1.0f / 0.09f));
    // Per-threshold squared radii: pixel heat index = number of thresholds with d2 >= thr.
    float thr[5];
    const float invK = rs * (1.0f / 0.55f);
    for (int i = 0; i < 5; ++i) {
      const float lim = (s_heatT[i] - base) * invK;
      thr[i] = lim <= 0.0f ? -1.0f : lim * lim;
    }
    if (halo && u < 0.4f && nHalo < 32) {
      halo[nHalo].x = (int16_t)cx; halo[nHalo].y = (int16_t)cy;
      halo[nHalo].rs = rs; halo[nHalo].dq = sp.dq; ++nHalo;
    }
    const uint8_t dq = sp.dq;
    if (rs < 0.5f) {
      if (!inFrame(cx, cy)) continue;
      int hi = 0;
      while (hi < 5 && 0.0f >= thr[hi]) ++hi;
      const uint8_t col = (uint8_t)(pal::FLAME0 + hi);
      const int q = cy * LW + cx;
      const uint8_t cur = px[q];
      ++touched;
      if (pal::isFlame(cur)) {
        if (col < cur) { px[q] = col; if (dq < depth[q]) depth[q] = dq; }
      } else if (dq <= depth[q]) {
        px[q] = col; depth[q] = dq;
      }
      if (cx < bx0) bx0 = cx;
      if (cx > bx1) bx1 = cx;
      if (cy < by0) by0 = cy;
      if (cy > by1) by1 = cy;
      continue;
    }
    const int R = ceilPos(rs);
    const float lim = rs * rs + 0.5f * rs;
    const int limI = (int)lim;   // d2 (integer) <= lim  <=>  d2 <= floor(lim)
    const int y0 = cy - R < 0 ? 0 : cy - R, y1 = cy + R > LH - 1 ? LH - 1 : cy + R;
    const int x0 = cx - R < 0 ? 0 : cx - R, x1 = cx + R > LW - 1 ? LW - 1 : cx + R;
    if (x0 > x1 || y0 > y1) continue;
    // Heat colour by integer d2 (thresholds are non-decreasing, so the index is monotone).
    const bool useTable = limI < 256;
    if (useTable) {
      int hi = 0;
      for (int d2 = 0; d2 <= limI; ++d2) {
        while (hi < 5 && (float)d2 >= thr[hi]) ++hi;
        colOf[d2] = (uint8_t)(pal::FLAME0 + hi);
      }
    }
    for (int y = y0; y <= y1; ++y) {
      const int dy = y - cy;
      const int dy2 = dy * dy;
      if (dy2 > limI) continue;
      uint8_t* prow = px + y * LW;
      uint8_t* drow = depth + y * LW;
      for (int x = x0; x <= x1; ++x) {
        const int dx = x - cx;
        const int d2 = dx * dx + dy2;
        if (d2 > limI) continue;
        uint8_t col;
        if (useTable) {
          col = colOf[d2];
        } else {
          int hi = 0;
          while (hi < 5 && (float)d2 >= thr[hi]) ++hi;
          col = (uint8_t)(pal::FLAME0 + hi);
        }
        const uint8_t cur = prow[x];
        ++touched;
        if ((uint8_t)(cur - pal::FLAME0) < 6) {   // max-heat rule: only a hotter index replaces
          if (col < cur) { prow[x] = col; if (dq < drow[x]) drow[x] = dq; }
        } else if (dq <= drow[x]) {
          prow[x] = col; drow[x] = dq;
        }
      }
    }
    if (x0 < bx0) bx0 = x0;
    if (x1 > bx1) bx1 = x1;
    if (y0 < by0) by0 = y0;
    if (y1 > by1) by1 = y1;
  }
  // Shock diamonds: 2 px of white-hot FLAME0 on the axis.
  if (shock) {
    ScreenPt a = {}, b = {};
    bool haveAxis = project(v, noz, a, cfg::JET_CULL_DEN) &&
                    project(v, flameCentre(ax, p, noz, len, len), b, cfg::JET_CULL_DEN);
    float axx = haveAxis ? b.x - a.x : 1.0f, axy = haveAxis ? b.y - a.y : 0.0f;
    // second pixel goes across the plume's screen direction
    int ox = 0, oy = 0;
    if ((axy < 0.0f ? -axy : axy) >= (axx < 0.0f ? -axx : axx)) ox = 1; else oy = 1;
    for (int i = 0; i < 2; ++i) {
      float t = cfg::SHOCK_T[i] * len;
      ScreenPt sp;
      if (!project(v, flameCentre(ax, p, noz, t, len), sp, cfg::JET_CULL_DEN)) continue;
      int cx = fastRound(sp.x), cy = fastRound(sp.y);
      for (int j = 0; j < 2; ++j) {
        int x = cx + j * ox, y = cy + j * oy;
        if (!inFrame(x, y)) continue;
        int q = y * LW + x;
        ++touched;
        if (pal::isFlame(px[q]) || sp.dq <= depth[q]) {
          px[q] = pal::FLAME0;
          if (sp.dq < depth[q]) depth[q] = sp.dq;
        }
      }
    }
  } else {
    nHalo = 0;   // halo only under boost
  }
  st.flamePx += touched;
}

// Body splat + flame + outline + halo + nav lights (DESIGN 6.2). The per-voxel and per-pixel
// loops are here (IRAM); the per-frame setup is jetFrameSetup() (flash).
JET_HOT void renderJet(FrameBuf& fb, const View& v, const JetModel& m, const JetPose& p, Tod tod,
                       RenderStats& st, Rect* bounds) {
  if (bounds) { bounds->x0 = 0; bounds->y0 = 0; bounds->x1 = 0; bounds->y1 = 0; }
  if (!p.visible || m.count() == 0) return;
  JetFrame& F = s_fr;
  if (!jetFrameSetup(v, m, p, F)) return;

  uint8_t* px = fb.px;
  uint8_t* depth = fb.depth;
  uint32_t nVox = 0, nWrites = 0;
  NavPt nav[4];
  int nNav = 0;
  int box[4] = {0, 0, 0, 0};   // body box x0, x1, y0, y1 (+1 for 2x2 splats)

  if (F.fast) {
    const JetVox* vx = m.voxels();
    const uint16_t* keys = m.renderKeys();
    const uint8_t* colT = s_col;
    const uint8_t* xm = s_xm;
    const uint8_t* ym = s_ym;
    const float xR = F.xR, xF = F.xF, aR = F.aR, aF = F.aF, yR = F.yR, yF = F.yF, cx8 = F.cx8;
    const float zJ = F.zJ, tBig = F.tBig;
    const int fx8 = F.fx8, fy8 = F.fy8;
    const unsigned fxN8 = F.fxN8, fyN8 = F.fyN8;
    // Cull window of the old renderer: sx in [-8, LW + 8], sy in [-8, LH + 8].
    constexpr int LO8 = (ST_BIAS - 8) * ST_FRAC;
    constexpr unsigned XSPAN8 = (unsigned)((LW + 16) * ST_FRAC);
    constexpr unsigned YSPAN8 = (unsigned)((LH + 16) * ST_FRAC);
    uint32_t acc = 0;   // drawn voxels << 16 | pixel writes (one add per voxel)
    const int step = F.noseFirst ? 1 : -1;

    for (int oi = 0; oi < GZ; ++oi) {
      const int k = F.order[oi];
      if (F.hidden[k]) continue;
      const float xs = F.xs[k], As = F.As[k], Ys = F.Ys[k], cy8 = F.cy8[k];
      const unsigned zmk = F.zm[k];
      // back fill toward the culled side: neighbour below (eye above) / above, if solid
      const int fd = F.fillDir[k];
      const unsigned fillMask = fd ? (unsigned)JF_PZ : (unsigned)JF_NZ;
      const int i0 = m.sliceBegin(k), i1 = m.sliceEnd(k);
      int i = F.noseFirst ? i0 : i1 - 1;
      for (int cnt = i1 - i0; cnt > 0; --cnt, i += step) {
        const unsigned key = keys[i];
        const JetVox& vv = vx[i];
        // All exposed faces turned away -> hidden (nav lights carry all face bits).
        if (!(key & (unsigned)(xm[(uint8_t)vv.a2] | ym[(uint8_t)vv.b] | zmk))) continue;
        const uint8_t col = colT[(key >> JET_KEY_CIDX_SHIFT) & 511u];
        if (!col) continue;
        const float fa2 = (float)vv.a2, fbv = (float)vv.b;
        const float x = xs + fa2 * xR + fbv * xF;            // q * dDepth / den_j
        const float Am = As + fa2 * aR + fbv * aF;           // 8 * mJ * across
        const float Ym = Ys + fa2 * yR + fbv * yF;           // 8 * mJ * height
        const float t = recip1p(x);                          // m / mJ
        const float sx8 = cx8 + t * Am;
        const float sy8 = cy8 - t * Ym;
        const int xi = (int)sx8, yi = (int)sy8;
        const int cse = ((yi & (ST_FRAC - 1)) * ST_FRAC) | (xi & (ST_FRAC - 1));
        const StampHead* sh;
        const int16_t* offs;
        if (t < tBig) {
          sh = &s_stS[cse]; offs = s_stS[cse].off;
        } else {
          sh = &s_stB[cse]; offs = s_stB[cse].off;
        }
        const int n = sh->n;
        const int nf = (key & fillMask) ? 0 : sh->nf[fd];   // face exposed: no neighbour there
        const int16_t* foff = sh->fill[fd];
        const uint8_t dq = (uint8_t)(int)(zJ + zJ * x);
        const int pxl = (xi >> 3) - ST_BIAS, pyl = (yi >> 3) - ST_BIAS;
        if ((unsigned)(xi - fx8) < fxN8 && (unsigned)(yi - fy8) < fyN8) {
          // whole stamp inside the frame
          const int q0 = pyl * LW + pxl;
          uint8_t* pp = px + q0;
          uint8_t* dp = depth + q0;
          for (int j = 0; j < n; ++j) {
            const int o = offs[j];
            if (dq <= dp[o]) { pp[o] = col; dp[o] = dq; }
          }
          const uint8_t dqf = (uint8_t)(dq + FILL_DQ_BIAS);
          for (int j = 0; j < nf; ++j) {
            const int o = foff[j];
            if (dqf <= dp[o]) { pp[o] = col; dp[o] = dqf; }
          }
        } else {
          if ((unsigned)(xi - LO8) > XSPAN8 || (unsigned)(yi - LO8) > YSPAN8) continue;
          for (int j = 0; j < n + nf; ++j) {
            const int o = (j < n ? offs[j] : foff[j - n]) + ST_OFF0;   // (dy + 3) * LW + dx + 3
            const int dyp = o / LW;
            const int xx = pxl + (o - dyp * LW) - 3, yy = pyl + dyp - 3;
            if ((unsigned)xx < (unsigned)LW && (unsigned)yy < (unsigned)LH) {
              const int qq = yy * LW + xx;
              const uint8_t d = (uint8_t)(j < n ? dq : dq + FILL_DQ_BIAS);
              if (d <= depth[qq]) { px[qq] = col; depth[qq] = d; }
            }
          }
        }
        acc += 0x10000u + (uint32_t)(n + nf);   // voxels << 16 | pixel writes
        if (JET_UNLIKELY(key & JET_KEY_NAV) && nNav < 4) {
          nav[nNav].x = sx8 * (1.0f / ST_FRAC) - (float)ST_BIAS;
          nav[nNav].y = sy8 * (1.0f / ST_FRAC) - (float)ST_BIAS;
          nav[nNav].dq = dq;
          nav[nNav].color = vv.mat == JM_NAV_RED ? (F.navOn ? pal::NAV_RED : (uint8_t)0)
                          : vv.mat == JM_NAV_GREEN ? (F.navOn ? pal::NAV_GREEN : (uint8_t)0)
                          : (F.strobeOn ? pal::STROBE : (uint8_t)0);
          ++nNav;
        }
      }
    }
    nVox = acc >> 16;
    nWrites = acc & 0xffffu;
    for (int j = 0; j < 4; ++j) box[j] = F.box[j];
  } else {
    splatGeneral(fb, v, m, F, nVox, nWrites, nav, nNav, box);
  }
  st.voxels += nVox;
  st.voxWrites += nWrites;
  if (!nVox) return;

  // Outline ring around the body box.
  const int ox0 = box[0] - 1 < 0 ? 0 : box[0] - 1, ox1 = box[1] + 1 > LW - 1 ? LW - 1 : box[1] + 1;
  const int oy0 = box[2] - 1 < 0 ? 0 : box[2] - 1, oy1 = box[3] + 1 > LH - 1 ? LH - 1 : box[3] + 1;

  // Afterburner (only once fully assembled).
  int fx0 = LW, fy0 = LH, fx1 = -1, fy1 = -1;
  HaloPt halo[32];
  int nHalo = 0;
  if (F.asmb >= 1.0f) drawFlame(fb, v, p, F.ax, st, fx0, fy0, fx1, fy1, halo, nHalo);

  // Outline: background pixels 4-adjacent to body pixels, unless hidden by nearer world.
  // Interior pixels (not on the frame border) need no neighbour bounds checks.
  const int dqLimit = (int)F.dqPivot - 24;
  const uint8_t* cls = s_cls;
  const int ix0 = ox0 < 1 ? 1 : ox0, ix1 = ox1 > LW - 2 ? LW - 2 : ox1;
  for (int y = oy0; y <= oy1; ++y) {
    uint8_t* row = px + y * LW;
    const uint8_t* drow = depth + y * LW;
    const bool up = y > 0, dn = y < LH - 1;
    if (up && dn) {
      for (int x = ix0; x <= ix1; ++x) {
        if (cls[row[x]] & CLS_SKIP) continue;
        const unsigned nb = cls[row[x - 1]] | cls[row[x + 1]] | cls[row[x - LW]] | cls[row[x + LW]];
        if ((nb & CLS_BODY) && (int)drow[x] >= dqLimit) row[x] = pal::JET_OUTLINE;
      }
    }
    // frame-border pixels of the box (checked neighbours)
    for (int x = ox0; x <= ox1; ++x) {
      if (up && dn && x >= ix0 && x <= ix1) { x = ix1; continue; }
      if (cls[row[x]] & CLS_SKIP) continue;
      unsigned nb = 0;
      if (x > 0) nb |= cls[row[x - 1]];
      if (x < LW - 1) nb |= cls[row[x + 1]];
      if (up) nb |= cls[row[x - LW]];
      if (dn) nb |= cls[row[x + LW]];
      if ((nb & CLS_BODY) && (int)drow[x] >= dqLimit) row[x] = pal::JET_OUTLINE;
    }
  }

  // Boost glow halo around the first 40 % of the plume (25 % Bayer FLAME2, no depth write).
  // bayer(x, y) < 4 holds exactly for even x and even y, so only those pixels are visited.
  for (int h = 0; h < nHalo; ++h) {
    const HaloPt& hp = halo[h];
    const float r0 = hp.rs, r1 = hp.rs + 2.0f;
    const int R = ceilPos(r1);
    const int r0s = (int)(r0 * r0 + 0.5f * r0), r1s = (int)(r1 * r1);   // floor (>= 0)
    int ya = hp.y - R, yb = hp.y + R, xa = hp.x - R, xb = hp.x + R;
    if (ya < 0) ya = 0;
    if (yb > LH - 1) yb = LH - 1;
    if (xa < 0) xa = 0;
    if (xb > LW - 1) xb = LW - 1;
    ya += ya & 1;
    xa += xa & 1;
    for (int y = ya; y <= yb; y += 2) {
      const int dy = y - hp.y;
      uint8_t* row = px + y * LW;
      const uint8_t* drow = depth + y * LW;
      for (int x = xa; x <= xb; x += 2) {
        const int dx = x - hp.x;
        const int d2 = dx * dx + dy * dy;
        if (d2 <= r0s || d2 > r1s) continue;
        const uint8_t cc = row[x];
        if (cc >= pal::JET_FIRST && cc < pal::FLAME0 + 6) continue;   // jet, outline, flame
        if (hp.dq <= drow[x]) row[x] = (uint8_t)(pal::FLAME0 + 2);
      }
    }
    if (hp.x - R < fx0) fx0 = hp.x - R;
    if (hp.x + R > fx1) fx1 = hp.x + R;
    if (hp.y - R < fy0) fy0 = hp.y - R;
    if (hp.y + R > fy1) fy1 = hp.y + R;
  }

  // Nav lights: 1 px emissive, plus a soft glow at night.
  const bool night = tod == Tod::Night;
  for (int i = 0; i < nNav; ++i) {
    if (!nav[i].color) continue;
    int x = fastRound(nav[i].x), y = fastRound(nav[i].y);
    if (!inFrame(x, y)) continue;
    int qq = y * LW + x;
    if ((int)nav[i].dq > (int)depth[qq] + 1) continue;   // hidden behind nearer geometry
    px[qq] = nav[i].color;
    if (night) {
      float r1 = nav[i].color == pal::STROBE ? 2.5f : 2.0f;
      drawGlow(fb, x, y, 1.0f, r1, nav[i].dq, nav[i].color, 4);
    }
  }

  if (bounds) {
    int x0 = ox0, y0 = oy0, x1 = ox1, y1 = oy1;
    if (fx1 >= fx0) {
      if (fx0 < x0) x0 = fx0;
      if (fx1 > x1) x1 = fx1;
      if (fy0 < y0) y0 = fy0;
      if (fy1 > y1) y1 = fy1;
    }
    x0 = clampi(x0, 0, LW); x1 = clampi(x1 + 1, 0, LW);
    y0 = clampi(y0, 0, LH); y1 = clampi(y1 + 1, 0, LH);
    bounds->x0 = (int16_t)x0; bounds->y0 = (int16_t)y0;
    bounds->x1 = (int16_t)x1; bounds->y1 = (int16_t)y1;
  }
}

}  // namespace kj
