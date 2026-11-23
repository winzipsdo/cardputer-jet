// jet_model.cpp - the ORIGINAL "Kestrel" voxel jet, sculpted procedurally at boot.
// OWNER: group C. Design: DESIGN.md section 6.1. No reference model data is used: every
// voxel comes from the analytic primitives below (fuselage ellipse sections, intake boxes,
// canopy ellipsoid, cranked-arrow wing, canards, single fin, spine chevron, nozzle ring).
#include "jet_model.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

namespace kj {

// 26 quantised outward directions, model axes (x = right, y = forward/nose, z = up).
const int8_t JET_NORMAL_DIR[JET_NORMAL_COUNT][3] = {
    {-1, -1, -1}, {-1, -1, 0}, {-1, -1, 1}, {-1, 0, -1}, {-1, 0, 0}, {-1, 0, 1},
    {-1, 1, -1},  {-1, 1, 0},  {-1, 1, 1},  {0, -1, -1}, {0, -1, 0}, {0, -1, 1},
    {0, 0, -1},   {0, 0, 1},   {0, 1, -1},  {0, 1, 0},   {0, 1, 1},  {1, -1, -1},
    {1, -1, 0},   {1, -1, 1},  {1, 0, -1},  {1, 0, 0},   {1, 0, 1},  {1, 1, -1},
    {1, 1, 0},    {1, 1, 1}};

namespace {

constexpr int GX = cfg::JET_GRID_X;   // 26 span
constexpr int GY = cfg::JET_GRID_Y;   // 34 rows (0 = nose)
constexpr int GZ = cfg::JET_GRID_Z;   // 11 slices (0 = belly)
constexpr int GN = GX * GY * GZ;      // 9,724 B temporary grid

// Grid cell code: 0 = empty, else 1 + mat * 8 + tone.
inline uint8_t code(int mat, int tone) { return (uint8_t)(1 + mat * 8 + tone); }
inline int codeMat(uint8_t c) { return (c - 1) >> 3; }
inline int codeTone(uint8_t c) { return (c - 1) & 7; }
inline int gidx(int x, int r, int z) { return (z * GY + r) * GX + x; }

inline uint8_t at(const uint8_t* g, int x, int r, int z) {
  if (x < 0 || x >= GX || r < 0 || r >= GY || z < 0 || z >= GZ) return 0;
  return g[gidx(x, r, z)];
}

inline float xcOf(int x) { return (float)x - 12.5f; }
inline float absf(float v) { return v < 0.0f ? -v : v; }

// Fuselage half-width w(row), piecewise linear through the design breakpoints.
float fuselageW(float row) {
  static const float R[8] = {1.0f, 3.0f, 7.0f, 12.0f, 16.0f, 26.0f, 30.0f, 33.0f};
  static const float W[8] = {0.6f, 1.5f, 2.3f, 2.8f, 3.4f, 3.4f, 2.7f, 2.3f};
  if (row <= R[0]) return W[0];
  for (int i = 1; i < 8; ++i) {
    if (row <= R[i]) {
      float t = (row - R[i - 1]) / (R[i] - R[i - 1]);
      return W[i - 1] + (W[i] - W[i - 1]) * t;
    }
  }
  return W[7];
}

// Main wing leading / trailing edge rows as a function of |xc| (cranked arrow).
float wingLE(float ax) {
  if (ax <= 7.0f) return 14.0f + (ax - 3.0f) * (7.0f / 4.0f);  // inner, steep
  return 21.0f + (ax - 7.0f) * (4.0f / 5.5f);                    // outer
}
float wingTE(float ax) {
  if (ax <= 9.0f) return 30.0f;
  return 30.0f - (ax - 9.0f) * (2.0f / 3.5f);                    // forward to row 28 at the tip
}
// A cell (row r) overlaps the span [lo, hi] of rows.
inline bool rowIn(int r, float lo, float hi) { return (float)r + 0.5f > lo && (float)r - 0.5f < hi; }

void sculpt(uint8_t* g) {
  memset(g, 0, (size_t)GN);

  // ---- fuselage: elliptical sections, drooped nose
  for (int r = 1; r <= 32; ++r) {
    float w = fuselageW((float)r);
    float hh = 0.8f * w < 0.7f ? 0.7f : 0.8f * w;
    float droop = (6.0f - (float)r) / 6.0f;
    float z0 = 4.0f - 0.7f * (droop > 0.0f ? droop : 0.0f);
    for (int z = 0; z < GZ; ++z) {
      for (int x = 0; x < GX; ++x) {
        float xc = xcOf(x) / w, zz = ((float)z - z0) / hh;
        if (xc * xc + zz * zz <= 1.05f) {
          int tone = 4;
          if (r <= 3) tone = 1;                                   // radome
          else if ((float)z < z0 - 0.5f * hh) tone = 2;           // belly
          g[gidx(x, r, z)] = code(JM_BODY, tone);
        }
      }
    }
  }

  // ---- side intakes (row-12 face is the dark mouth)
  for (int r = 12; r <= 19; ++r)
    for (int z = 3; z <= 5; ++z)
      for (int x = 0; x < GX; ++x) {
        float ax = absf(xcOf(x));
        if (ax >= 3.0f && ax <= 4.6f)
          g[gidx(x, r, z)] = (r == 12) ? code(JM_NOZZLE, 0) : code(JM_BODY, 3);
      }

  // ---- canopy bubble (only where the fuselage is not)
  int canopyMinRow = GY;
  for (int z = 6; z < GZ; ++z)
    for (int r = 0; r < GY; ++r)
      for (int x = 0; x < GX; ++x) {
        if (g[gidx(x, r, z)]) continue;
        float ex = xcOf(x) / 1.7f, ey = ((float)r - 10.0f) / 4.0f, ez = ((float)z - 6.6f) / 1.4f;
        if (ex * ex + ey * ey + ez * ez <= 1.0f) {
          g[gidx(x, r, z)] = code(JM_CANOPY, 1);
          if (r < canopyMinRow) canopyMinRow = r;
        }
      }
  for (int r = 0; r < GY; ++r)
    for (int x = 0; x < GX; ++x) {
      int topZ = -1;
      for (int z = GZ - 1; z >= 6; --z)
        if (g[gidx(x, r, z)] && codeMat(g[gidx(x, r, z)]) == JM_CANOPY) { topZ = z; break; }
      for (int z = 6; z < GZ; ++z) {
        uint8_t& c = g[gidx(x, r, z)];
        if (!c || codeMat(c) != JM_CANOPY) continue;
        if (r == 13) c = code(JM_BODY, 2);                            // frame bar
        else if (r <= canopyMinRow + 1) c = code(JM_CANOPY, 0);       // windscreen
        else if (z == topZ && absf(xcOf(x)) < 1.0f) c = code(JM_CANOPY, 2);  // top glint line
      }
    }

  // ---- main wing, cranked arrow. Two-layer plate: slice 4 is the top skin (tone 4), slice 3
  // the underside (tone 3); slice 5 thickens the root where |xc| <= 5.5 and row >= 16.
  for (int x = 0; x < GX; ++x) {
    float ax = absf(xcOf(x));
    if (ax < 2.5f) continue;
    float le = wingLE(ax), te = wingTE(ax);
    for (int r = 0; r < GY; ++r) {
      if (!rowIn(r, le, te)) continue;
      int tone = 4, mat = JM_BODY;
      if (ax >= 11.5f) { mat = JM_ACCENT; tone = 2; }     // red outer columns
      else if (ax > 7.0f && ax < 8.0f) tone = 3;          // panel line at the crank
      uint8_t& c4 = g[gidx(x, r, 4)];
      if (!c4) c4 = code(mat, tone);
      uint8_t& c3 = g[gidx(x, r, 3)];
      if (!c3) c3 = code(mat, mat == JM_BODY ? 3 : 1);
      if (ax <= 5.5f && r >= 16) {
        uint8_t& c5 = g[gidx(x, r, 5)];
        if (!c5) c5 = code(JM_BODY, 4);
      }
    }
  }

  // ---- canards: top skin slice 5, underside slice 4. Root extended to |xc| = 2.5 so they
  // meet the fuselage.
  for (int x = 0; x < GX; ++x) {
    float ax = absf(xcOf(x));
    if (ax < 2.0f || ax > 6.0f) continue;
    float le = 8.0f + 0.8f * (ax - 3.0f);
    for (int r = 8; r <= 11; ++r) {
      if (!rowIn(r, le, 11.0f)) continue;
      bool tip = ax > 5.0f;
      uint8_t& c5 = g[gidx(x, r, 5)];
      if (!c5) c5 = tip ? code(JM_ACCENT, 2) : code(JM_BODY, 4);
      uint8_t& c4 = g[gidx(x, r, 4)];
      if (!c4) c4 = tip ? code(JM_ACCENT, 1) : code(JM_BODY, 3);
    }
  }

  // ---- dorsal spine fairing: canopy to fin (slice 7, rows 14..23, the fin's two columns)
  for (int r = 14; r <= 23; ++r)
    for (int x = 12; x <= 13; ++x)
      if (!g[gidx(x, r, 7)]) g[gidx(x, r, 7)] = code(JM_BODY, 5);

  // ---- twin ventral strakes under the rear fuselage (|xc| = 2.5): rows 23..31 at slices
  // 1..2, one slice deeper (slice 0) along rows 26..29.
  for (int r = 23; r <= 31; ++r)
    for (int z = 0; z <= 2; ++z) {
      if (z == 0 && (r < 26 || r > 29)) continue;
      if (z == 1 && (r == 23 || r == 31)) continue;   // swept ends
      uint8_t c = code(JM_BODY, z == 0 ? 2 : 3);
      if (!g[gidx(10, r, z)]) g[gidx(10, r, z)] = c;
      if (!g[gidx(15, r, z)]) g[gidx(15, r, z)] = c;
    }

  // ---- spine chevron: top fuselage voxels, rows 22..25, |xc| <= 1.5
  for (int r = 22; r <= 25; ++r)
    for (int x = 0; x < GX; ++x) {
      if (absf(xcOf(x)) > 1.5f) continue;
      for (int z = GZ - 1; z >= 0; --z) {
        uint8_t c = g[gidx(x, r, z)];
        if (c && codeMat(c) == JM_BODY) { g[gidx(x, r, z)] = code(JM_ACCENT, 2); break; }
      }
    }

  // ---- single fin, xc = +-0.5, slices 6..10
  for (int z = 6; z <= 10; ++z) {
    float le = 22.0f + 1.6f * (float)(z - 6);
    float te = 32.0f - 0.3f * (float)(z - 6);
    int leRow = -1;
    for (int r = 0; r < GY; ++r)
      if (rowIn(r, le, te)) { leRow = r; break; }
    for (int x = 12; x <= 13; ++x)
      for (int r = 0; r < GY; ++r) {
        if (!rowIn(r, le, te)) continue;
        uint8_t& c = g[gidx(x, r, z)];
        if (c) continue;
        if (z >= 9) c = code(JM_ACCENT, 2);
        else if (z == 8 && r == leRow) c = code(JM_ACCENT, 1);
        else c = code(JM_BODY, 5);
      }
  }

  // ---- nozzle: ring rows 31..33, emissive core disc at row 33
  for (int r = 31; r <= 33; ++r)
    for (int z = 0; z < GZ; ++z)
      for (int x = 0; x < GX; ++x) {
        float xc = xcOf(x), zc = (float)z - 4.0f;
        float rr = sqrtf(xc * xc + zc * zc);
        if (rr >= 1.3f && rr <= 2.4f) g[gidx(x, r, z)] = code(JM_NOZZLE, 0);
        else if (r == 33 && rr < 1.3f) g[gidx(x, r, z)] = code(JM_GLOW, 0);
      }

  // ---- nav lights: wingtip outermost voxels at row 27, strobe on the fin top rear
  if (g[gidx(0, 27, 4)]) g[gidx(0, 27, 4)] = code(JM_NAV_RED, 0);
  if (g[gidx(GX - 1, 27, 4)]) g[gidx(GX - 1, 27, 4)] = code(JM_NAV_GREEN, 0);
  for (int r = GY - 1; r >= 0; --r) {
    bool any = false;
    for (int x = 12; x <= 13; ++x)
      if (g[gidx(x, r, GZ - 1)]) { g[gidx(x, r, GZ - 1)] = code(JM_STROBE, 0); any = true; }
    if (any) break;
  }
}

// Quantise a summed direction to the nearest of the 26 directions.
uint8_t quantiseNormal(int sx, int sy, int sz) {
  int best = 13;  // (0,0,1)
  float bestDot = -1e9f;
  for (int i = 0; i < JET_NORMAL_COUNT; ++i) {
    const int8_t* d = JET_NORMAL_DIR[i];
    float len2 = (float)(d[0] * d[0] + d[1] * d[1] + d[2] * d[2]);
    float dp = (float)(d[0] * sx + d[1] * sy + d[2] * sz) / sqrtf(len2);
    if (dp > bestDot) { bestDot = dp; best = i; }
  }
  return (uint8_t)best;
}

int normalIndexOf(int x, int y, int z) {
  for (int i = 0; i < JET_NORMAL_COUNT; ++i)
    if (JET_NORMAL_DIR[i][0] == x && JET_NORMAL_DIR[i][1] == y && JET_NORMAL_DIR[i][2] == z) return i;
  return 13;
}

char matChar(uint8_t c) {
  if (!c) return '.';
  switch (codeMat(c)) {
    case JM_BODY: return codeTone(c) <= 2 ? '=' : '#';
    case JM_ACCENT: return 'r';
    case JM_CANOPY: return 'c';
    case JM_NOZZLE: return 'n';
    case JM_GLOW: return '*';
    case JM_NAV_RED: return 'R';
    case JM_NAV_GREEN: return 'G';
    case JM_STROBE: return 'S';
    default: return '?';
  }
}

// See JET_KEY_* in jet_model.h.
uint16_t renderKey(int mat, int tone, int nidx, unsigned faces) {
  static const uint8_t SLOT_BASE[JM_COUNT] = {0, 7, 11, 14, 15, 16, 17, 18};
  const bool nav = mat >= JM_NAV_RED;
  const int slot = SLOT_BASE[mat & 7] + tone;
  unsigned key = (nav ? (unsigned)JF_ALL : faces) |
                 ((unsigned)(nidx * JET_SLOT_COUNT + slot) << JET_KEY_CIDX_SHIFT) |
                 (nav ? (unsigned)JET_KEY_NAV : 0u);
  return (uint16_t)key;
}

}  // namespace

bool JetModel::build() {
  n_ = 0;
  memset(sliceStart_, 0, sizeof(sliceStart_));
  memset(sliceExt_, 0, sizeof(sliceExt_));
  memset(sil_, 0, sizeof(sil_));
  uint8_t* g = (uint8_t*)malloc((size_t)GN);
  if (!g) return false;
  sculpt(g);

  // Top-down silhouette (every occupied column per row).
  for (int r = 0; r < GY; ++r) {
    uint32_t bits = 0;
    for (int x = 0; x < GX; ++x)
      for (int z = 0; z < GZ; ++z)
        if (g[gidx(x, r, z)]) { bits |= 1u << x; break; }
    sil_[r] = bits;
  }

  const int nUp = normalIndexOf(0, 0, 1);
  const int nUpL = normalIndexOf(-1, 0, 1);
  const int nUpR = normalIndexOf(1, 0, 1);
  const int nDown = normalIndexOf(0, 0, -1);
  int n = 0;
  bool ok = true;
  // Slice-major, then row (nose first), then x: the list comes out already sorted.
  for (int z = 0; z < GZ && ok; ++z) {
    sliceStart_[z] = (uint16_t)n;
    for (int r = 0; r < GY && ok; ++r)
      for (int x = 0; x < GX; ++x) {
        uint8_t c = g[gidx(x, r, z)];
        if (!c) continue;
        bool e6[6] = {!at(g, x - 1, r, z), !at(g, x + 1, r, z), !at(g, x, r - 1, z),
                      !at(g, x, r + 1, z), !at(g, x, r, z - 1), !at(g, x, r, z + 1)};
        if (!(e6[0] || e6[1] || e6[2] || e6[3] || e6[4] || e6[5])) continue;  // interior
        if (n >= cfg::JET_MAX_VOX) { ok = false; break; }
        int nidx;
        if (e6[4] && e6[5]) {
          // One-slice plate (wing, canard, nose tip): an "up" normal, tilted outward on the
          // wings like a small dihedral so a bank lights the raised wing and darkens the
          // lowered one. The renderer treats these three normals as two-sided.
          float xc = xcOf(x);
          nidx = (xc >= 2.5f) ? nUpR : (xc <= -2.5f ? nUpL : nUp);
        } else {
          // Sum of directions to empty neighbours; 6-neighbours count twice (6 + 26 rule).
          int sx = 0, sy = 0, sz = 0;
          for (int dz = -1; dz <= 1; ++dz)
            for (int dr = -1; dr <= 1; ++dr)
              for (int dx = -1; dx <= 1; ++dx) {
                if (!dx && !dr && !dz) continue;
                if (at(g, x + dx, r + dr, z + dz)) continue;
                int w = ((dx != 0) + (dr != 0) + (dz != 0) == 1) ? 2 : 1;
                sx += dx * w; sy -= dr * w; sz += dz * w;   // model y = toward the nose = -row
              }
          if (sx == 0 && sy == 0 && sz == 0) {
            // Symmetric exposure: fall back to the first open 6-face, preferring up.
            if (e6[5]) sz = 1; else if (e6[4]) sz = -1;
            else if (e6[0]) sx = -1; else if (e6[1]) sx = 1;
            else if (e6[2]) sy = 1; else sy = -1;
          }
          nidx = quantiseNormal(sx, sy, sz);
          // Wing / canard / intake skins (outside the fuselage): tilt the flat up/down normal outward like a
          // small dihedral, so a bank lights the raised wing and darkens the lowered one.
          float xc = xcOf(x);
          if ((xc >= 3.5f || xc <= -3.5f) && (nidx == nUp || nidx == nDown))
            nidx = normalIndexOf(xc > 0.0f ? 1 : -1, 0, nidx == nUp ? 1 : -1);
        }
        const unsigned faces = (e6[0] ? (unsigned)JF_NX : 0u) | (e6[1] ? (unsigned)JF_PX : 0u) | (e6[2] ? (unsigned)JF_PY : 0u) |
                               (e6[3] ? (unsigned)JF_NY : 0u) | (e6[4] ? (unsigned)JF_NZ : 0u) | (e6[5] ? (unsigned)JF_PZ : 0u);
        rkey_[n] = renderKey(codeMat(c), codeTone(c), nidx, faces);
        JetVox& v = vox_[n++];
        v.a2 = (int8_t)(2 * x - 25);
        v.b = (int8_t)(cfg::JET_PIVOT_ROW - r);
        v.s = (int8_t)(z - cfg::JET_PIVOT_SLICE);
        v.mat = (uint8_t)codeMat(c);
        v.tone = (uint8_t)codeTone(c);
        v.normal = (uint8_t)nidx;
      }
  }
  free(g);
  if (!ok) { n_ = 0; return false; }
  sliceStart_[GZ] = (uint16_t)n;
  // Slices after an early stop above keep 0; fix them so ranges stay monotonic.
  for (int z = 1; z <= GZ; ++z)
    if (sliceStart_[z] < sliceStart_[z - 1]) sliceStart_[z] = sliceStart_[z - 1];
  n_ = (uint16_t)n;
  for (int z = 0; z < GZ; ++z) {
    int8_t* e = sliceExt_[z];
    e[0] = e[1] = e[2] = e[3] = 0;
    for (int i = sliceStart_[z]; i < sliceStart_[z + 1]; ++i) {
      const JetVox& v = vox_[i];
      if (i == sliceStart_[z] || v.a2 < e[0]) e[0] = v.a2;
      if (i == sliceStart_[z] || v.a2 > e[1]) e[1] = v.a2;
      if (i == sliceStart_[z] || v.b < e[2]) e[2] = v.b;
      if (i == sliceStart_[z] || v.b > e[3]) e[3] = v.b;
    }
  }
  return true;
}

void JetModel::dumpAscii(void (*log)(const char*)) const {
  if (!log) return;
  // Works straight from the shell list (no grid, no heap): the top-most / side-most shell
  // voxel is what the eye sees, so interior voxels are irrelevant for the views.
  char line[200];
  snprintf(line, sizeof(line), "[JET] Kestrel voxels=%d (shell) grid=%dx%dx%d", (int)n_, GX, GY, GZ);
  log(line);
  log("[JET] legend: #/= body (light/dark)  r accent  c canopy  n nozzle  * glow  R/G nav  S strobe");
  log("[JET] top view (nose up, + right):");
  for (int r = 0; r < GY; ++r) {
    int k = snprintf(line, sizeof(line), "[JET] %2d ", r);
    int8_t bestZ[GX];
    for (int x = 0; x < GX; ++x) { bestZ[x] = -1; line[k + x] = '.'; }
    for (int i = 0; i < n_; ++i) {
      const JetVox& v = vox_[i];
      if (cfg::JET_PIVOT_ROW - v.b != r) continue;
      int x = (v.a2 + 25) / 2, z = v.s + cfg::JET_PIVOT_SLICE;
      if (x < 0 || x >= GX || z <= bestZ[x]) continue;
      bestZ[x] = (int8_t)z;
      line[k + x] = matChar(code(v.mat, v.tone));
    }
    line[k + GX] = 0;
    log(line);
  }
  log("[JET] side view (nose left, seen from the right):");
  for (int z = GZ - 1; z >= 0; --z) {
    int k = snprintf(line, sizeof(line), "[JET] s%2d ", z);
    int8_t bestX[GY];
    for (int r = 0; r < GY; ++r) { bestX[r] = -1; line[k + r] = '.'; }
    for (int i = sliceBegin(z); i < sliceEnd(z); ++i) {
      const JetVox& v = vox_[i];
      int x = (v.a2 + 25) / 2, r = cfg::JET_PIVOT_ROW - v.b;
      if (r < 0 || r >= GY || x <= bestX[r]) continue;
      bestX[r] = (int8_t)x;
      line[k + r] = matChar(code(v.mat, v.tone));
    }
    line[k + GY] = 0;
    log(line);
  }
  // Per-slice shell maps, side by side (slices 0..5, then 6..10).
  for (int part = 0; part < 2; ++part) {
    int z0 = part ? 6 : 0, z1 = part ? GZ : 6;
    int k = snprintf(line, sizeof(line), "[JET] slices ");
    for (int z = z0; z < z1; ++z) k += snprintf(line + k, sizeof(line) - (size_t)k, "s%-26d", z);
    log(line);
    for (int r = 0; r < GY; ++r) {
      k = snprintf(line, sizeof(line), "[JET] %2d ", r);
      for (int z = z0; z < z1; ++z) {
        char* seg = line + k;
        for (int x = 0; x < GX; ++x) seg[x] = '.';
        for (int i = sliceBegin(z); i < sliceEnd(z); ++i) {
          const JetVox& v = vox_[i];
          if (cfg::JET_PIVOT_ROW - v.b != r) continue;
          int x = (v.a2 + 25) / 2;
          if (x >= 0 && x < GX) seg[x] = matChar(code(v.mat, v.tone));
        }
        k += GX;
        line[k++] = ' ';
      }
      line[k] = 0;
      log(line);
    }
  }
  int cnt[JM_COUNT] = {};
  for (int i = 0; i < n_; ++i)
    if (vox_[i].mat < JM_COUNT) cnt[vox_[i].mat]++;
  snprintf(line, sizeof(line),
           "[JET] count=%d body=%d accent=%d canopy=%d nozzle=%d glow=%d nav=%d/%d strobe=%d",
           (int)n_, cnt[JM_BODY], cnt[JM_ACCENT], cnt[JM_CANOPY], cnt[JM_NOZZLE], cnt[JM_GLOW],
           cnt[JM_NAV_RED], cnt[JM_NAV_GREEN], cnt[JM_STROBE]);
  log(line);
}

}  // namespace kj
