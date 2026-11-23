// jet_model.h - the ORIGINAL "Kestrel" voxel jet, sculpted procedurally at boot.
// OWNER: group C (jet_model.cpp). SHARED CONTRACT.
//
// Grid: x 0..25 (span), row 0..33 (0 = nose), slice 0..10 (0 = belly). Pivot row 19, slice 4,
// x centre 12.5. Stored voxel coordinates relative to the pivot:
//   a2 = 2*x - 25  (across, doubled, odd; + = right wing)   -> a = a2 * 0.5 u
//   b  = 19 - row  (+ = toward the nose)
//   s  = slice - 4 (+ = up)
// Silhouette (single tall fin, cranked-arrow delta, canards, two side intakes, one nozzle,
// silver/red/gold) is specified in DESIGN.md section 6.1. No reference model data is used.
#pragma once
#include <stdint.h>
#include "config.h"

namespace kj {

enum JetMat : uint8_t {
  JM_BODY = 0,    // tones 0..6  -> pal::JET_BODY0 + tone
  JM_ACCENT,      // tones 0..3  -> pal::JET_ACCENT0 + tone
  JM_CANOPY,      // tones 0..2  -> pal::JET_CANOPY0 + tone
  JM_NOZZLE,      // pal::JET_NOZZLE (unlit)
  JM_GLOW,        // pal::JET_GLOW, emissive (flame-tinted under boost)
  JM_NAV_RED,     // emissive, blinks
  JM_NAV_GREEN,   // emissive, blinks
  JM_STROBE,      // emissive, double flash every cfg::STROBE_PERIOD
  JM_COUNT
};

struct JetVox {
  int8_t a2, b, s;   // see header comment
  uint8_t mat;       // JetMat
  uint8_t tone;      // base tone within the family (lighting shifts it at render time)
  uint8_t normal;    // index into JET_NORMAL_DIR (0..25)
};

// Exposed-face bits: which of the 6 axis neighbours of a shell voxel are empty, in model axes
// (x = right, y = forward/nose, z = up). Used by the renderer for exact back-face culling (a
// voxel with no exposed face toward the eye cannot be seen) and to pick the shadow samples.
enum JetFace : uint8_t {
  JF_NX = 1, JF_PX = 2,    // -x / +x (left / right)
  JF_PY = 4, JF_NY = 8,    // +y (nose-ward, row - 1) / -y (tail-ward)
  JF_NZ = 16, JF_PZ = 32,  // -z (belly) / +z (top)
  JF_ALL = 63
};

// (material, tone) slot of a voxel: BODY 0..6, ACCENT 7..10, CANOPY 11..13, NOZZLE 14, GLOW 15,
// NAV_RED 16, NAV_GREEN 17, STROBE 18.
constexpr int JET_SLOT_COUNT = 19;

// Per-voxel render key (JetModel::renderKeys(), parallel to voxels()):
//   bits 0..5  JetFace mask (all set for nav lights / strobe: never face-culled)
//   bits 6..14 normal * JET_SLOT_COUNT + slot  (< 26 * 19 = 494)
//   bit  15    nav light or strobe voxel
constexpr int JET_KEY_CIDX_SHIFT = 6;
constexpr uint16_t JET_KEY_NAV = 0x8000u;

constexpr int JET_NORMAL_COUNT = 26;
// The 26 quantised outward directions (components -1/0/1, never all zero), in model axes
// (x = right/across, y = forward/nose, z = up). Defined in jet_model.cpp.
extern const int8_t JET_NORMAL_DIR[JET_NORMAL_COUNT][3];

class JetModel {
 public:
  // Sculpt into a temporary heap grid (26*34*11 B, freed before return), keep shell voxels
  // (any empty 6-neighbour), compute quantised normals (sum of empty-neighbour directions),
  // sort by slice, build the top-down silhouette. Returns false on overflow/alloc failure.
  bool build();
  int count() const { return n_; }
  const JetVox* voxels() const { return vox_; }
  // Per-voxel render key, parallel to voxels() (see JET_KEY_* above).
  const uint16_t* renderKeys() const { return rkey_; }
  // Voxels of slice index k (0..JET_GRID_Z-1, k = s + PIVOT_SLICE) are [sliceBegin(k), sliceEnd(k)).
  int sliceBegin(int k) const { return sliceStart_[k]; }
  int sliceEnd(int k) const { return sliceStart_[k + 1]; }
  // a2 / b extent of slice k's voxels: {a2Min, a2Max, bMin, bMax} (all 0 for an empty slice).
  const int8_t* sliceExtent(int k) const { return sliceExt_[k]; }
  // Top-down silhouette: row r (0 = nose) bit x set if any voxel occupies column x.
  const uint32_t* silhouette() const { return sil_; }
  // ASCII top view + side view + per-slice dumps, line by line (sim --model, serial 'm').
  void dumpAscii(void (*log)(const char*)) const;

 private:
  JetVox vox_[cfg::JET_MAX_VOX] = {};
  uint16_t rkey_[cfg::JET_MAX_VOX] = {};
  uint16_t n_ = 0;
  uint16_t sliceStart_[cfg::JET_GRID_Z + 1] = {};
  int8_t sliceExt_[cfg::JET_GRID_Z][4] = {};
  uint32_t sil_[cfg::JET_GRID_Y] = {};
};

}  // namespace kj
