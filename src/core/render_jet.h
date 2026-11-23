// render_jet.h - voxel-splat renderer for the jet, its afterburner, nav lights and shadow.
// OWNER: group C (render_jet.cpp). SHARED CONTRACT.
//
// Jet axes (world frame; psi = yaw, r = totalRoll = bank + rollAngle, th = pitch):
//   f0    = (sin psi, -cos psi, 0)
//   right = (cos psi*cos r, sin psi*cos r, -sin r)
//   up0   = (cos psi*sin r, sin psi*sin r,  cos r)
//   fwd   = f0*cos th + up0*sin th
//   up    = up0*cos th - f0*sin th
// Voxel world position = pos + (a2*0.5)*right + b*fwd + s*up  (see jet_model.h).
#pragma once
#include <stdint.h>
#include "types.h"
#include "frame.h"
#include "camera.h"
#include "perf.h"

namespace kj {

class JetModel;
class World;
class Palette;

struct JetPose {
  Vec3 pos;          // pivot (world, wrapped)
  float yaw, roll, pitch;   // roll = JetState::totalRoll()
  float boost;       // 0..1
  float yawRateN;    // yawRate / MAX_YAW, -1..1 (flame curl)
  float climbRateN;  // pitchRate / CLIMB_RATE_UP, -1..1 (flame curl)
  float time;        // game clock (flicker, strobe)
  float assemble;    // 0..1 slice-drop intro progress (1 = fully assembled)
  bool visible;      // false = skip (crash, invulnerability blink off-phase)
};

struct JetAxes {
  Vec3 right, fwd, up;
};
JetAxes jetAxes(const JetPose& p);

// World-space anchor points used by fx (trails, smoke, spray) and audio.
struct JetAnchors {
  Vec3 tipL, tipR;     // wingtips
  Vec3 nozzle;         // nozzle centre
  Vec3 flameTip;       // end of the current flame
  float flameLen;      // u
};
JetAnchors jetAnchors(const JetPose& p);
JetAnchors jetAnchors(const JetPose& p, const JetAxes& ax);  // ax = jetAxes(p), already computed

// Shadow: projects the shell voxels that have an exposed face toward the ground-projection
// direction (they cover the whole footprint) to the ground (offset by cfg::SHADOW_K[tod] *
// height) and recolours a 1-2 px splat with Palette::shade(), depth-tested with tolerance 2.
// Skipped when height above ground > cfg::JET_SHADOW_MAX_H. Call BEFORE renderJet.
// Definition is JET_HOT.
// st (optional): the shadow's samples are added to st->voxels (1/2 each, see render_jet.cpp)
// so the simulator's ESP32 cost model sees this work.
void renderJetShadow(FrameBuf& fb, const View& v, const JetModel& m, const JetPose& p,
                     const World& w, const Palette& pal, Tod tod, RenderStats* st = nullptr);

// Body splat (back-to-front slices; voxels whose exposed faces all face away from the eye are
// skipped; per-frame precomputed 8x8 sub-pixel "stamps" of the 4 sub-samples per voxel,
// per-normal lighting from the light direction, emissive nav lights + glow at night),
// afterburner (writes depth, max-heat rule), shock diamonds, then 1 px outline
// (pal::JET_OUTLINE) around body pixels. st.voxels counts drawn voxels, st.voxWrites stamp
// pixel writes. Definition is JET_HOT (its per-frame setup lives in flash).
// bounds (optional) receives the screen box of jet + flame (conservative).
void renderJet(FrameBuf& fb, const View& v, const JetModel& m, const JetPose& p, Tod tod,
               RenderStats& st, Rect* bounds);

}  // namespace kj
