// render_world.h - terrain/sea/cloud column march and the sky (gradient, stars, moon/sun).
// OWNER: group B (render_world.cpp, render_sky.cpp). SHARED CONTRACT.
//
// Frame order (Game::render): clear -> renderWorld -> renderSky -> objects -> jet -> fx -> HUD.
// renderWorld writes only EMPTY pixels, front to back, and writes depth for everything it draws.
// renderSky fills every pixel that is still EMPTY: rows above the horizon get sky bands /
// stars / moon, rows below get pal::HAZE (far sea). After renderSky no EMPTY pixel remains.
#pragma once
#include <stdint.h>
#include "types.h"
#include "frame.h"
#include "camera.h"
#include "world.h"
#include "perf.h"

namespace kj {

struct SeaFx {
  float time;        // game clock (s), drives surf flicker, glints, ripple drift
  int moonX;         // moon column (logical px) from skyMoonScreen(); -1000 if not visible
  bool moonPath;     // draw the moon/sun glitter path on near sea
  bool cloudShadows; // day/sunset only (night: false)
};

// One-time tables (noise64 animation texture). Call after World::generate.
void worldRenderInit(uint32_t seed);

// Front-to-back voxel-space march. Definition is JET_HOT (IRAM).
void renderWorld(FrameBuf& fb, const View& v, const World& w, Tod tod, const SeaFx& sea,
                 RenderStats& st);

// Star list (cfg::STARS), Milky Way dust (cfg::MILKY_DUST), shooting-star timer.
void skyInit(uint32_t seed);
// Screen position of the moon (night) / sun (sunset). false if not drawn (day, behind, off-screen).
bool skyMoonScreen(const View& v, Tod tod, int& x, int& y);
// Fills all remaining EMPTY pixels (see header comment). dt advances shooting stars.
void renderSky(FrameBuf& fb, const View& v, Tod tod, float time, float dt, RenderStats& st);

}  // namespace kj
