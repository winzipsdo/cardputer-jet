# MOONSKIM: final technical design

MOONSKIM is an original pixel-art jet game for the M5Stack Cardputer-Adv. You fly a voxel jet called "Kestrel", seen from a third-person chase camera, over a moonlit ocean dotted with terraced islands and under a twinkling sky. You steer by tilting the device, fly through ring gates against the clock, and score extra for flying low and fast.

This is the binding design. The shared headers under `src/core/` and `src/platform/cardputer/plat.h` and the file `platformio.ini` are the contract between the implementation groups. They are reproduced verbatim in Appendix A. Where this text and a header disagree, the header wins. Report any such disagreement so it can be fixed.

Labels used throughout:
- **[E]** marks an estimate that must be confirmed on the device. Counters in `[STAT]` exist so it can be checked.
- **[V]** marks a fact verified in library source or by a compile. The headers in this repository were compile-checked with MSVC `/W4` and with the device toolchain (`pio run`, platform espressif32@8.0.0, Arduino core 3.3.12). That device build also confirmed these APIs exist: `Speaker config().task_pinned_core`, `HWCDC::setTxTimeoutMs`, `M5.Imu.getType()` and `lgfx::Bus_SPI::config()`.

Research inputs are in `docs/research-hw.md`, `docs/research-ref.md` and `docs/research-perf.md`.

---

## 1. Verdict on the three candidate designs

Each design was scored from 1 to 10 on five criteria.

| Criterion | perf (KESTREL-P) | visual (KESTREL-V) | feel (MOONSKIM) |
|---|---|---|---|
| Frame-rate safety | **9**: 120x68, one task, render about 4 ms against a 13.5 ms DMA push | 6: native 240x135 world at about 8–12 ms [E] needs a quality scaler; three core-0 tasks | 8: 120x68, but a core-0 display task, queues and RGB565 logical buffers |
| Visual charm | 6: solid but plain (hash stars, no moon path in the march, no glow) | **9**: moon path, Milky Way, nav-light glow, 26-normal lighting, island lights, bioluminescent wake | 8: ceiling overcast, beacons, speed lines, Milky Way |
| Game feel | 6: rings and a timer, a little thin | 7: good juice, keyboard fallback | **9**: gyro-fused tilt, soft curve, ring magnet, skim risk/reward, hit-stop, buffered roll |
| Implementability / robustness | **9**: single task, simplest DMA path, explicit fallbacks | 5: dihedral macro tiles, band DMA task, 245 KB static, quality scaler, many tasks | 7: two-core handoff and stamps are fine, but more moving parts |
| Hardware assumptions | 8: correct DMA, I2C and HWCDC handling; `qio` left unchecked | 7: native framebuffers plus depth leave about 80 KB [E]; `-std=gnu++17` override is risky | 8: `lib_ignore IRremote`, stale-IMU handling, `isPlaying` gating |
| **Total (of 50)** | **38** | **34** | **40** |

**Base design: perf.** Its rendering and pipeline architecture is the base: 120x68 8-bit indexed frame, a single game task on core 1, one 64.8 KB DMA RGB565 buffer, the expand happening after `waitDMA`, and rendering overlapping the SPI push.

**Grafted from feel:**
- The whole game layer: Ring Run and Free Flight modes, ring magnet assist, skim risk/reward, the crash/skim-touch rules, hit-stop, shake and flash.
- The tilt algorithm: gyro-fused gravity vector, neutral basis, soft-deadzone curve, neutral drift.
- Island stamps on a macro table (8192 u wrap, no visible repetition).
- The portable synth, the sim script grammar with asserts, and the world-CRC parity check.

**Grafted from visual:**
- The night palette.
- Per-voxel quantised normals for jet lighting.
- The moon glitter path, Milky Way dust and shooting stars.
- Nav-light glow, the bioluminescent wake and island beacon lights.
- The max-heat flame rule.
- The piecewise-linear depth quantiser (the world depth buffer doubles as the jet z-buffer).
- Serial screenshot grab.

**Decisions on open questions:**
- **Resolution:** 120x68 only in v1. There is no native-resolution build.
- **SPI clock:** 40 MHz by default; 80 MHz is a separate env.
- **Cores:** no second render task, and no display task on core 0.
- **I2C:** all I2C on core 1.
- **Speaker:** the speaker task is pinned to core 0.
- **Clouds:** one march handles cloud tops, walls and undersides. There is no separate mode-7 pass and no cloud-tunnel cuts.
- **Screen post effects:** no vignette, bloom or grain.
- **Flash settings:** `flash_mode` and `f_flash` are not overridden; the board defaults are kept.
- **Auto loop:** the `q` key is reserved for it. It is milestone M6 (stretch).

---

## 2. Decisions at a glance

| Topic | Decision |
|---|---|
| Logical frame | 120x68, `uint8_t` palette indices plus `uint8_t` depth (8,160 B each), static `.bss`. It is expanded 2x nearest-neighbour to 240x135. Logical row 67 emits only output row 134. |
| Palette | 256 entries. Entries 0–79 are fixed (sky, stars, moon, jet, fx, HUD). Entries 80–255 are world colours, 8 fog levels × 22 bases. Flash, mist and fade are lerps applied to the whole LUT every frame (about 5 µs). |
| Present | A core-0 present task (prio 5) owns `M5.Display` after setup (`startWrite()` held open forever). Core 1 hands each frame over by copying `px` (8,160 B) and the 2× LUT (1 KB) into a staging buffer (~0.03 ms); core 0 then runs `expand2x` into the single 64,800 B DMA buffer and pushes rows 0–67 and 68–134 as two ≤ 32 KB transactions back to back (sleeping on an esp_timer until just before each half ends). The ~13.4 ms push is fully overlapped with core-1 rendering (§3). |
| Pacing | 60 fps lock (16,667 µs). `JET_FPS_CAP=0` turns it off (bench). |
| World | 12 island stamps of 64² cells (4 u per cell, 256 u each) with 32² max-mips, placed by a 32×32 macro table (ocean, or stamp id plus a transpose flag). The world wraps every 8192 u. Generation is integer-only, so the CRC is identical on PC and device. |
| Renderer | Comanche-style front-to-back march with a lens-shift level camera, a per-column occlusion row, write-if-empty, a fog level per step, and the cloud slab drawn in the same march. |
| Sky | 8 Bayer-dithered bands, 240 listed stars (twinkle, warm and dim stars), 300 Milky Way dust points, the moon with craters and a two-ring halo, shooting stars. At sunset the moon is replaced by the sun; day has no disc. |
| Jet | Original procedural "Kestrel" on a 26×34×11 grid, about 1,000 shell voxels. True 3D splat with 4 sub-samples per voxel, back-to-front slices, 26-normal lighting, outline, nav lights, flame and shadow. |
| Effects | 128-particle pool, 2×48-point trails, 32 wake marks, 20 speed lines, mist overlay. Everything is Bayer-dithered and depth-tested. |
| Game | Ring Run (45 s timer, rings add time) and Free Flight. Combo up to ×8. Bonuses: ROLL, LOW, BOOST, SKIM, CLOSE, LOOP. Fair crash rules. The best score is saved in NVS. |
| Input | Tilt via a gyro-fused up vector, with keyboard summed in as trim or full control. Keys are matched by physical position. One owner of I2C: a core-0 input task (prio 3), kicked by `pace()` 1.5–2.6 ms before each frame; core 1 only copies its snapshot. |
| Audio | Portable 16 kHz synth. Three 384-sample buffers are streamed with `M5.Speaker.playRaw` on channel 0. The speaker task is pinned to core 0. |
| Code | `src/core/` is portable C++17 with no Arduino headers. `src/platform/cardputer/` is the Arduino layer. `tools/sim/` is an MSVC harness that writes PNG and WAV files and prints timings and an ESP32 cost estimate. |

---

## 3. Architecture and frame pipeline

```
core 1: Arduino loopTask (prio 1), stack 12 KB (SET_LOOP_TASK_STACK_SIZE)
--------------------------------------------------------------------------------------------
 t0 ─ plat::pollInput  [copy the input task's snapshot + serial poll ≈0.03 ms]   (ST_INPUT)
    ─ game.update(in)  [states, tilt, flight ×2 substeps, camera, course, fx update] (ST_SIM)
    ─ game.render()    [clear, palette, world, sky, rings, jet, fx, HUD]  (ST_PAL..ST_HUD)
    ─ plat::audioPump  [synth 1–2 blocks of 384 samples]             (ST_AUDIO)
    ─ plat::present    [wait for the staging buffer (ST_DMAWAIT, 0 unless display-bound),
                        copy px + lut2x into it ≈0.03 ms (ST_EXPAND)]
    ─ plat::applySettings, plat::periodicLogs (≤ 3 lines/s), plat::logFlush
    ─ pace to t0 + 16,667 µs: vTaskDelay(1) while > 1.5 ms remain (kick the input task at
      ≤ 2.6 ms left), then spin on esp_timer
core 0: present task (prio 5): take frame → expand2x (≈0.4 ms) → free the staging buffer →
        push rows 0–67 → sleep until ~120 µs before it ends → push rows 68–134 → …
        input task (prio 3): one I2C poll per kick (keyboard if INT, BMI270, BtnA, battery ≈1.1 ms)
        USB-CDC, M5 Speaker mixer task (prio 2), IDLE0.
SPI3 GDMA: ─ frame N top (6.5 ms) ─ N bottom (6.4 ms) ─┤ idle ┤ N+1 top … (period ≥ 13.4 ms)
```

**Why the push is split.** The panel is on SPI3_HOST (M5GFX board data). On the ESP32-S3 the
segmented transfer (SCT) exists only on SPI2, so M5GFX's `Bus_SPI::writeBytes` cuts any push
larger than 32,768 B into 32 KB SPI transactions and busy-waits on the CPU for every one but
the last. Two pushes of 32,640 B and 32,160 B are one transaction each and return at once;
the present task waits for the first half with a timed sleep plus a short `dmaBusy()` spin
(`sp0` ≈ 0.1 ms per frame), so core 0's idle task still runs.

**Why this is safe and what it buys:**
- The display period is ≈ 13.4 ms at 40 MHz (≈ 75 fps uncapped; measured 74.1 fps in the bench env). Rendering of frame N+1 on core 1 overlaps the whole push of frame N, so 60 fps holds while core-1 work stays below ≈ 16.5 ms (it was ≈ 10 ms with the old "second half at the start of the next frame" scheme, which made the frame time ≈ c1 + dmaw). Beyond that, the extra time shows up as dmaw.
- Measured core-1 work on the device: ≈ 8 ms average, ≈ 10 ms max in the Ring Run autopilot bench (§19).
- Core 1 never touches `M5.Display` after setup; core 0 only reads its own staging copy, so the `[SHOT]` path (core 1, game `px`) stays consistent.

**Rules:**
1. Only `src/platform/cardputer/*` touches M5 objects. After setup, `M5.Display` is used only by the core-0 present task.
2. After `plat::begin`, the display transaction stays open. Nothing may call `M5.Display` drawing functions or `endWrite`. Brightness requests from core 1 are applied by the present task.
3. All I2C (keyboard, IMU) happens on the core-0 input task after setup. The ES8311 is touched only inside `Speaker.begin()` / `end()`; `setVolume` is software-only.
4. No heap allocation after `setup()` except the temporary jet build grid, which is freed during init. No `std::vector` or `std::string` in the core.
5. Variable `dt` is used. `game.update` clamps it to 0.05 s and runs flight in fixed 1/120 s sub-steps (at most 6) with an accumulator. Slow motion scales sim dt only. Audio, UI and camera shake use real dt.

**DMA fallback:** if a 64,800 B DMA-capable block cannot be allocated (logged as `[ERR] dma_alloc`):
- Allocate two band buffers of 240×16×2 B.
- Push 9 bands with ping-pong: expand band b+1 while band b DMAs, using `waitDMA` between pushes. BANDS is odd, so band 0 reuses the buffer of the previous frame's band 8: `waitDMA` first.
- The present task expands and pushes the 9 bands from its staging copy, so this path also overlaps core-1 rendering.

---

## 4. Directory layout, responsibilities and owners

Groups:
- **A:** platform, `platformio.ini`, `main`, expand, profiler.
- **B:** world rendering (sky, stars, ocean, terrain, clouds, fog, palettes).
- **C:** jet model, sprite-stack renderer, effects, draw primitives.
- **D:** game logic (flight, camera, tilt, input mapping, rings, scoring, crash, states, HUD, sound).
- **E:** PC simulator.

```
cardputer-jet/
  platformio.ini                         A  build envs (verbatim in Appendix A)
  .gitignore                             A
  docs/DESIGN.md                         -  this document
  docs/research-hw.md, research-ref.md, research-perf.md   -  research inputs
  src/
    main.cpp                             A  setup(): plat::begin, game.init; loop(): §3 sequence; pacing; bench summary
    platform/cardputer/
      plat.h                             A  CONTRACT: platform entry points
      plat_display.cpp                   A  display init, SPI-80 option, DMA buffer (+band fallback), present()
      plat_input.cpp                     A  keyboard drain → keysDown/keysPressed, BtnA, IMU read + axis remap, battery
      plat_audio.cpp                     A  Speaker config (pinned core 0), begin, 3-buffer stream pump, volume/mute
      plat_storage.cpp                   A  Preferences("moonskim") blob load/save with version check
      plat_log.cpp                       A  non-blocking serial, [BOOT]/[MEM]/[STAT]/[IMU]/[KBD] lines, serial cmd parse, [SHOT] dump
    core/                                (portable C++17; compiled by PlatformIO AND by tools/sim/build.bat)
      compat.h                           A  CONTRACT: JET_HOT, JET_DEVICE, rgb565 helpers
      config.h                           *  CONTRACT: all tuning constants (block owners noted inside)
      types.h                            D  CONTRACT: Vec3, Rect, Tod, Assist, Sens, world conventions
      mathx.h                            D  CONTRACT: clamp/ease/wrap/hash/Bayer/Rng (header-only)
      input.h                            D  CONTRACT: KeyPos, KEY_NAMES, ImuSample, InputFrame, Act, KEYMAP
      frame.h                            B  CONTRACT: FrameBuf, depthQ, depth rule
      perf.h / perf.cpp                  A  CONTRACT: Stage, RenderStats, CostModel, Profiler
      expand.h / expand.cpp              A  CONTRACT: expand2x / expand2xRows (JET_HOT)
      palette.h / palette.cpp            B  CONTRACT: index map; colour tables, grading, fog, fx lerps, shade table
      world.h / world.cpp                B  CONTRACT: stamps/mips/macro/clouds generation (integer), queries, CRC, beacons
      render_world.h                     B  CONTRACT: renderWorld / renderSky API
      render_world.cpp                   B  column march (JET_HOT), sea fx, cloud spans, cloud shadows
      render_sky.cpp                     B  bands, stars, Milky Way, moon/sun, shooting stars
      camera.h / camera.cpp              D  CONTRACT: View, project(); ChaseCamera
      draw.h / draw.cpp                  C  CONTRACT: depth-tested point/line/line3D/disc/glow/rect
      jet_model.h / jet_model.cpp        C  CONTRACT: Kestrel sculpt, shell, normals, silhouette, ASCII dump
      render_jet.h / render_jet.cpp      C  CONTRACT: axes/anchors; shadow; splat+light+flame+outline (JET_HOT)
      fx.h / fx.cpp                      C  CONTRACT: particles, trails, wake, speed lines, mist
      flight.h / flight.cpp              D  CONTRACT: JetState; flight model, crash rules, GPWS, easy floor
      tilt.h / tilt.cpp                  D  CONTRACT: TiltController
      course.h / course.cpp              D  CONTRACT: rings placement/pass/assist/draw/guide
      hud.h / hud.cpp / font3x5.h        D  CONTRACT: text + HUD + screens
      audio.h / audio_synth.cpp          D  CONTRACT: Synth
      game.h / game.cpp                  D  CONTRACT: Game, Persist, PlatformApi, Telemetry
  tools/
    sim/
      build.bat                          E  vcvars64 + cl, outputs tools/sim/out/jetsim.exe
      sim_main.cpp                       E  CLI, frame loop, PlatformApi impl (QPC clock, stdout log, INI persist)
      script.h / script.cpp              E  script parser + per-frame input scheduler + asserts
      synth_imu.h / synth_imu.cpp        E  synthetic BMI270 samples from (roll, pitch) relative to a 35° hold pose
      png_write.h / png_write.cpp        E  own PNG encoder (stored deflate, CRC32, Adler-32) + contact sheets
      wav_write.h / wav_write.cpp        E  16-bit mono PCM WAV writer
      scripts/*.jsc                      E  standard scenarios (§18)
      out/                               -  (gitignored) PNG/WAV/log output
    grab_shot.py                         A  device [SHOT] serial dump → PNG (run with PlatformIO's penv python)
```

**Ownership rules:**
- A header marked CONTRACT may only change its public declarations with the agreement of every group that includes it.
- The owner may add private members, private helpers and new `.cpp`-local code freely.
- The owner of a `config.h` block may retune that block's values freely.
- PlatformIO compiles all of `src/` recursively (`-I$PROJECT_SRC_DIR` is set, so platform code includes `"core/game.h"`; core files include each other by bare name).
- The sim compiles `src/core/*.cpp` and `tools/sim/*.cpp` with `/I src/core`.
- `src/core` must compile warning-free under both `-Wall -Wdouble-promotion` (gcc) and `/W4` (MSVC).

**Coding rules for the core:**
- Use `float` only. Write literals with the `f` suffix and use `sinf`, `cosf` and so on.
- No exceptions, no RTTI-dependent code, no static constructors with side effects.
- Integer-only math in world generation (CRC parity).
- One divide per depth step or row in hot loops.
- Large arrays are static in their own `.cpp` files.
- `Game` is one static global.

---

## 5. Camera (group D; projection formula is shared)

**Model:** a level pinhole camera with a vertical lens shift, as in the reference. There is no camera pitch or roll, so the horizon is always a horizontal row and world verticals stay screen verticals. The formula lives in `camera.h::project()`.

**Per frame:**

| Quantity | Formula / value |
|---|---|
| Preset (`v` key) | `view` 0 near (default): dist 88 u, tilt 0.21 rad. 1 far: 120 u, 0.24 rad. 2 high: 150 u, 0.38 rad. |
| dist | Eased toward `CAM_DIST[view]·(1 − 0.16·boost)` at 4/s. |
| f | Spring toward `CAM_F0·(1 − 0.10·boost)`, ω = 14, ζ = 0.55. That is 88 px (hFOV 68.6°) widening under boost, with about 3 % overshoot on ignition. |
| S | f / dist. About 1.0 px/u at cruise, so the 26 u jet span is about 26 px (22 % of W). |
| T | `tan(clamp(tilt − 0.75·sin(pitch), −1.1, 1.1))`, eased at 6/s, plus `tfix`. |
| Cx, Cy | Cx = 60 + shakeX. Cy = 68·(0.64 − 0.03·boost) + shakeY ≈ 43.5. |
| horizon | Cy − f·T ≈ 24.8 at cruise, so the top 36 % is sky. It drops in climbs and rises in dives. |
| viewDist, fogStart | `min(1000, 640 + 2·alt)` and `120 + 1.5·alt` (zc units). |

**Follow:**
- Focus: F += v·dt + (jet − (F + v·dt))·(1 − e^(−3.5 dt)), using wrapped deltas.
- Altitude: alt += climbRate·dt·0.8, then eases toward jet.z at 3/s.
- View heading ψ eases toward jet yaw at 3.4/s along the shortest arc (`wrapPi`). In a full-rate turn it lags about 22°, so the flank and belly show.

**Terrain and cloud avoidance:**
- The eye is at F − forward·dist, at height alt + T·dist.
- Floor = `World::maxGroundAround(eye, 8)` + 5.
- If the eye is below the floor, or inside [CLOUD_BASE − 3, CLOUD_TOP + 3], compute the ΔT that moves it out. It moves below the slab when the jet is below CLOUD_BASE, otherwise above.
- `tfix` eases toward that ΔT at 7/s.

**Shake:**
- `shake` (0..1) is owned by the game. It is set as max(shake, event) and decays e^(−6t).
- Offset: dx = round(2.5·shake·sin 91t), dy = round(2.0·shake·cos 77t).
- A continuous rumble of 0.06·boost + 0.08·skim is added.

**Crash orbit:** the focus freezes at the wreck, ψ += 0.6 rad/s, and dist eases to 110.

---

## 6. The jet (group C)

### 6.1 Original model "Kestrel" (procedural, `jet_model.cpp`)

**Grid:** x 0..25 (span, centre 12.5), row 0..33 (0 = nose), slice 0..10 (0 = belly). Pivot is row 19, slice 4.

The silhouette deliberately differs from the reference (twin canted fins, twin engines, trapezoid delta, slate/orange/teal):
- one tall fin
- a cranked-arrow delta wing
- canards
- two side intakes
- one large nozzle
- a silver body with red accents and a gold canopy

The model is sculpted by evaluating these primitives into a temporary 26×34×11 byte grid (`malloc`, then `free`d):

| Part | Rule (xc = x − 12.5) |
|---|---|
| Fuselage | Rows 1..32. Elliptical section centred (xc 0, slice z0(row) = 4 − 0.7·max(0, (6 − row)/6)), which gives a drooped nose. Half-width w(row), piecewise linear: 1→0.6, 3→1.5, 7→2.3, 12→2.8, 16→3.4, 26→3.4, 30→2.7, 33→2.3. Half-height = max(0.7, 0.8w). Filled when (xc/w)² + ((z − z0)/hh)² ≤ 1.05. BODY tone 4; cells below z0 − 0.5hh are tone 2 (belly). Rows 0..3 are radome tone 1. |
| Side intakes | Boxes \|xc\| ∈ [3.0, 4.6], slices 3..5, rows 12..19. The row-12 face is NOZZLE (dark mouth); the rest is BODY tone 3. |
| Canopy | Ellipsoid centre (0, row 10, slice 6.6), radii (1.7, 4.0, 1.4), cells with slice ≥ 6 not already fuselage. CANOPY tone 1; the top-centre line is tone 2; front 2 rows are tone 0. Frame bar at row 13, BODY tone 2. |
| Main wing (cranked arrow) | Slice 4 (plus slice 5 where \|xc\| ≤ 5.5 and row ≥ 16). Leading edge: row 14 at \|xc\| = 3 → row 21 at \|xc\| = 7 (inner, steep), then → row 25 at \|xc\| = 12.5 (outer). Trailing edge: row 30 out to \|xc\| = 9, then forward to row 28 at the tip. BODY tone 4 on top, tone 3 underneath. Outer 2 columns are ACCENT tone 2. Panel line at \|xc\| = 7 is BODY tone 3. |
| Canards | Slice 5, rows 8..11, \|xc\| ∈ [3, 6], leading edge row 8 + 0.8(\|xc\| − 3). Tip column is ACCENT tone 2. |
| Fin (single) | xc ∈ {−0.5, +0.5}, slices 6..10. Leading edge row 22 + 1.6·(slice − 6); trailing edge row 32 − 0.3·(slice − 6). Slices 9–10 are ACCENT tone 2; slice 8 has an ACCENT tone 1 stripe on the leading-edge cells. |
| Spine chevron | Top fuselage voxels at rows 22..25 with \|xc\| ≤ 1.5 are ACCENT tone 2. |
| Nozzle | Rows 31..33, ring radius 1.3..2.4 around (xc 0, slice 4): NOZZLE. The core disc radius < 1.3 at row 33 is GLOW (emissive). |
| Nav lights | Left wingtip outermost voxel at row 27: NAV_RED. Right: NAV_GREEN. Fin top rearmost voxel: STROBE. |

**Post-processing:**
- Keep the shell (any empty 6-neighbour).
- Normal = the sum of the directions to empty 6- and 26-neighbours, quantised to the nearest of the 26 directions in `JET_NORMAL_DIR`.
- Sort by slice and build `sliceStart`.
- Build the silhouette: top-down bitmask per row.
- **Expected:** 900–1,300 shell voxels. `build()` fails above `JET_MAX_VOX` (1,400).
- `dumpAscii` prints a top view, a side view, per-slice maps and the count. It is used by `jetsim --model` and serial `m`.

### 6.2 Splat renderer (`renderJet`, JET_HOT; device: jet stage ≈ 1.7 ms including shadow, outline, nav lights and the cloud-depth push)

1. **Axes:**
   - Compute `jetAxes(pose)` (formulas in `render_jet.h`).
   - Convert each axis to view space: across = A.x·cψ + A.y·sψ, depth = A.x·sψ − A.y·cψ, height = A.z.
   - Project the pivot to get (Cx-relative across_j, depth_j, z_j) and den_j.
2. **Per-voxel perspective:**
   - den = den_j + q·depth_off. m = mJ/(1 + x) with x = q·depth_off/den_j (|x| ≤ ~0.33), evaluated as a 4th-order polynomial (error ≤ x⁵ ≈ 0.02 px); zc = zJ·(1 + x). No divide, libm call or flash table in any per-voxel loop.
   - sx = Cx + m·(across_j + across_off).
   - sy = Cy − m·((z_j − alt + height_off) + T·(depth_j + depth_off)).
   - zc = dist·den, dq = depthQ(zc).
   - Offsets are a·R + b·F + s·U in view space (a = a2/2). Precompute per slice: base = s·U.
3. **Visibility:** exact back-face culling per voxel: the model stores which of each voxel's 6 faces are exposed (`JetModel::renderKeys`), and a voxel is drawn only if one of them faces the eye (perspective-correct). About 470 of 876 shell voxels are drawn. A culled voxel's solid neighbour along the up axis gets its 4 sub-samples written 2 depth steps behind, which fills the 1 px pinholes culling would otherwise leave.
4. **Order:**
   - Let eyeUp = dot(eye − pos, up).
   - Draw slices in order of decreasing |s − eyeUp_clamped|, farthest first.
   - Within a slice, iterate rows from the end far from the eye (sign of dot(eye − pos, fwd)).
   - The depth test `dq <= depth[p]` plus painter's order resolves the rest.
5. **Sub-samples:**
   - Each voxel writes 4 samples at ±0.25·R̂ ± 0.25·F̂. The deduplicated pixel offsets (4 samples, up-axis gap fill, 2×2 close-up splat) are precomputed per frame as stamps for an 8×8 grid of sub-pixel positions (`buildStamps`), so a voxel writes ~2–3 px with no bounds check away from the frame edge.
   - If |U| on screen > 1.2 px, add 1–2 samples along +U·k/nu to fill slice gaps when seen edge-on.
   - When m ≥ 1.6 each sample writes 2×2 px.
   - Counters: `voxels`, `voxWrites`.
6. **Lighting:**
   - L = normalize(forward(LIGHT_AZ) + (0, 0, 1.1)).
   - Per normal: d = n·L. Shift is +1 if d > 0.55, −1 if d < −0.15, −2 if d < −0.55.
   - Specular +1 for BODY and CANOPY when reflect(−L, n)·toEye > 0.92 (the canopy flashes in moonlight).
   - Index = familyBase + clamp(tone + shift, 0, familyMax).
   - NOZZLE, GLOW and nav materials are unlit. GLOW uses FLAME1 while boost > 0.5.
   - A banked jet therefore brightens its raised wing, and a roll sweeps the light across it.
7. **Assembly intro:** slice k (bottom first) is offset by −20 px·(1 − ease((assemble·(0.05·11 + 0.3) − 0.05k)/0.3)) in screen y. The flame appears only at assemble = 1.
8. **Flame (§6.3)** is drawn after the body.
9. **Outline:**
   - Over the jet bounding box ±1 px, every pixel that is not in 30..49 and not flame becomes `JET_OUTLINE` if it has a 4-neighbour in 31..49 and depth[p] ≥ dq_pivot − 24 (not hidden by nearer world).
   - Outline pixels do not write depth.
10. **Nav lights:**
    - Red and green blink at 1 Hz with 70 % duty. The strobe double-flashes every 1.2 s.
    - Drawn as 1 px emissive plus, at night, `drawGlow` radius 1–2.5 at 25 % density.
11. **Bounds:** the rect returned covers body, flame and outline (used by `debugVar("jet_w")`).

### 6.3 Afterburner (single nozzle, part of `renderJet`, about 0.1 ms)

- **Length:** len = (FLAME_LEN_CRUISE + FLAME_LEN_BOOST·boost)·(0.82 + 0.18·sin 47t·sin 31t). That is 6 u at cruise and 20 u at boost.
- **Samples:** every 0.5 u from the nozzle backward along −fwd. With u = t/len:
  - radius = 2.6·(1 + 0.25·boost)·(1 − u)^0.75·(0.9 + 0.2·sin(1.9·t − 40·time))
  - curl offset = (t²/len)·(0.36·yawRateN·right + 0.30·climbRateN·up)
- **Drawing:** each sample is a projected disc of screen radius r·m.
- **Colour:**
  - Pixel heat h = 0.9u + 0.55·(dist from axis / r) − 0.16·boost.
  - h maps to FLAME0..5 with thresholds 0.28, 0.5, 0.75, 1.0, 1.25.
  - Max-heat rule: a pixel already holding a flame index is overwritten only by a hotter (lower) index.
  - The flame writes depth.
- **Boost extras:**
  - boost > 0.6: shock diamonds, 2 px of FLAME0 at u = 0.28 and 0.52.
  - Glow: Bayer 25 % FLAME2 halo out to r + 2 px over the first 40 %.
- **Counter:** `flamePx`.

### 6.4 Shadow (`renderJetShadow`, before the jet, device ≈ 0.4–0.5 ms below 60 u)

- Skipped when the height above the ground is above 60 u.
- For every shell voxel with an exposed face toward the ground-projection direction (they cover the whole footprint): P = world position, h = groundHeight(P.xy), ground point G = P.xy + (k, k)·(P.z − h), with k = SHADOW_K[tod]. G, its depth and its height are linear in the voxel coordinates (plus the §6.2 polynomial); over uneven ground the cell heights under the jet are read once into a 16×16 grid.
- Project (G, h). Set the pixel to `pal.shade(px)` for a 1 px splat (2×1 when m > 1.2), only if |depth[p] − dq| ≤ 2 (it lies on the visible surface).
- At night, apply it to 50 % of the pixels through a Bayer mask.
- The shadow therefore narrows with bank and turns with rolls automatically.
- Each sample is counted as 1/2 unit in `RenderStats::voxels` so the simulator's cost model sees it.
- **Clouds never hide the jet:** while the jet is in the cloud slab, `Game::render` pushes cloud pixels to the far plane only inside last frame's jet + flame box grown by 8 px (the whole frame when there is no box from the previous frame), so trails and the shadow away from the jet are still hidden by clouds in front of them.

---

## 7. World and sky (group B)

### 7.1 Generation (`World::generate`, integer-only, about 150 ms [E])

**Stamps.** Each of the 12 stamps is 64×64 cells, built from its own seed with `hash32(seed + id)`:
1. **Elevation**, Q16:
   - Value noise from a 64-periodic lattice with smoothstep in integer fixed point. Octaves at cell sizes 32/16/8/4, amplitudes 1, 0.5, 0.25, 0.12.
   - Multiplied by a radial falloff: centre jitter ±8 cells, radius 16–26 cells, elongation 0.7–1.3 at a random angle, smoothstep to 0 at 0.95·radius.
   - The border 4 cells are forced to DEEP.
2. **Sea level:** from a 256-bin histogram so that the land fraction is uniform in [0.30, 0.45] per stamp. This gives atolls, ridges and twin islands.
3. **Water** (level 0), by depth below sea level d (in Q16 of the elevation range): > 0.07 DEEP, > 0.03 MID, > 0.008 SHALLOW, else SURF.
4. **Beach:** 0 < e − sea < 0.012 is SAND (level 0), with 13 % hash SPECK.
5. **Land:** hr = (e − sea − 0.012)/(1 − sea − 0.012).
   - hr < 0.5: grass, level 1 + ⌊hr/0.5·8⌋ (≤ 8).
   - hr < 0.78: rock, levels 9..12.
   - Otherwise: snow, levels 13..15.
6. **Baked slope light** (light from the (−x, −y) side, which the transpose preserves): g = E(x−2, y−2) − E(x+2, y+2).
   - g < −τ: lit variant (GRASS_L, ROCK_L).
   - g > τ: dark variant (GRASS_D, ROCK_D, SNOW_D).
   - τ = 0.007 in elevation units, plus a 1/8 hash sprinkle.
7. **Trees:**
   - On a jittered 5-cell grid, on grass levels 1..5, gated by a second noise sample > 0.55 and a hash < 40 %.
   - Each tree is a radius-2 cone: centre level +2 TREE_TOP, ring level +1 TREE.
8. **Beacons:** 1–2 per stamp on sand or low grass next to water (stamp-local list, transformed per macro instance).

**Mips:** a 2×2 max of height per mip cell; the material comes from the max cell.

**Macro table:**
- 32×32 entries, hash-filled. 45 % of entries are an island: stamp id 0..11, with the transpose flag at 50 %. The rest are `MACRO_OCEAN`.
- No two equal stamp ids may be 4-adjacent.
- The spawn macro tile (16, 16) is forced to ocean, with islands in the neighbouring tiles.

**Clouds:**
- 48 clusters, each with a rank 0..15, visible if rank < 0.40·16.
- Each cluster has 3–7 dome puffs on a line: angle 0.3 ± 0.7 rad, length 4–12 cells, puff radius 1.5–3.5 cells·(1 − 0.45|t|).
- Thickness = round(sqrt(1 − d²)·clamp(0.6r·2, 2, 6)), combined with max.

**Spawn:** the nearest open-water cell to the world centre with land 120–300 u ahead. Yaw faces the land; ties are broken toward LIGHT_AZ, so the moon is ahead at start.

**CRC:** CRC32 (table) over stamps, mips, macro and clouds. Logged as `world_crc=` on device and in the sim; the two must be equal.

### 7.2 Column march (`renderWorld`, JET_HOT, about 1.8 ms typical / 3.5 ms worst [E])

**Setup:**
- `colY[x] = LH` for all 120 columns. `px` is EMPTY and `depth` is 255 (cleared by the game).
- Material → base tables `TOP_BASE[16]` and `SIDE_BASE[16]`:
  - water: SEA_* / SEA_* (same)
  - sand: SAND / SAND_SIDE; speck: SAND_SIDE / SAND_SIDE
  - grass L / N / D: GRASS_LIT / GRASS / GRASS_DARK, all with side GRASS_SIDE
  - tree top / tree: TREE_TOP / TREE, side TREE_SIDE
  - rock L / N / D: ROCK_LIT / ROCK / ROCK_DARK, side ROCK_SIDE
  - snow / snow dark: SNOW / SNOW_SIDE, side SNOW_SIDE

**Depth range:**
- Work in den (zc = dist·den, v = zc − dist).
- Start at den₀ = max(0.15, min(denTerrain, denCloud)):
  - denTerrain = S(alt + T·dist − ZTOP)/(LH − horizon) when that is positive, else 0.15. It is where the tallest terrain first touches the bottom row.
  - denCloud = where the slab rows first enter [0, LH).
- End at zc = viewDist, or after 180 steps.

**Per step:** floats, one divide.
- m = S/den.
- Row of height z: row(z) = horizon + S·(alt + T·dist − z)/den, the same as `project()`.
- `rowNow[l] = floor(row(HEIGHT_U[l]))` for l = 0..15. `rowPrev` keeps the previous step's values.
- Cloud rows `cNow[j] = floor(row(CLOUD_BASE + 3j))` for j = 0..6.
- fog = min(7, round(7·sqrt(sat((zc − fogStart)/(viewDist − fogStart))))).
- `topIdx[16]`, `sideIdx[16]`, cloud indices = pal::world(fog, base).
- dq = depthQ(zc).
- Use the mip (8 u cells) when m < 0.125.
- **Scan start:** across at column x is a = (x + 0.5 − Cx)/m. The world point is F + a·right + v·forward. Convert to Q16.16 cells: wx = (Fx + a₀·cψ + v·sψ)·(65536/CELL_U), wy = (Fy + a₀·sψ − v·cψ)·(65536/CELL_U). The per-pixel step is (cψ, sψ)/m·(65536/CELL_U).
- Cloud walk: the same, minus the drift offset, at 1/CLOUD_CELL_U.
- **Step size:** dv = clamp(den²/(S·(T + q·max(alt, 8))), 0.5, 2.5·den/S), ×1.0. This is about one row of sea per step.

**Per column** (integers only):

```
if (!colDone[x]) {
  c = map(wx, wy); lvl = c>>4; mt = c&15;
  yTop = rowNow[lvl]; yWall = rowPrev[lvl];
  if (yTop < colY[x]) {
     for (y = colY[x]-1; y >= max(yTop,0); --y)
       if (px[y][x]==EMPTY) { px = (y >= yWall) ? sideIdx[mt] : topIdx[mt]; depth = dq; ++fills; }
     colY[x] = max(yTop, 0);
     if lvl==0 && zc < NEAR_FX_DIST: sea extras on the rows just written (§7.3)
  }
  colDone[x] = colY[x] <= max(0, rowNow[15]);       // nothing farther can show (terrain)
}
clouds (if the slab rows at this step intersect [0, LH)): k = cloudThick at the cloud walk position;
  if k: top = CLOUD_BASE + 3k
     front wall (only if this column's previous cloud sample was 0): rows [cPrev[k], cPrev[0]) → CLOUD_SIDE
     if eyeZ > top: top face rows [cNow[k], cPrev[k]) → CLOUD_TOP (k==1: only (x+y)&1 pixels)
     if eyeZ < CLOUD_BASE: underside rows [cPrev[0], cNow[0]) → CLOUD_UNDER (k==1 checkerboard)
  clouds write only EMPTY pixels (+depth) and never move colY.
```

**Early outs:**
- When every column is `colDone` and no cloud rows can intersect the frame further out, stop.
- Steps whose rows are all ≥ LH are skipped cheaply. Their rows are still recorded into rowPrev.

**Cloud shadows** (day and sunset only, `SeaFx.cloudShadows`): for terrain top pixels at fog ≤ 2, if the cloud thickness at (cell centre − (k, k)·CLOUD_BASE) is greater than 1, write `shadeTable()[idx]`.

**After the march, `renderSky` fills the rest:**
- EMPTY rows ≥ horizon become `HAZE`.
- EMPTY rows < horizon become sky (§7.4).

**Counters:** steps, samples (terrain and cloud lookups), fills.

### 7.3 Sea effects (only for h = 0 and zc < 300; fixed palette indices, unfogged)

- **Surf foam:** on SURF cells, `FOAM` when `(noise64[cell & 4095] + (int)(36·t)) & 255 < 110`. The shoreline flickers.
- **Near glints:** two taps scrolling opposite ways, n1 = noise64 at (cellX·4 + 6t, cellY) and n2 at (cellX·4 − 4t, cellY + 17). `GLINT_B` when n1 > 252, `GLINT_A` when n1 > 243 && n2 > 128.
- **Moon (sun) path** (night and sunset, `moonPath`):
  - Applies to water pixels with |x − moonX| < 3 + 0.25·(y − horizon).
  - h = hash3i(cellX, cellY, (int)(8t)) & 255: < 40 gives GLINT_B, < 90 GLINT_A, < 150 MOONPATH, otherwise unchanged.
  - The result is a shimmering vertical path from the horizon down to the screen bottom.
- **Bioluminescent wake** is drawn by fx (C), not here.

### 7.4 Sky (`renderSky`, about 0.2 ms)

**Gradient:**
- span = 0.75·LH = 51 rows. For a row y, t = (horizon − y)/51·7, band = ⌊t⌋ clamped to 0..7.
- Use band+1 where (frac(t) − 0.7)·54 > bayer(x, y).
- Precompute per row (bandA, bandB, threshold). Write only EMPTY pixels.

**Stars** (night, `skyInit`):
- 240 stars, each with azimuth (12-bit turn), elevation (0.5°–55°), class and twinkle phase.
- 25 % are dim, 10 % warm (`STAR_WARM`), and the 6 brightest draw a 5-px "+" at twinkle peaks.
- Twinkle: 1 in 3 stars toggles mid/dim when ((t·rate·40 + phase·16) & 255) < 90.
- Projection at infinity: Δ = az − ψ. Skip if cos Δ < 0.25. sx = Cx + f·tan Δ, sy = horizon − f·tan(el)/cos Δ.
- Stars are drawn only on pixels holding a sky index, never in the lowest 2 bands.

**Milky Way:** 300 dust points within ±7° of a great circle tilted 62°, drawn as `MILKY` at 50 % Bayer on sky pixels.

**Moon:**
- Direction forward(LIGHT_AZ), elevation tan 0.17, so about 15 px above the horizon.
- Disc r = 3 px (`MOON_DISC`) with 2 `MOON_CRATER` pixels.
- Inner halo out to 1.9r: `MOON_GLOW_IN` with Bayer density (1.9 − d/r)·14. Outer halo out to 3.2r: `MOON_GLOW_OUT` at density ≤ 3.
- Only on sky pixels, so clouds and islands hide it.
- `skyMoonScreen()` gives moonX to the march (computed before `renderWorld`).

**Sun** (sunset): same placement, elevation tan 0.08, r = 6 px, same halo indices. Day has no disc.

**Shooting star:** every 8–18 s at night, a 14 px streak over 0.5 s with a 3-tone tail (`STAR_BRIGHT`, `STAR_MID`, `STAR_DIM`), on sky pixels only.

---

## 8. Tilt control (group D, `tilt.cpp`; platform supplies device-frame samples)

**Device frame** (M5 doc axes): X right (toward Enter), Y away from the user (toward the screen), Z out of the keys. Flat on a table: az ≈ +1 g.

The platform applies `cfg::IMU_SRC_*` / `IMU_SGN_*` (identity by default). The runtime sign flags `rollSign` and `pitchSign` live in `Persist` and are flipped by serial `r` / `p`.

**Sampling:** once per frame on core 1 (`M5.Imu.update()`, then `getImuData()`, about 0.7 ms [E]). If `update()` returned 0, `fresh = false` and the gyro step is not integrated twice.

**Filter:**
- The algorithm is in the `tilt.h` header comment.
- Correction time constant τ: 0.25 s when ||a| − 1| < 0.10 g, 2.0 s when < 0.40 g, otherwise no correction.
- No fresh sample for 0.2 s: output frozen at 0, `valid = false`, `[ERR] imu_stale` logged once.

**Calibration:**
- Triggered by: the CALIBRATE state at boot (window 4 s), the countdown (re-zero only if already still; it adopts the new pitch only and keeps the previous roll component, so a bank held while waiting does not become a turn bias), and the `c` key (0.5 s, game paused; ENTER/BKSP end it).
- A fresh but unusable sample (non-finite, |a| < 0.05 g) is handled like a missing one: calibration windows still time out and the output goes stale.
- Accept samples while |gyro| < 8 dps for 0.5 s continuously and ||a| − 1| < 0.1.
- Result: n = mean(â), bias = mean(gyro). Save n to Persist.
- On timeout, use the stored neutral and show "HOLD STILL" once.

**Angles:**
- Neutral basis zn = n, xn = ⊥(X), yn = zn × xn.
- roll = rollSign·atan2(−u·xn, u·zn). + means the right edge is down, which turns right.
- pitch = pitchSign·atan2(−u·yn, u·zn). + means the top is pushed away, which dives.
- climbIn = −shape(pitch); `invert` flips it.
- **Guard:** |pitch| > 60° or u·zn < 0.2 (upside down): output 0, `valid = false`, HUD shows "TILT?".

**Curve** (soft deadzone, `cfg::TILT_*[sens]`): u = sat((|θ| − dz)/(full − dz)), out = sign·u·(0.45 + 0.55u).

| Sensitivity | Roll dz / full | Pitch dz / full |
|---|---|---|
| Low | 3° / 32° | 4° / 28° |
| **Med (default)** | **3° / 25°** | **4° / 22°** |
| High | 2.5° / 19° | 3.5° / 17° |

**Output and drift:**
- Output low-pass: one pole at 10 Hz.
- **Neutral drift** (default on): when |pitch| < 8° and |turn| < 0.3 for more than 2 s, rotate n toward u about xn with τ = 20 s, limited to ±12° from the calibrated neutral.

**Final stick:**
- turn = clamp(tiltTurn + keyTurn + assistTurn, −1, 1); climb likewise.
- Keys ramp toward ±1 at 6/s and back at 10/s.
- With tilt off, or with no IMU (not ADV, or `getType() == m5::imu_none`), keys alone drive the stick. The HUD badge shows KEYS or NO IMU.

---

## 9. Keys (decided; `input.h::KEYMAP`)

| Action | Keys (position) | Notes |
|---|---|---|
| Turn left / right | `,` / `/` ; `a` / `d` | Fn-arrow positions, without Fn |
| Climb / dive | `;` / `.` ; `w` / `s` | `i` inverts the pitch (keys and tilt) |
| Boost (hold) | SPACE, ALT | right thumb / left thumb |
| Barrel roll left / right | `z` / `x` ; `l` / `'` | 1 roll buffered |
| ENTER | confirm / start; in play: roll toward the current turn (alternates when level) | |
| Pause / menu | `p`, `` ` ``, G0 button | |
| Back (menus) / quit to title (pause) | BKSP | |
| Recalibrate tilt | `c` | |
| Tilt on/off | `t` | saved |
| Time of day | `n` | night → sunset → day, 0.5 flash |
| View preset | `v` | near / far / high |
| Stats overlay | `f` | fps + core ms |
| IMU debug overlay + serial stream | `g` | |
| Help overlay | `h` | |
| Mute / volume − / + | `m` / `-` / `=` | step 16 |
| Brightness − / + | `[` / `]` | step 24, min 24 |
| Title: Ring Run / Free Flight | ENTER or `1` / `2` | |
| Auto loop (M6 stretch) | `q` | |

**Keymap verification:** serial `k` echoes `[KBD] down=SPACE,SEMI kp=55,39` on every change.

**Chord risk:** test SPACE + `;` + `,` on the device. If ghosting appears, document it (ALT is the alternative boost key).

---

## 10. Flight model (group D, `flight.cpp`, 1/120 s sub-steps)

All constants are in `config.h` (flight block). Easing is first-order everywhere.

**Boost:**
- Target 1 while held and energy ≥ 0.10 (once engaged, it holds until energy reaches 0).
- Eases up at 5/s and down at 2.4/s.
- Energy drains 0.22/s while boost > 0.5. It regenerates 0.10/s, or 0.35/s while skimming (< 8 u above water). +0.25 per ring.
- Ignition (rising edge) sets the `boostIgnite` event.

**Turning:**
- want = turn·(MAX_YAW lerped to MAX_YAW_BOOST by boost); yawRate eases at 3.2/s; yaw += yawRate·dt.
- bank eases at 6/s toward clamp(yawRate/MAX_YAW, −1, 1)·0.96. Bank is visual only.
- Nose-up in turns: +0.08·|yawRate/MAX_YAW|.

**Climbing:**
- ask = climb > 0 ? climb·1.05 : climb·0.70. With no input, ask = 0 (altitude hold).
- pitch eases at 7/s toward ask, rate-limited to +1.6 / −1.35 rad/s.

**Speed and energy:**
- energy eases at 1.1/s toward 1 − 0.22·sin(pitch)·(pitch > 0 ? 1 − boost : 1), clamped to [0.8, 1.15].
- speed = 68·(1 + 0.8·boost)·energy.

**Motion:**
- dz = speed·sin(pitch)·dt; horizontal = speed·cos(pitch)·dt along forward(yaw), plus the roll dash.
- Ceiling cushion: within 20 u of 220, dz is scaled by gap/20.
- Idle bob: +1.0·cos(0.9t)·dt when |climb| < 0.05.

**Barrel roll:**
- 0.80 s, rollAngle = dir·2π·easeInOutQuad(rollT).
- Lateral dash of 12 u total, with velocity ∝ sin(π·rollT) along right(yaw).
- Cooldown 0.25 s, 1 roll buffered. Rolling pumps g.

**g gauge:** eases at 2.5/s toward clamp(|yawRate|/MAX_YAW + 0.7·rolling + 0.9·boost + |pitchRate|/1.6, 0, 1).

**Crash rules** (ignored while invuln > 0):
- **Terrain:** the pivot is over land with z < ground + 1.5, or the nose probe (pos + forward(yaw)·17, z + 17·sin pitch) is below ground(nose) + 1.0.
- **Sea:** z ≤ 2.0 and (vz < −14 u/s or pitch < −0.35) is a crash. Otherwise it is a **skim touch**: clamp z = 2, vz = 0, pitch = max(pitch, 0), and the `skimTouch` event fires (shake 0.35, spray, slap sound, combo kept).
- **EASY assist:** a terrain floor. It is the max ground over 5 points ahead (0..0.64 s) × 3 lateral (±7 u), + 6, eased up at 9/s and down at 3/s. The jet is lifted to it. Only dives steeper than −0.6 rad with full forward input held for 0.4 s can still crash.
- **Close call:** a 5-point probe within 5 u of terrain with no crash gives the `closeCall` event (2 s cooldown).
- **GPWS:** over land, a predicted gap at 0.8 s < 4 u while sinking faster than 8 u/s sets `pullUp`. Over the sea it warns only when the contact would be a crash (vz < 0.8·SEA_CRASH_VZ or pitch < 0.8·SEA_CRASH_PITCH); a gentle descent onto the water is a safe skim touch and stays silent. The HUD flashes "PULL UP" and plays a 1000 Hz beep every 0.16 s.

**Clouds:** `enteredCloud`, `inCloud` and `exitedCloud` come from `World::insideCloud(pos)`.

---

## 11. Game (group D, `game.cpp`, `course.cpp`)

### 11.1 States

```
BOOT (world gen, "GENERATING ISLANDS" drawn by the first render)
 → CALIBRATE ("HOLD COMFORTABLY": ring progress + live tilt dot; ENTER skips; 4 s timeout → stored neutral)
 → TITLE (attract autopilot over the sea, barrel roll every 9–14 s; "MOONSKIM", BEST, "ENTER RING RUN", "2 FREE FLIGHT", "H KEYS")
 → COUNTDOWN (3-2-1-GO in 1.5 s; jet assembles slice by slice; tilt re-zero if still)
 → PLAY ⇄ PAUSE (menu: Resume, Tilt, Sensitivity, Invert, Assist, Time, Volume, Brightness, Quit)
 → CRASH (2.2 s) → PLAY (respawn, altitude clears the peaks along the first 320 u of the respawn line) … → GAMEOVER (Ring Run timer 0) → ENTER: COUNTDOWN (retry) | BKSP/P: TITLE
```

**Ring Run:**
- The timer starts at 45 s. A ring adds +4 s, falling linearly to +2.4 s by ring 41 (the spacing shrinks too), and a PERFECT adds +1.5 s more. The cap is 60 s, and a crash costs −5 s. On GAME OVER, ENTER retries straight into the countdown; BKSP or P returns to the title.
- The game is over at 0.

**Free Flight:** no timer and no score pressure. Rings still spawn; the best combo is stored in NVS.

**Crash sequence:**
1. Slow motion 0.3× for 0.6 s, flash 1.0 (#FFF4E0, decaying at 4/s), shake 1.0, explosion burst (Crash or CrashSea).
2. The jet is hidden and the camera orbits the wreck.
3. Respawn 200 u back along the course at 50 u, facing the current ring, with 2 s of invulnerability (the jet blinks at 8 Hz) and the assembly animation.
4. The combo resets.

### 11.2 Rings (`Course`)

**Live rings:** always 3. The current ring is amber and pulsing (3 Hz), the next is cyan at 60 % dither, and the far one is a beacon dot plus a 6-dot light column.

**Next-ring placement:**
- Distance lerp(300, 210, d), with d = min(1, passed/40).
- |Δheading| uniform in [0.26, 0.70] rad at d = 0, widening to [0.44, 0.96] at d = 1. The sign alternates with p = 0.65.
- Altitude class:
  - LOW, 10–20 u above the surface: 50 %
  - MID, 25–65 u: 35 %
  - HIGH, 128–145 u, above the cloud top: 15 %, only from ring 6
- LOW rings try 4 Δheading candidates and keep the one with land 40–90 u from the centre but none inside.

**Validity:**
- Ground + 8 under every rim point (8 samples); otherwise raise the ring.
- Retry up to 6 times, then accept a MID ring over water.

**Geometry and pass test:**
- Radius 20 shrinking to 14 by ring 40; PERFECT within 6.5 u.
- Pass = the segment prevPos→pos crosses the ring plane: ≤ R is THROUGH, ≤ 6.5 is PERFECT, otherwise MISSED.
- Also MISSED: more than 520 u away, or 25 s without reaching the ring. After a miss, the next ring is placed relative to the jet's current pose.

**Magnet assist** (current ring within 200 u and inside a ±35° cone):
- turnAdd = clamp(1.2·bearingErr, ±0.25)·gain·(1 − |playerTurn|)
- climbAdd = clamp(0.016·altErr, ±0.15)·gain·(1 − |playerClimb|)
- gain is Easy 1.0, Normal 0.6 (default), Pro 0.

**Drawing:**
- 20 rim points as 3D lines via `drawLine3D`: 2 px thick when m·R > 10 px, depth-tested and depth-writing. The near arc correctly occludes the jet.
- The top-left half uses `RING_GOLD`, the bottom-right `RING_GOLD_DARK`. At night there is a `RING_GLOW` Bayer halo.
- Pass animation: scale 1 → 1.6 over 0.3 s, fading, plus `Burst::RingPass`.

**Guide:** if the current ring is off-screen or behind, a 5 px chevron sits at the screen edge on the bearing, with ▲/▼ when |Δalt| > 30 u.

### 11.3 Scoring and juice

**Combo:** ×1..×8. +1 per ring, +2 per PERFECT. It resets on a miss or crash.

| Event | Points | Juice |
|---|---|---|
| Ring | 100·c | Shake 0.15, Chime with pitch 2^(c/12), sparks |
| PERFECT | +50·c | Hit-stop 0.06 s at 0.5×, flash 0.35 cool white (#E8F4FF), Perfect sfx |
| ROLL! (roll active at the pass) | +100·c | |
| LOW! (a LOW-kind ring, or any ring passed below 20 u) | +50·c | LOW ring centres sit at ≥ surface + R + 3, so the altitude test alone would miss centred passes |
| BOOST! (boost > 0.5 at the pass) | +25·c | |
| SKIM (below 8 u over water) | +10·c every 0.1 s | Boost regenerates 3.5× faster |
| CLOSE! | +25·c | 2 s cooldown |
| LOOP! (M6) | +200·c | |

- Popups: up to 4, rising 6 px over 0.8 s.
- Boost ignition: shake 0.25 plus `Burst::Ignite`.
- Cloud entry: shake 0.2, `CloudEntry` vapour, Whoosh.
- In cloud: mist 0.55–0.75 with ±0.08 flicker, via `Fx::drawMist` plus a palette mist of 0.3.

**Persistence:**
- On game over, if the score beats the best: NEW BEST, a chime, and a save.
- Settings changes are saved 2 s after the last change.
- Nothing is ever saved mid-flight except through the pause menu.

**Autopilot** (title, `a` serial, bench):
- Steers to the current ring with gain 1.6 on the bearing error, holds the ring altitude, boosts on straights.
- Rolls every 9–14 s.
- On the title screen it flies with no rings, toward the moon and then wandering.

### 11.4 Render order (`Game::render`)

```
prof ST_PAL:   pal.setFx(flash/mist/fade); pal.finalize()
               memset(px, EMPTY), memset(depth, 255)
prof ST_WORLD: skyMoonScreen → SeaFx; renderWorld
prof ST_SKY:   renderSky
prof ST_OBJ:   course.draw; beacons (World::beaconsNear → emissive 1 px + glow at night)
prof ST_JET:   renderJetShadow; renderJet (unless hidden)
prof ST_FX:    fx.draw; fx.drawMist (if in cloud)
prof ST_HUD:   drawHud
```

---

## 12. Palette and colours (group B, `palette.cpp`)

- The index map is fixed by `palette.h`.
- Night and day are authored tables. Sunset is graded from day with r' = 0.95r + 0.10g + 12, g' = 0.72g + 0.05r + 4, b' = 0.70b + 0.08r + 10, except the sky, haze, sun and foam entries, which are authored.
- Vividness (saturation ×1.10, contrast ×1.06) is baked into all tables at `init()`.
- **Fog:** world(f, b) = lerp(base b, HAZE, FOG_T[f]), with FOG_T = {0, .12, .25, .38, .52, .66, .80, .92}.
- **Day shadows:** SH_* = base × (0.62, 0.67, 0.80). Night shadows are authored.
- `finalize()` applies:
  - flash: lerp toward flashRgb
  - mist: lerp toward MIST
  - fade: toward black
  - HUD entries 70..75 are exempt
- It then converts every entry to `lut2x` (`(c<<16)|c`, c = `rgb565sw`) and `rgb888`.
- The jet colours are authored once and used for every Tod. The night readability comes from the outline and lighting.

### World bases (night | day)

| Base | Night | Day | Base | Night | Day |
|---|---|---|---|---|---|
| SEA_DEEP | #0B2266 | #2366B0 | TREE_TOP | #1C4A34 | #44A042 |
| SEA_MID | #0E2C78 | #2E7EC8 | TREE | #143A2C | #2C7C38 |
| SEA_SHALLOW | #134088 | #44A2D6 | TREE_SIDE | #0A2019 | #1A4E2C |
| SURF | #1E5296 | #6ACAD2 | ROCK_LIT | #3A4466 | #B0AAA0 |
| SAND | #3E4C6A | #ECD8A0 | ROCK | #2C3555 | #908A84 |
| SAND_SIDE | #2C3650 | #C8B078 | ROCK_DARK | #222A46 | #74706C |
| GRASS_LIT | #24503A | #84CA4E | ROCK_SIDE | #1A2038 | #5E5A5C |
| GRASS | #1A4030 | #5EAE36 | SNOW | #6C82C0 | #F4F8FF |
| GRASS_DARK | #13322A | #44902E | SNOW_SIDE | #3E5288 | #B8C8E2 |
| GRASS_SIDE | #10291F | #377630 | CLOUD_TOP | #6A7CB2 | #FFFFFF |
| | | | CLOUD_SIDE | #44568C | #B4C8E6 |
| | | | CLOUD_UNDER | #24315C | #8CA0C6 |

### Fixed entries (night | day; sunset = graded day unless listed)

| Entries | Night | Day | Sunset (authored) |
|---|---|---|---|
| HAZE | #1A2A58 | #BADEF4 | #F6B282 |
| SKY0..7 (horizon → zenith) | #2A3C76 #24346A #1E2C5E #182452 #141E48 #0F173A #0A112C #06091C | lerp #C4E6F8 → #78BCEE (0–3) → #3C8CDC (4–7) | lerp #FFC07E → #EA7680 → #52428F |
| STAR_BRIGHT / MID / DIM / WARM, MILKY | #F2F5FF #A4B2E2 #5E6A9C #FFE2B0, #1C2452 | (unused) | (unused) |
| MOON_DISC / CRATER / GLOW_IN / GLOW_OUT | #F4F1DC #C9C6B4 #4A5FA2 #2E3F78 | (unused) | sun #FFF2C4 #FFE6A8 #FFDA92 #F8B880 |
| FOAM, GLINT_A, GLINT_B, MOONPATH | #7FA8E0 #6E90E0 #D8E6FF #2A4C9A | #F2FCFF #7CC0EA #EEFAFF #8CCCF0 | #FFE2D6 #FFAA6E #FFE2AA #E89A80 |
| SH_SEA, SH_SHALLOW, SH_SAND, SH_GRASS, SH_TREE, SH_ROCK, SH_SNOW | #071640 #0B2258 #222A40 #0A1A14 #08140F #141A2C #2C3A66 | base×(0.62,0.67,0.80) | graded |
| JET_OUTLINE | #1C2650 | #0E1020 | #2A1426 |
| JET_BODY0..6 | #12161E #2A303C #474F5E #6A7486 #929DB0 #BCC5D4 #E6ECF4 | same | same |
| JET_ACCENT0..3 | #4A1014 #8E1C24 #D02E36 #FF6A62 | same | same |
| JET_CANOPY0..2 | #6A4A12 #C08A24 #FFD86A | same | same |
| JET_NOZZLE, JET_GLOW | #2E2F36 #FF9A40 | same | same |
| NAV_RED, NAV_GREEN, STROBE | #FF3030 #30FF70 #FFFFFF | same | same |
| FLAME0..5 | #FFFFF0 #FFE890 #FFB040 #FF6A1C #C0301C #5A1E2A | same | same |
| TRAIL0..2 | #DCE6FF #9EB4E8 #6680C0 | #FFFFFF #D6E8FA #AACCF0 | graded |
| SMOKE0..2 | #1A1C24 #2E323E #4A505E | #3A3C44 #5A5E68 #8A909C | graded |
| WAKE_A, WAKE_B | #3CF0D0 #1E9E9A (bioluminescent) | #F2FCFF #CFEAF6 | graded |
| RING_GOLD, RING_GOLD_DARK, RING_CYAN, RING_GLOW | #FFC83C #A0661C #5FE0FF #FFF0A0 | same | same |
| BEACON, SPEEDLINE | #FFC060 #9FB8FF | #FFC060 #FFFFFF | same as day |
| HUD_WHITE/AMBER/RED/GREEN/SHADOW/CYAN | #F0F4FF #FFB000 #FF3A30 #50FF70 #0A0C18 #5FE0FF | same | same |
| MIST, SPARK, SPRAY | #6A7CB2 #FFF0A0 #DCEBFF | #FFFFFF #FFF0A0 #FFFFFF | graded |

`EMPTY` (index 0) is shown as the HAZE colour, a safety net in case any pixel is left EMPTY.

---

## 13. platformio.ini

See Appendix A for the verbatim file. Key points [V, compiled]:
- espressif32@8.0.0 (Arduino 3.3.12).
- Board `m5stack-cardputer` with an 8 MB flash override and `default_8MB.csv`.
- `-O2` replaces `-Os`.
- `-I$PROJECT_SRC_DIR`.
- Warnings only for `src` (`build_src_flags`).
- `lib_ignore = IRremote`.
- Envs: `cardputer-adv`, `-spi80`, `-bench` and `-debug`.
- `flash_mode` and `f_flash` are intentionally not set (board defaults). The `[BOOT]` line prints the flash mode and speed.

---

## 14. Serial log format and commands (group A prints; core formats `[STAT]` and `[GAME]` via `PlatformApi::log`)

**Serial setup:**
- USB-CDC at 115200.
- `Serial.setTxTimeoutMs(0)` [V].
- Every line goes into a 4 KB software ring in front of the 2 KB HWCDC TX ring; the ring is pumped without blocking (only as far as `availableForWrite()` allows) from `logLine` and once per frame from `plat::logFlush()`.
- Periodic lines (`[STAT]`, `[IMU] ax=`, `[KBD] down=`, `[WARN] slow_frame`) may only fill 3 KB, so 1 KB stays free for event lines. A periodic line that does not fit is dropped (`drop=`).
- An event line that does not fit waits for room (at most 30 ms, `vTaskDelay`) while a host is reading (CDC connected and TX progress within 300 ms); with no host reading it is dropped and counted in `drop=` and `dropi=`. Drops while no host is attached are expected.
- Everything logged before the first `[STAT]` is kept (1.5 KB) and re-printed by serial `v` (the host usually attaches after the USB re-enumeration, too late for the boot lines). The last two `[BENCH]` result lines are re-printed by `B`.
- At most about 3 lines per second, except opt-in streams.

**Boot lines:**

```
[BOOT] fw=moonskim 0.1.0 build=<__DATE__ __TIME__> board=24 adv=1 cpu=240 flash=8MB/<mode>@<MHz> arduino=3.3.12 idf=<ver> spi=40MHz fps_cap=60
[BOOT] reset=<esp_reset_reason name> seed=0x4D4F4F4E
[MEM] stage=<boot|dma|game> free_int=<B> largest_dma=<B> min_free=<B> loop_stack_hw=<B>
[NVS] loaded best=<n> vol=110 bri=150 tilt=1 rs=1 ps=1
[DISP] rot=0 bri=150 rgb_dma=ok(64800)|band(2x7680) spi=40MHz w=240 h=135
[IMU] ok=1 type=<getType()> | [KBD] reader=TCA8418 | [SPK] ok=1 vol=110 core=0
[INPUT] task=1 core=0 prio=3
[DISP] present_task ok=1 core=0 prio=5 mode=halves|bands top_us=<wire time> bot_us=<wire time>
[INIT] palette=<ms> world=<ms> world_crc=<hex8> islands=<n> beacons=<n> jet=<ms> vox=<n> stars=240
```

Measured on the device (cardputer-adv, final integration build):

```
[BOOT] fw=moonskim 0.1.0 build=Nov 23 2026 17:07:50 board=24 adv=1 cpu=240 flash=8MB/QIO@80MHz arduino=3.3.12 idf=v5.5.5 spi=40MHz fps_cap=60
[MEM] stage=boot free_int=127752 largest_dma=81908 min_free=124700 loop_stack_hw=9008
[MEM] stage=dma free_int=62212 largest_dma=31732 min_free=62204 loop_stack_hw=9008
[DISP] rot=0 bri=150 rgb_dma=ok(64800) spi=40MHz w=240 h=135
[IMU] ok=1 type=6
[DISP] present_task ok=1 core=0 prio=5 mode=halves top_us=6528 bot_us=6432
[INIT] palette=0.50 world=333.0 world_crc=349EEF88 islands=475 beacons=19 jet=28.99 vox=914 stars=240
[MEM] stage=game free_int=52580 largest_dma=31732 min_free=42592 loop_stack_hw=9008
```

**Every second:**

```
[STAT] t=30 st=PLAY fps=60.0 ft=16.7/16.9 c1=8.41/8.63 in=0.03 sim=0.84 pal=0.06 wld=3.52 sky=0.56 obj=0.51 jet=1.95 fx=0.64 hud=0.25 aud=0.03 exp=0.03 dmaw=0.00 | smp=2341 stp=111 fill=5135 vox=826 vw=1667 fl=410 prt=17 lp=428 | heap=53964 min=44176 drop=0 dropi=0 bat=4152mV/100% | p0=0.41 sp0=0.08 lat=13.4 dw0=0.02 in0=1.12 ist=0 pto=0
```
- `ft` is the average and max frame time (ms, measured end to end, so a frame whose work grew by 2 ms reads ~18.7 ms although it made its 60 fps slot). `c1` is the average and max core-1 work (ms, excluding dmaw).
- The middle section is per-stage ms; then the work counters; then heap, drops and battery.
- Core-0 section: `p0` present-task CPU per frame (expand2x + push calls), `sp0` its dmaBusy spin per frame, `lat` hand-off to last pixel on the wire (ms), `dw0` present task waiting for the previous push (display-bound), `in0` input-task time per I2C poll, `ist` frames that reused the previous input snapshot, `pto` present hand-off timeouts + bus hangs (must stay 0).

**IMU stream** (20 Hz while on; `g` key or serial `i`):

```
[IMU] ax=-0.012 ay=0.574 az=0.818 gx=0.31 gy=-1.20 gz=0.08 fresh=1 roll=-0.8 pitch=1.3 turn=0.00 climb=0.00 up=(0.004,0.571,0.821) n=(0.004,0.571,0.821) rs=1 ps=1 cal=1
```

**Events and problems:**

```
[CAL] ok n=(0.004,0.571,0.821) bias=(0.6,-0.9,0.2) still_ms=520 | [CAL] timeout using_stored
[GAME] state TITLE->COUNTDOWN | [GAME] ring n=12 kind=PERFECT score=4300 combo=5 time=23.4 alt=18.2
[GAME] crash cause=sea alt=2.1 vz=-24.3 pitch=-0.41 pos=(3120,880) | [GAME] over score=8450 best=9100 rings=31 new=0
[KBD] down=SPACE,SEMI kp=55,39        (serial 'k' toggles)
[WARN] slow_frame ms=24.8 stage=wld steps=231   (any frame > 20 ms, at most 1/s)
[WARN] board_not_adv | imu_reject | dma_fallback
[ERR] dma_alloc size=64800 largest=<B> | imu_stale ms=210 | nvs_open | jet_build
[SHOT] begin w=120 h=68 fmt=idx8+pal888 ... base64 lines (96 chars) ... [SHOT] end crc=<hex8>
[BENCH] perframe frames=1201 fps_min=59.9 avg=60.0 start_ms=26.5 ft_max=18.2 over17=10 over22=0 late=1 c1_avg=8.07 c1_max=9.99 dmaw_avg=0.00 wld_max=4.51 jet_max=2.04
[BENCH] core frames=1200 fps_min=60.0 fps_avg=60.0
    (perframe, main.cpp: start_ms = the first frame (startRun: spawn search + ring layout, a
     scene cut, excluded from the rest); over17 / over22 = frames longer than 17 / 22.2 ms end
     to end; late = frames that really missed their 60 fps slot (pace() found the deadline
     passed); c1 / wld / jet maxima are per frame. core, game.cpp: the 1 s windows.)
```

**Serial commands** (one character):

| Char | Action | Handled by |
|---|---|---|
| `?` | Help | platform |
| `s` | `[STAT]` on/off | platform |
| `k` | Key echo on/off | platform |
| `h` | Heap detail plus loop stack high-water, DMA heap, present / input task stack free, queued log bytes | platform |
| `v` | Re-print the boot transcript (every line before the first `[STAT]`) | platform |
| `B` | Re-print the last two `[BENCH]` result lines | platform |
| `g` | Grab screenshot (`[SHOT]`) | platform |
| `i` | IMU stream on/off | core |
| `r` / `p` | Flip roll / pitch sign (saved) | core |
| `c` | Recalibrate | core |
| `t` | Tilt on/off | core |
| `n` | Time of day | core |
| `a` | Autopilot on/off | core |
| `b` | 20 s autopilot bench, then `[BENCH]` | core |
| `x` | Force crash and respawn | core |
| `X` | Reset best | core |
| `f` | Stats overlay | core |
| `m` | Jet model ASCII dump | core |

`tools/grab_shot.py COMx out.png` sends `g`, parses `[SHOT]` and writes a 240×135 PNG (2× nearest). Run it with `%USERPROFILE%\.platformio\penv\Scripts\python.exe` (it has pyserial).

---

## 15. HUD (group D, 3×5 font, logical pixels, 1 px `HUD_SHADOW`)

**During play:**
- Top-left: score, with combo `x5` below in amber, pulsing on change.
- Top-right: timer `34.2` at 2× (the only fail-state readout), red and blinking below 5 s (Ring Run only). The low-battery icon moves below it.
- Bottom-left: boost bar 24×2 (amber, blinks when empty) and `SPD 136`.
- Bottom-right: `ALT 52`, orange "SKIM" below 8 u, and the tilt badge: green T, amber T needing calibration, grey K, red ? for guard.
- Battery icon when below 15 %.
- Centre: popups, the "PULL UP" blink (4 Hz, red), the ring guide chevron.
- `f`: `60 4.1ms` at top-centre.

**Screens:**
- Calibrate: ring progress, live dot, "HOLD COMFORTABLY".
- Title: "MOONSKIM" at 2×, BEST, mode hints.
- Countdown: digits at 2×.
- Pause: 9-item menu on a 50 % dithered panel, plus battery %.
- Help: keymap summary.
- Game over: score, best, rings, best combo, NEW BEST blink.

---

## 16. Audio (group D synth, group A pump)

**Synth** (16 kHz mono int16, parameters ramped per block):
- Engine: two detuned saws at 140 + 80·speedRatio + 90·boost Hz (+0.7 %), through a one-pole low-pass at 700 + 1500·boost Hz, gain 0.18.
- Roar: noise band-passed around 1.5 kHz, gain 0.08 + 0.30·boost.
- Wind: noise band-passed around 3 kHz, gain 0.12·min(1.6, speedRatio)².
- Spray: high-passed noise × wake strength, with random 30–60 ms bursts.
- Cloud: the low-pass opens while inCloud.
- 4 one-shot voices (see `audio.h`).
- Soft clip x/(1 + |x|).
- The 8 Ω speaker gives little below 300 Hz, so nothing important sits there.

**Pump:**
- Call `M5.Speaker.isPlaying(0)`; while it is below 2, render the next of 3 rotating 384-sample buffers and call `playRaw(buf, 384, 16000, false, 1, 0, false)`.
- The buffer being written is never queued or playing.
- A hitch longer than 48 ms gives silence, never garbage.
- `JET_AUDIO=0` compiles the pump out.

**Speaker setup** (in `setup()`, before the game init):

```cpp
auto sc = M5.Speaker.config(); sc.task_pinned_core = 0; sc.task_priority = 2; M5.Speaker.config(sc);
M5.Speaker.begin(); M5.Speaker.setVolume(persist.volume);
```

Mute is `setVolume(0)`.

---

## 17. Platform layer details (group A)

**`setup()`** (`SET_LOOP_TASK_STACK_SIZE(12*1024)` in `main.cpp`):
1. `auto cfg = M5.config(); cfg.serial_baudrate = 115200; cfg.fallback_board = m5::board_t::board_M5CardputerADV; cfg.internal_mic = false; M5Cardputer.begin(cfg, true);`
2. `Serial.setTxTimeoutMs(0);` then the `[BOOT]` lines and `[MEM] stage=boot`.
3. **DMA first:** `heap_caps_aligned_alloc(4, 64800, MALLOC_CAP_DMA | MALLOC_CAP_INTERNAL)`. On failure, allocate 2 band buffers and log `[ERR]` / `[WARN] dma_fallback`. Then `[MEM] stage=dma`.
4. Display: landscape rotation. With M5GFX 0.3.0 the Cardputer(-ADV) panel is described as 135×240 with `offset_rotation = 1`, so landscape is `setRotation(0)`; the code tries 0, then 1, and halts with `[ERR] disp_size` if neither reports 240×135. Then `fillScreen(TFT_BLACK)`, brightness from NVS.
5. If `JET_SPI_MHZ == 80`:

   ```cpp
   auto* bus = static_cast<lgfx::Bus_SPI*>(M5.Display.getPanel()->getBus());
   auto c = bus->config(); c.freq_write = 80000000; bus->config(c); bus->init();
   ```

   This is compile-verified. If the panel shows corruption, use the 40 MHz env.
6. Speaker (§16).
7. NVS: `Preferences.begin("moonskim")`, `getBytes("p")`. Use defaults if the size or version mismatches.
8. `game.init(api, persist, 0x4D4F4F4E, imuPresent())`, then `[INIT]` and `[MEM] stage=game`.
9. `M5.Display.startWrite();` (never ended).

**Input (core-0 input task, `plat_input.cpp`):**
- After setup the input task (core 0, prio 3) is the only I2C user. `pace()` kicks it when ≤ 2.6 ms of the frame slot remain (immediately when the frame is late or the build is uncapped); one poll costs ≈ 1.1 ms (`in0=`), mostly the BMI270 read (1 status + 20 data bytes at 400 kHz).
- Keyboard: the TCA8418 is read only when its INT flag is set; positions kp = y·14 + x are OR'd into `seen`, keysDown is the final list.
- BtnA via `M5.update()` (presses OR'd), IMU via `M5.Imu.update()` / `getImuData()` (remapped; `fresh` = a new sample arrived since the last `pollInput`), battery every 2 s.
- `pollInput` (core 1) only copies a spinlocked snapshot: keys are the OR of everything seen since the last call, so no tap is lost. In paced frames the sample is 1.5–2.6 ms old at frame start; in a late frame at most one frame old (`ist=` counts those).

**Pacing (`main.cpp`):** described in §3. In the bench env (`JET_FPS_CAP == 0`), call `vTaskDelay(1)` every 16 frames to keep the core-1 idle task and watchdog happy.

---

## 18. PC simulator (group E, `tools/sim`)

**Build** (`tools/sim/build.bat`):

```
call "C:\Program Files (x86)\Microsoft Visual Studio\2022\BuildTools\VC\Auxiliary\Build\vcvars64.bat" >nul
cl /nologo /std:c++17 /O2 /EHsc /W4 /DJET_SIM=1 /I ..\..\src\core ..\..\src\core\*.cpp sim_main.cpp script.cpp synth_imu.cpp png_write.cpp wav_write.cpp /Fe:out\jetsim.exe
```

**Run:**
- `jetsim <script.jsc> [--out dir] [--scale 1|2|4] [--seed N] [--wav] [--model] [--selftest] [--hash] [--bench N]`
- Fixed dt 1/60. Deterministic.

**Platform implementation in the sim:**
- `PlatformApi.micros` uses QPC.
- `log` writes to stdout (same tags as the device).
- `save` writes `out/persist.ini`.
- The profiler records PC µs. `ST_INPUT`, `ST_EXPAND` and `ST_AUDIO` are measured around the sim's own equivalents.

**Script grammar** (`<t_seconds> <cmd> [args]`, `#` comments, commands applied when the sim time reaches t):

| Command | Meaning |
|---|---|
| `seed N` (t = 0) | world seed |
| `tod night\|sunset\|day` | time of day |
| `view 0\|1\|2` | camera preset |
| `state title\|ring\|free` | ring and free call `debugStart`, skipping calibration (synthetic level IMU) |
| `key NAME down\|up`, `tap NAME`, `hold NAME secs` | keyboard, by `KEY_NAMES` |
| `tilt rollDeg pitchDeg [rampSec]` | synthetic IMU relative to the hold pose |
| `teleport x y z yawDeg` | `debugTeleport` |
| `autopilot on\|off` | |
| `shot NAME` | expand2x and the real LUT → `out/<script>/<NAME>.png` at `--scale` (default 2 → 480×270) |
| `sheet NAME cols` | contact sheet of the shots since the last sheet |
| `assert VAR OP VALUE` | `debugVar` names plus `empty_px` (EMPTY count after render) and `fps_est`. Exit code 1 on failure. |
| `bench FRAMES` | |
| `end` | |

**Synthetic IMU (`synth_imu.cpp`):**
- Device orientation = hold pose (pitched 35° toward the face) × script roll/pitch.
- accel = R⁻¹·(0, 0, 1) + N(0, 0.01 g).
- gyro = the finite-difference body rate (deg/s) + N(0, 0.5) + bias (0.8, −0.5, 0.3) dps.
- Signs follow the M5 doc axes, so `tilt_cal.jsc` verifies the core mapping end to end. The device check (§22) still decides the real signs.

**PNG writer:**
- Signature, IHDR (8-bit RGB), IDAT (zlib 0x78 0x01, stored deflate blocks ≤ 65,535 B, Adler-32), IEND. CRC-32 is table-driven. About 120 lines.
- Pixels: device-exact path px → `expand2x` → swapped 565 → `rgb565swToRgb888`.

**Output per run:**
- Per-stage PC ms.
- Counters per frame.
- `esp_est_ms` from `CostModel` (default constants in `perf.h`, refit from device `[STAT]`).
- `world_crc`.
- With `--hash`: FNV-1a per shot (determinism and regression).

**Other modes:**
- `--model` runs `JetModel::dumpAscii`.
- `--selftest` runs:
  - expand2x on a test pattern (row 134 = logical 67, byte order)
  - a PNG round-trip size check
  - tilt curve unit checks
  - `depthQ` monotonicity

**Standard scripts (`tools/sim/scripts/`):**

| Script | Contents |
|---|---|
| `night_cruise.jsc` | |
| `bank_left_right.jsc` | |
| `barrel_roll.jsc` | 8-frame sheet |
| `boost.jsc` | |
| `skim_wake.jsc` | |
| `island_canyon.jsc` | |
| `climb_through_clouds.jsc` | under → in → above |
| `steep_climb_stars.jsc` | |
| `ring_run.jsc` | autopilot 60 s |
| `crash_island.jsc` | |
| `crash_sea.jsc` | |
| `tod_all.jsc` | |
| `hud_all.jsc` | title, calibrate, pause, help, gameover |
| `jet_turntable.jsc` | yaw 0..315 step 45 × roll 0 / 90 / 180 × pitch ±30 → sheet |
| `tilt_cal.jsc` | asserts |
| `perf_worst.jsc` | low among islands, boost, clouds overhead, crash particles |
| `determinism.jsc` | run with `--hash` twice |

**Optional (M6):** `--live` Win32 GDI window, keyboard plus mouse-as-tilt, for tuning the feel.

---

## 19. Frame budget (core 1, 240 MHz, 120×68, SPI at 40 MHz) [measured]

Device measurements (cardputer-adv, Ring Run autopilot bench `b`, night and day; per-stage
averages of the 1 s `[STAT]` windows, maxima per frame from `[BENCH] perframe`). "Before" is
the build measured at the start of the optimisation pass (frame ≈ c1 + dmaw, since the second
display half was only started at the beginning of the next frame).

| Stage | Before (ms) | Now avg (ms) | Now max (ms) | Notes |
|---|---|---|---|---|
| in | 1.09 | 0.03 | 0.05 | I2C poll moved to the core-0 input task (`in0` ≈ 1.1 ms there) |
| sim | 1.07–1.10 | 0.78 | 0.9 | + ≈ 12 ms once at run start (spawn search, a scene cut) |
| pal | 0.09 | 0.06 | 0.13 | |
| wld | 2.69–5.56 | 3.5 | 4.7 | ≈ 110 march steps; slow-frame warnings gone |
| sky | 1.31–1.67 | 0.45–0.65 | 0.8 | |
| obj | 0.71 | 0.41 | 0.6 | |
| jet | 2.14–2.40 | 1.65 | 2.1 | incl. shadow (≈ 0.45 below 60 u) |
| fx | 0.46–0.51 | 0.44 | 0.8 | crash explosion ≈ 0.8 |
| hud | 0.43 | 0.25–0.33 | 0.7 | title / menus ≈ 0.65 |
| aud | 0.48–0.59 | 0.04 | 0.1 | |
| exp | 0.37 | 0.03 | 0.03 | expand2x now on core 0 (`p0` ≈ 0.4 ms) |
| **c1** | **11.45–13.89** | **≈ 7.4–8.3** | **≈ 10.2** | |
| dmaw | 6.43–6.45 | 0.00 | | display push fully overlapped (§3) |
| **fps** | **48.9–55.5** | **60.0 locked** | min window 59.6–59.9 | late frames: only the one after a run start; uncapped (bench env) 74.1 fps, display-bound |

- **Result:** locked 60 fps in normal play with ≈ 6 ms of core-1 headroom (60 fps holds while c1 ≤ ≈ 16.5 ms); no `[WARN] slow_frame` in normal flight. The only slow frames are the first frame of a run / title flyover (≈ 25–30 ms, `stage=sim`, the spawn search in `World::findSpawn`), which coincide with a scene cut.
- **What it took** (on top of the owners' work): the framework compiles with `-fno-builtin-memcpy -fno-builtin-memset`, so every 4-byte `memcpy` used as a bit-cast in the world and sky fills was a ROM call (~25 per chunk call in the march, one per word in the sky bands); they are aligned word accesses now (wld −1.2 ms, sky −0.5 ms). Instruction-count models under-estimate this core: real code runs at ~2–3 cycles per instruction (taken branches ~3 cycles, load-use stalls), measured with `rsr.ccount` microbenchmarks. `-fno-stack-protector` was tried and made the march ~0.6 ms slower (inlining changes), so the framework's stack protector stays.
- **Cost model:** `perf.h::CostModel` and the sim's `JET_C_*` / `ESP_FIXED_MS` are fitted to 71 device windows: the sim's ESP32 estimate for its bench is c1 ≈ 7.95 ms against 7.4–8.3 measured (wld 3.50 vs ≈ 3.5, jet 1.74 vs ≈ 1.7).
- **Escape hatches,** in order, if a scene measures slower:
  1. dv ×1.4 (fewer march steps; each step costs ≈ 20 µs on the device)
  2. glints and surf only within 150 u
  3. cloud samples on every other step
  4. viewDist ×0.8
  5. particle cap 64
  6. the 80 MHz env (halves the display period to ≈ 6.7 ms; only matters if the frame becomes display-bound, which at 60 fps it is not)

---

## 20. Memory plan (internal SRAM; no PSRAM) [E except static sizes]

| Item | Bytes | Where |
|---|---|---|
| RGB565 DMA frame | 64,800 | heap `MALLOC_CAP_DMA` (or 2 × 7,680 band fallback) |
| Island stamps 12 × 4,096 | 49,152 | static (world.cpp) |
| Stamp mips 12 × 1,024 | 12,288 | static |
| Macro table | 1,024 | static |
| Cloud map 128² | 16,384 | static |
| noise64 (64²) | 4,096 | static (render_world.cpp) |
| px + depth | 16,320 | static (game.cpp) |
| Palette (3 × 1 KB base + LUT 1 KB + rgb 768 + shade 256) | 5,120 | in Game |
| Jet voxels 1,400 × 6 + silhouette | 8,536 | in Game |
| Jet build temp grid | (9,724) | heap, freed during init |
| Fx (particles 128 × 40, trails, wake, speed lines) | 7,900 | in Game |
| Stars 240 × 4 + dust 300 × 3 | 1,860 | static (render_sky.cpp) |
| March tables (colY, rowNow/Prev, cloud rows, colDone) | 600 | static |
| Audio stream 3 × 384 × 2 | 2,304 | static (plat_audio.cpp) |
| Present staging (px + lut2x) | 9,184 | static (plat_display.cpp) |
| Log ring + boot transcript + bench lines | 6,144 | static (plat_log.cpp) |
| Present + input task stacks | 8,192 | static (plat_display.cpp / plat_input.cpp) |
| Jet render keys, stamps, tables | ≈ 9,000 | static (jet_model.cpp, render_jet.cpp) |
| World block tables, sky stencils | ≈ 2,600 | static (world.cpp, render_sky.cpp) |
| Game, course, tilt, HUD state | about 3,000 | in Game |
| **Total ours** | **≈ 193 KB** | |
| Arduino + M5 libs + USB-CDC + speaker task + loop stack 12 KB | ≈ 70–90 KB | static RAM of a bare build is 26.7 KB [V] |
| **Headroom of the 327,680 B** | **≈ 45–65 KB** | |

**Measured (final integration build):** static RAM 200,988 B (61.3 % of 327,680). `[MEM]` free internal heap: 127,752 B at boot, 62,212 B after the DMA frame, 52,580 B after game init, minimum 42,592 B, stable in play. IRAM code (`.iram0.text`) is ≈ 101 KB (`JET_HOT` render paths plus group D's `JET_D_IRAM=2` set of 14 KB), which comes out of the same SRAM; `-DJET_D_IRAM=1` (or 0) returns ≈ 6 KB (14 KB) of heap if a later feature needs it.

**Rules:**
- The large arrays are static, so the linker reports overflow at build time.
- The DMA buffer is the only big heap block and is allocated first.
- `[MEM]` is logged at boot, after DMA, after game init, and in every `[STAT]` (heap / min).

**If tight, in order:**
1. Clouds as nibbles (−8 KB).
2. Drop the mips and use a step cap instead (−12 KB).
3. Hash instead of noise64 (−4 KB).
4. The band DMA path permanently (−49 KB).

---

## 21. Acceptance criteria per group

Timing in the sim is not representative of the device, but every run must print the per-stage PC ms, the counters and `esp_est_ms`.

**Group A (platform, build):**
- All 4 PlatformIO envs build with zero warnings from `src/`.
- `expand2x` passes `--selftest`.
- On device:
  - The `[BOOT]`, `[MEM]`, `[DISP]`, `[IMU]`, `[KBD]` and `[SPK]` lines appear.
  - `[STAT]` once per second with fps ≥ 58 average in night cruise and `dmaw` > 0 (proves overlap).
  - No watchdog reset or `[WARN] slow_frame` burst in 10 minutes.
  - `heap min` stable.
  - The key echo matches `KEY_NAMES` for all 56 keys.
  - NVS survives a power cycle.
  - `grab_shot.py` produces a PNG matching the sim's framing.
  - With USB unplugged (battery), the game runs at full speed.

**Group B (world, sky, palette):**
- `world_crc` is identical across two sim runs and equal to the device's.
- `assert empty_px == 0` after every render.

| Sim shot | Must show |
|---|---|
| `night_cruise` | Horizon at row 25 ± 3 (logical) at cruise. 8 dithered sky bands. ≥ 40 stars with visible twinkle between two shots 0.5 s apart. Milky Way band. Moon with craters and halo. Glitter path below the moon. Terraced islands with lit and dark faces and side walls. Surf foam that changes between frames. Fog fading islands toward the horizon. |
| `climb_through_clouds` | Undersides when below, mist when inside, tops with dithered rims and walls when above. |
| `tod_all` | Day and sunset palettes (sun disc at sunset, no stars by day). |
| `island_canyon` | Terrain occluding terrain correctly (no see-through walls). |

- Counters in `perf_worst`: steps ≤ 180, samples ≤ 25k; `esp_est` world ≤ 3.5 ms.

**Group C (jet, effects):**
- `--model` reports 900–1,300 voxels; the ASCII top view shows one fin, a cranked delta, canards and intakes.
- `jet_turntable` sheet: no pinholes in the body at any pose; continuous outline; red nav light on the jet's left and green on its right; gold canopy; when banked, the raised wing is lighter than the lowered one.
- `boost`: flame about 6 u at cruise and about 20 u at boost, with shock dots and a white core.
- `skim_wake`: shadow on the sea under the jet that narrows when banked; wake V; bioluminescent wake at night; spray.
- `bank_left_right`: trails appear in hard turns and fade when level.
- `crash_*`: fire, debris, smoke and the sea spray column.
- `esp_est` jet ≤ 0.8 ms and fx ≤ 0.9 ms in `perf_worst`.

**Group D (game, flight, tilt, HUD, audio):**
- `tilt_cal` asserts:
  - roll +15° → turn > 0.4
  - roll −15° → turn < −0.4
  - roll 2° → turn == 0
  - top toward the face (pitch −10°) → climb > 0
  - invert flips it
  - recalibrating at a new hold pose zeroes the output
- `ring_run`: autopilot passes ≥ 10 rings in 60 s with no crash.
- `crash_island` crashes and respawns; score and time are adjusted.
- `crash_sea`: vz −5 → skim touch (no crash), vz −20 → crash.
- Steady full turn gives yaw rate 1.30 ± 0.05 rad/s; boost speed ≈ 122 u/s.
- `hud_all` shots are legible at scale 2.
- `--wav` engine audio rises with boost and contains the chime, roll and crash.
- Persist is saved on game over with a new best.

**Group E (simulator):**
- `build.bat` builds `/W4` clean.
- All standard scripts run and write PNGs that open in a standard viewer. A Python `zlib.decompress` of the IDAT succeeds.
- `determinism` produces identical hashes twice.
- `--bench 600` prints the timing table plus `esp_est`.
- `--selftest` passes.
- Script errors report the line number.

---

## 22. Bring-up order and hardware verification

| Milestone | Scope |
|---|---|
| M0 | A + E skeleton: platform, DMA present of a test gradient with the real palette, `[STAT]`, sim + PNG + selftest. On device: check `dmaw`, fps, heap, `largest_dma`. |
| M1 | B: world gen + march + sky. Sim shots; device `world_crc` parity. |
| M2 | C: jet model, splat, flame, outline, shadow. Turntable. |
| M3 | D: flight, camera, tilt, keys. **IMU sign check on device** (below). Feel tuning. |
| M4 | D: rings, scoring, states, HUD, NVS, crash/respawn. |
| M5 | C + D: fx, trails, wake, mist, audio. `perf_worst` within budget on device. |
| M6 | Polish: help, battery, beacons, auto loop (`q`), 80 MHz trial, `--live` sim. |

**IMU sign check** (debug env or `g`, with the `[IMU]` stream):
- Flat on a table: az ≈ +1.
- Right edge down: ax < 0, roll > 0, the dot moves right, turn > 0.
- Screen edge away and down: ay < 0, pitch > 0, climb < 0 (dive).
- Rotating clockwise seen from above: gz < 0.

If anything is wrong:
- Use serial `r` / `p` to flip a sign. It is saved.
- If axes are swapped, change `IMU_SRC_*` / `IMU_SGN_*` in `config.h` and rebuild.
- Record the result in this section.

---

## 23. Risks and mitigations

| Risk | Mitigation |
|---|---|
| IMU signs or axes differ from the doc | Runtime sign flags saved in NVS, compile-time remap, the `[IMU]` stream, the on-screen dot and the 4-pose check. |
| Board detection falls back to non-ADV (key held at boot) | `fallback_board = ADV`. Log the board. If not ADV: `[WARN]`, tilt disabled, keys still work. |
| DMA buffer allocation fails or fragments | Allocate first. Band fallback. `[ERR]` with the largest block. |
| Something calls `M5.Display` while the transaction is open | Only `plat_display.cpp` uses it; code review rule; M5 log-to-display off. |
| 80 MHz SPI instability | 40 MHz default; 80 MHz only in its own env, checked visually. |
| HWCDC stalls with no host | `setTxTimeoutMs(0)` [V], `availableForWrite` gating, ≤ 3 lines/s. |
| Keyboard FIFO lag and lost taps | 4 drains per frame, `keysPressed` accumulation, act on edges. |
| I2C contention | All I2C on core 1; ES8311 only during `Speaker.begin`. |
| World render slower than estimated | IRAM hot loop, counters + `esp_est` before flashing, escape hatches in §19. |
| Holes in the jet when rotated | 4 sub-samples + up-axis fill + 2×2 at m ≥ 1.6; turntable acceptance. |
| Sim/device float drift | Integer world gen (CRC parity). Rendering differences are tolerated. |
| Tilt fatigue or imprecision | Soft curve, 3 sensitivities, neutral drift, ring magnet, roll dash, keyboard trim, keys-only mode. |
| Unfair crashes | Skim touch, GPWS warning, EASY floor, crash costs time not the run. |
| Speaker task disturbing the frame | Pinned to core 0 [V]; synth on core 1 about 0.1 ms; `JET_AUDIO=0` fallback. |
| Flash wear | Saves debounced; only on game over or settings change. |
| Library drift | Versions pinned (espressif32 8.0.0, M5GFX/M5Unified 0.3.0, M5Cardputer tag 1.2.0). |

---

## 24. Originality and IP

- No reference code, voxel data, palette ramps, pivot or engine constants are used.
- The reference (`docs/research-ref.md`) contributes only techniques (lens-shift level camera, terraced voxel-space march, Bayer dithering, sprite-stack splatting) and public tuning numbers (easing rates, turn rate, bank angle, boost factor).
- The jet "Kestrel" is an original procedural design: single fin, cranked-arrow delta, canards, side intakes, one nozzle, silver/red/gold.
- World generation, palettes, grading, HUD, font, synth and game rules are our own.
- The following are additions not present in the reference:
  - star twinkle, the Milky Way and shooting stars
  - the moon glitter path
  - nav lights and the bioluminescent wake
  - island beacons
  - ring gates, skim scoring and the tilt control scheme

---

## Appendix A: shared contract files (verbatim copies of the files on disk)

The files under `src/` and `platformio.ini` are authoritative; this appendix is a snapshot, refreshed at the device integration pass.

### `platformio.ini`

```ini
; MOONSKIM - pixel-art night jet flyer for the M5Stack Cardputer-Adv
; OWNER: group A. See docs/DESIGN.md section 13.
;
; Envs:
;   cardputer-adv        default: 40 MHz SPI, 60 fps cap
;   cardputer-adv-spi80  80 MHz SPI overclock (verify visually on the panel before trusting)
;   cardputer-adv-bench  uncapped fps + 20 s autopilot bench at boot (prints a summary)
;   cardputer-adv-debug  IMU stream + key echo on at boot, verbose logs
;
; Upload: pio run -e cardputer-adv -t upload   (USB-Serial/JTAG auto-reset; if the port
; vanished: power OFF, hold G0, plug USB, release G0, upload, then reset)
; Monitor: pio device monitor -e cardputer-adv

[platformio]
default_envs = cardputer-adv

[env]
platform = platformio/espressif32@8.0.0
board = m5stack-cardputer
framework = arduino
board_build.f_cpu = 240000000L
board_upload.flash_size = 8MB
board_upload.maximum_size = 8388608
board_build.partitions = default_8MB.csv
upload_speed = 1500000
monitor_speed = 115200
monitor_filters = esp32_exception_decoder, time
build_type = release
build_unflags = -Os
build_flags =
  -O2
  -DARDUINO_USB_MODE=1
  -DARDUINO_USB_CDC_ON_BOOT=1
  -I$PROJECT_SRC_DIR
; warnings only for our own sources (library code would drown them)
; Note: the framework passes -fno-builtin-memcpy/-memset, so a small memcpy/memset is a real ROM
; call here; hot code uses aligned word access instead (see render_world.cpp / render_sky.cpp).
; (-fno-stack-protector was tried: it made renderWorld's chunk loop ~0.6 ms SLOWER through
; different inlining, so the framework's -fstack-protector stays.)
build_src_flags =
  -Wall
  -Wdouble-promotion
lib_deps =
  m5stack/M5GFX@0.3.0
  m5stack/M5Unified@0.3.0
  https://github.com/m5stack/M5Cardputer.git#1.2.0
lib_ignore = IRremote

[env:cardputer-adv]

[env:cardputer-adv-spi80]
build_flags = ${env.build_flags} -DJET_SPI_MHZ=80

[env:cardputer-adv-bench]
build_flags = ${env.build_flags} -DJET_FPS_CAP=0 -DJET_BOOT_BENCH=1

[env:cardputer-adv-debug]
build_flags = ${env.build_flags} -DJET_DEBUG=1
```

### `src/core/compat.h`

```cpp
// compat.h - portability macros for the portable core (src/core).
// OWNER: group A. SHARED CONTRACT - change only by agreement of all groups.
// Rule: nothing in src/core may include Arduino.h, M5*.h or any ESP-IDF header
// except through this file.
#pragma once
#include <stdint.h>
#include <stddef.h>

#if defined(ESP_PLATFORM) || defined(ARDUINO_ARCH_ESP32)
  #include "esp_attr.h"
  // Put on the DEFINITION (in the .cpp) of hot functions only, never on inline
  // header functions. Hot set: renderWorld, renderJet, expand2x, drawLine.
  #define JET_HOT IRAM_ATTR
  #define JET_DEVICE 1
#else
  #define JET_HOT
  #define JET_DEVICE 0
#endif

#if defined(__GNUC__) || defined(__clang__)
  #define JET_LIKELY(x)   __builtin_expect(!!(x), 1)
  #define JET_UNLIKELY(x) __builtin_expect(!!(x), 0)
  #define JET_NOINLINE    __attribute__((noinline))
#else
  #define JET_LIKELY(x)   (x)
  #define JET_UNLIKELY(x) (x)
  #define JET_NOINLINE    __declspec(noinline)
#endif

#ifndef JET_SIM
  #define JET_SIM 0
#endif

namespace kj {

inline uint16_t bswap16(uint16_t v) { return (uint16_t)((v >> 8) | (v << 8)); }

// RGB888 (0xRRGGBB) -> RGB565, NOT swapped.
inline uint16_t rgb565(uint32_t rgb) {
  uint32_t r = (rgb >> 16) & 0xFF, g = (rgb >> 8) & 0xFF, b = rgb & 0xFF;
  return (uint16_t)(((r & 0xF8) << 8) | ((g & 0xFC) << 3) | (b >> 3));
}
// RGB888 -> RGB565 byte-swapped (big-endian in memory) = what the ST7789 DMA path wants.
inline uint16_t rgb565sw(uint32_t rgb) { return bswap16(rgb565(rgb)); }

// Inverse used by the simulator: byte-swapped 565 -> RGB888 with bit replication.
inline uint32_t rgb565swToRgb888(uint16_t sw) {
  uint16_t c = bswap16(sw);
  uint32_t r = (c >> 11) & 31, g = (c >> 5) & 63, b = c & 31;
  r = (r << 3) | (r >> 2); g = (g << 2) | (g >> 4); b = (b << 3) | (b >> 2);
  return (r << 16) | (g << 8) | b;
}

}  // namespace kj
```

### `src/core/config.h`

```cpp
// config.h - EVERY tuning constant of MOONSKIM lives here.
// OWNER: shared. Each block names the group that tunes it; a group may change the
// VALUES of its own block freely, but adding/removing/renaming a constant used by
// another group needs agreement. Units: u = world unit (= 1 jet voxel), s, rad, px =
// LOGICAL pixel (120x68 frame) unless stated otherwise.
#pragma once
#include <stdint.h>

// ---------------------------------------------------------------- build switches
#ifndef JET_SPI_MHZ
#define JET_SPI_MHZ 40          // 40 (safe, default) or 80 (env cardputer-adv-spi80)
#endif
#ifndef JET_FPS_CAP
#define JET_FPS_CAP 60          // 0 = uncapped (bench env)
#endif
#ifndef JET_BOOT_BENCH
#define JET_BOOT_BENCH 0        // 1 = run the 20 s autopilot bench right after boot
#endif
#ifndef JET_AUDIO
#define JET_AUDIO 1             // 0 = silent build (no Speaker use at all)
#endif
#ifndef JET_DEBUG
#define JET_DEBUG 0             // 1 = IMU stream + key echo on at boot
#endif
#define JET_FW_NAME    "moonskim"
#define JET_FW_VERSION "0.1.0"

namespace kj {
namespace cfg {

// ================================================================ screen (A)
constexpr int SCREEN_W = 240, SCREEN_H = 135;  // panel, landscape (M5GFX 0.3.0 rotation 0)
constexpr int LW = 120, LH = 68;               // logical frame, 2x nearest expand
constexpr int LPIX = LW * LH;                  // 8160
constexpr uint8_t BRIGHT_DEFAULT = 150, BRIGHT_STEP = 24, BRIGHT_MIN = 24;
constexpr uint32_t FRAME_US = 16667;           // 60 fps pacing period
constexpr float DT_MAX = 0.05f;                // frame dt clamp
constexpr float SIM_STEP = 1.0f / 120.0f;      // fixed sub-step for flight
constexpr int   SIM_MAX_SUBSTEPS = 6;

// ================================================================ world data (B)
constexpr float CELL_U = 4.0f;          // world units per terrain cell
constexpr int   CELL_SHIFT = 2;         // log2(CELL_U)
constexpr int   WORLD_CELLS = 2048;     // world wraps every 2048 cells
constexpr float WORLD_U = 8192.0f;      // = WORLD_CELLS * CELL_U
constexpr int   STAMP_N = 64;           // island stamp = 64x64 cells (256 u)
constexpr int   STAMP_COUNT = 12;
constexpr int   MIP_N = 32;             // per-stamp 2x2 max mip (8 u cells)
constexpr int   MACRO_N = 32;           // macro table 32x32 stamps = 2048 cells
constexpr uint8_t MACRO_OCEAN = 0xFF;   // macro entry: open deep ocean
constexpr uint8_t MACRO_TRANSPOSE = 0x10;  // macro entry bit: swap lx/ly
constexpr float MACRO_ISLAND_FRACTION = 0.45f;
constexpr float STAMP_LAND_MIN = 0.30f, STAMP_LAND_MAX = 0.45f;  // land fraction per stamp
constexpr int   STAMP_BORDER = 4;       // border cells forced to DEEP
// Terrace level (cell high nibble) -> height in u. Levels 1-8 grass, 9-12 rock, 13-15 snow.
constexpr uint8_t HEIGHT_U[16] = {0, 3, 6, 9, 12, 15, 18, 21, 24, 28, 32, 36, 40, 45, 50, 56};
constexpr int   ZTOP = 56;              // tallest possible terrain (u)
constexpr int   BEACONS_MAX = 32;

// clouds (B)
constexpr int   CLOUD_N = 128;          // 128x128 byte map
constexpr float CLOUD_CELL_U = 8.0f;    // -> wraps every 1024 u (independent of terrain)
constexpr float CLOUD_BASE = 100.0f;    // slab base (u)
constexpr float CLOUD_LAYER_U = 3.0f;
constexpr int   CLOUD_MAX_LAYERS = 6;
constexpr float CLOUD_TOP = CLOUD_BASE + CLOUD_LAYER_U * CLOUD_MAX_LAYERS;  // 118
constexpr float CLOUD_COVER = 0.40f;
constexpr int   CLOUD_CLUSTERS = 48;
constexpr float CLOUD_DRIFT_X = 6.0f, CLOUD_DRIFT_Y = 2.5f;  // u/s

// ================================================================ world render (B)
constexpr float VIEW_DIST_BASE = 640.0f, VIEW_DIST_ALT = 2.0f, VIEW_DIST_MAX = 1000.0f;
constexpr float FOG_START_BASE = 120.0f, FOG_START_ALT = 1.5f;
constexpr int   FOG_LEVELS = 8;
constexpr float FOG_T[FOG_LEVELS] = {0.0f, 0.12f, 0.25f, 0.38f, 0.52f, 0.66f, 0.80f, 0.92f};
constexpr int   MARCH_MAX_STEPS = 180;
constexpr float MARCH_START_DEN = 0.15f;    // first depth step at den = 0.15
constexpr float MARCH_DV_MIN = 2.0f, MARCH_DV_MAXK = 2.5f;
constexpr float MIP_M_THRESHOLD = 0.125f;  // use mip when px-per-u m < this
constexpr float NEAR_FX_DIST = 300.0f;     // glints / surf flicker / moon path only nearer than this (zc)
constexpr int   SKY_BANDS = 8;
constexpr float SKY_SPAN_FRAC = 0.75f;     // bands span 0.75*LH rows above the horizon
constexpr int   STARS = 240;
constexpr int   MILKY_DUST = 300;
constexpr float SHOOTING_STAR_MIN_S = 8.0f, SHOOTING_STAR_MAX_S = 18.0f;
// Light: one azimuth for moon (night) and sun (sunset/day). Direction TOWARD the light
// is forward(MOON_AZ). Shadows fall the opposite way.
constexpr float LIGHT_AZ = -0.7853982f;             // forward(-pi/4) = (-0.707,-0.707)
constexpr float MOON_EL_TAN = 0.17f;                // ~10 deg above horizon
constexpr float SUN_EL_TAN_SUNSET = 0.08f;
constexpr float MOON_R_PX = 3.0f, SUN_R_PX = 6.0f;  // logical px
// Ground offset of a shadow per unit of height (x,y), per Tod {Night, Sunset, Day}.
constexpr float SHADOW_K[3] = {0.35f, 0.80f, 0.25f};  // offset = (SHADOW_K, SHADOW_K) * h

// ================================================================ camera (D)
constexpr float CAM_F0 = 88.0f;              // focal length, logical px (hFOV ~68.6 deg)
constexpr float CAM_DIST[3] = {88.0f, 120.0f, 150.0f};  // view presets: near(default)/far/high
constexpr float CAM_TILT[3] = {0.21f, 0.24f, 0.38f};    // base look-down angle (rad)
constexpr float CAM_TILT_CLIMB_K = 0.75f;    // tilt = base - K*sin(pitch)
constexpr float CAM_T_CLAMP = 1.1f;          // clamp on the angle before tan()
constexpr float CAM_T_EASE = 6.0f;
constexpr float CAM_CY_FRAC = 0.64f, CAM_CY_BOOST = 0.03f;
constexpr float CAM_BOOST_PULL = 0.16f, CAM_BOOST_EASE = 4.0f;
constexpr float CAM_FOV_PUNCH = 0.10f, CAM_FOV_SPRING_W = 14.0f, CAM_FOV_SPRING_Z = 0.55f;
constexpr float CAM_FOCUS_EASE = 3.5f, CAM_ALT_EASE = 3.0f, CAM_ALT_FEED = 0.8f;
constexpr float CAM_HEADING_EASE = 3.4f;
constexpr float CAM_FLOOR_MARGIN = 5.0f, CAM_FLOOR_PROBE = 8.0f;
constexpr float CAM_CLOUD_MARGIN = 3.0f, CAM_TFIX_EASE = 7.0f;
constexpr float SHAKE_DECAY = 6.0f, SHAKE_X_PX = 2.5f, SHAKE_Y_PX = 2.0f;
constexpr float CRASH_ORBIT_RATE = 0.6f, CRASH_ORBIT_DIST = 110.0f;
constexpr float CULL_DEN = 0.15f, JET_CULL_DEN = 0.2f;

// ================================================================ flight (D)
constexpr float JET_LEN_U = 34.0f, JET_SPAN_U = 26.0f;
constexpr float V_CRUISE = 68.0f;            // u/s (2 jet lengths per second)
constexpr float BOOST_MUL = 1.8f;
constexpr float BOOST_UP = 5.0f, BOOST_DOWN = 2.4f;
constexpr float BOOST_DRAIN = 0.22f, BOOST_REGEN = 0.10f, BOOST_REGEN_SKIM = 0.35f;
constexpr float BOOST_RING_BONUS = 0.25f, BOOST_MIN_ENGAGE = 0.10f;
constexpr float MAX_YAW = 1.30f, MAX_YAW_BOOST = 1.15f;  // rad/s
constexpr float YAW_EASE = 3.2f;
constexpr float BANK_MAX = 0.96f, BANK_EASE = 6.0f;      // 55 deg, visual only
constexpr float TURN_NOSE_UP = 0.08f;
constexpr float CLIMB_MAX = 1.05f, DIVE_MAX = 0.70f;     // rad
constexpr float CLIMB_EASE = 7.0f, CLIMB_RATE_UP = 1.6f, CLIMB_RATE_DOWN = 1.35f;
constexpr float ENERGY_EASE = 1.1f, CLIMB_BLEED = 0.22f, ENERGY_MAX = 1.15f;
constexpr float CEILING = 220.0f, CEIL_CUSHION = 20.0f;
constexpr float BOB_AMP = 1.0f, BOB_FREQ = 0.9f;
constexpr float SPAWN_ALT = 40.0f;
constexpr float SKIM_Z = 2.0f;                 // sea floor for the jet pivot
constexpr float SKIM_BAND = 8.0f;              // "skimming" below this height above water
constexpr float SEA_CRASH_VZ = -14.0f;         // u/s
constexpr float SEA_CRASH_PITCH = -0.35f;      // rad
constexpr float TERRAIN_CLEAR = 1.5f, NOSE_PROBE = 17.0f, NOSE_CLEAR = 1.0f;
constexpr float CLOSE_CALL_DIST = 5.0f, CLOSE_CALL_COOLDOWN = 2.0f;
constexpr float GPWS_LOOKAHEAD = 0.8f, GPWS_GAP = 4.0f, GPWS_SINK = 8.0f;
constexpr float EASY_FLOOR_CLEAR = 6.0f, EASY_FLOOR_UP = 9.0f, EASY_FLOOR_DOWN = 3.0f;
constexpr float ROLL_TIME = 0.80f, ROLL_DASH_U = 12.0f, ROLL_COOLDOWN = 0.25f;
constexpr float LOOP_TIME = 3.4f;
constexpr float G_EASE = 2.5f;
constexpr float INVULN_TIME = 2.0f;

// ================================================================ tilt / input (D)
// Sensitivity presets [Low, Med, High]: deadzone and full-scale angles in degrees.
constexpr float TILT_ROLL_DZ[3] = {3.0f, 3.0f, 2.5f};
constexpr float TILT_ROLL_FULL[3] = {32.0f, 24.0f, 19.0f};
constexpr float TILT_PITCH_DZ[3] = {4.0f, 4.0f, 3.5f};
constexpr float TILT_PITCH_FULL[3] = {28.0f, 22.0f, 17.0f};
constexpr float TILT_CURVE_LIN = 0.45f;         // out = u*(LIN + (1-LIN)*u)
constexpr float TILT_TAU_STILL = 0.25f, TILT_TAU_MOVING = 2.0f;   // accel correction time const
constexpr float TILT_ACC_TOL_FAST = 0.10f, TILT_ACC_TOL_SLOW = 0.40f;  // | |a|-1g |
constexpr float TILT_OUT_HZ = 10.0f;            // output low-pass
constexpr float TILT_CAL_STILL_DPS = 8.0f, TILT_CAL_STILL_S = 0.5f, TILT_CAL_TIMEOUT_S = 4.0f;
constexpr float TILT_RECAL_S = 0.5f;
constexpr float TILT_DRIFT_TAU = 20.0f, TILT_DRIFT_LIMIT_DEG = 12.0f;
constexpr float TILT_DRIFT_WINDOW_DEG = 8.0f, TILT_DRIFT_TURN = 0.3f, TILT_DRIFT_HOLD_S = 2.0f;
constexpr float TILT_STALE_S = 0.2f;
constexpr float TILT_GUARD_PITCH_DEG = 60.0f;   // beyond this: output frozen at 0, HUD "TILT?"
constexpr float KEY_RAMP_UP = 6.0f, KEY_RAMP_DOWN = 10.0f;
constexpr int8_t ROLL_SIGN_DEFAULT = 1, PITCH_SIGN_DEFAULT = 1;  // VERIFY ON DEVICE (serial r/p flips)
// Platform-side raw->device axis remap (applied in plat_input.cpp). Device frame:
// X right (toward Enter), Y away from user (toward screen edge), Z out of the keys.
constexpr int   IMU_SRC_X = 0, IMU_SRC_Y = 1, IMU_SRC_Z = 2;   // raw index feeding device x,y,z
constexpr float IMU_SGN_X = 1.0f, IMU_SGN_Y = 1.0f, IMU_SGN_Z = 1.0f;

// ================================================================ course / scoring (D)
constexpr int   LIVE_RINGS = 3;
constexpr float RING_R_START = 20.0f, RING_R_END = 14.0f;   // shrinks over RING_DIFF_RINGS
constexpr float RING_PERFECT_R = 6.5f;
constexpr int   RING_DIFF_RINGS = 40;
constexpr float RING_DIST_START = 300.0f, RING_DIST_END = 210.0f;
constexpr float RING_TURN_MIN0 = 0.26f, RING_TURN_MAX0 = 0.70f;   // |delta heading| at d=0 (rad)
constexpr float RING_TURN_MIN1 = 0.44f, RING_TURN_MAX1 = 0.96f;   // at d=1
constexpr float RING_ALT_LOW_MIN = 10.0f, RING_ALT_LOW_MAX = 20.0f;  // above surface
constexpr float RING_ALT_MID_MIN = 25.0f, RING_ALT_MID_MAX = 65.0f;
constexpr float RING_ALT_HIGH_MIN = 128.0f, RING_ALT_HIGH_MAX = 145.0f;
constexpr float RING_P_LOW = 0.50f, RING_P_MID = 0.35f;   // rest HIGH
constexpr int   RING_HIGH_FROM = 6;                       // no HIGH rings before ring 6
constexpr float RING_RIM_CLEAR = 8.0f;
constexpr float RING_MISS_DIST = 520.0f, RING_MISS_TIME = 25.0f;
constexpr float MAGNET_RANGE = 200.0f, MAGNET_CONE = 0.61f;     // rad (35 deg)
constexpr float MAGNET_TURN_K = 1.2f, MAGNET_TURN_MAX = 0.25f;
constexpr float MAGNET_CLIMB_K = 0.016f, MAGNET_CLIMB_MAX = 0.15f;
constexpr float ASSIST_GAIN[3] = {1.0f, 0.6f, 0.0f};            // Easy, Normal, Pro
constexpr float TIMER_START = 45.0f, TIMER_RING = 4.0f, TIMER_PERFECT = 1.5f;
constexpr float TIMER_MAX = 60.0f, TIMER_CRASH = 5.0f;
constexpr int   COMBO_MAX = 8;
constexpr int   PTS_RING = 100, PTS_PERFECT = 50, PTS_ROLL = 100, PTS_LOW = 50, PTS_BOOST = 25;
constexpr int   PTS_SKIM_TICK = 10;  constexpr float SKIM_TICK_S = 0.1f;
constexpr int   PTS_CLOSE = 25, PTS_LOOP = 200;
constexpr float LOW_RING_ALT = 20.0f;
constexpr float HITSTOP_PERFECT_S = 0.06f, HITSTOP_PERFECT_SCALE = 0.5f;
constexpr float CRASH_SLOWMO_S = 0.6f, CRASH_SLOWMO_SCALE = 0.3f, TIMESCALE_EASE = 8.0f;
constexpr float CRASH_SEQ_S = 2.2f, RESPAWN_BACK_U = 200.0f, RESPAWN_ALT = 50.0f;
constexpr float COUNTDOWN_S = 1.5f;
constexpr float ATTRACT_ROLL_MIN_S = 9.0f, ATTRACT_ROLL_MAX_S = 14.0f, ATTRACT_ALT = 55.0f;
constexpr float PERSIST_DEBOUNCE_S = 2.0f;

// ================================================================ jet model / render (C)
constexpr int   JET_GRID_X = 26, JET_GRID_Y = 34, JET_GRID_Z = 11;  // span x length x slices
constexpr int   JET_PIVOT_ROW = 19;       // b = PIVOT_ROW - row  (b > 0 toward the nose)
constexpr int   JET_PIVOT_SLICE = 4;      // s = slice - PIVOT_SLICE
// across a = (x - 12.5) u, stored doubled as a2 = 2x - 25 (odd integers)
constexpr int   JET_MAX_VOX = 1400;
constexpr float JET_SPLAT_BIG_M = 1.6f;   // m >= this: each sub-sample writes 2x2 px
constexpr float JET_LIT_DOT = 0.55f, JET_DIM_DOT = -0.15f, JET_DARK_DOT = -0.55f;
constexpr float JET_SPEC_DOT = 0.92f;
constexpr float NOZZLE_ROW = 33.0f, NOZZLE_SLICE = 4.0f, NOZZLE_R = 2.4f;
constexpr float FLAME_LEN_CRUISE = 6.0f, FLAME_LEN_BOOST = 14.0f;  // u: len = C + B*boost
constexpr float FLAME_R = 2.0f;
constexpr float FLAME_CURL = 0.36f;
constexpr float SHOCK_T[2] = {0.28f, 0.52f};
constexpr float STROBE_PERIOD = 1.2f;
constexpr float NAV_BLINK_PERIOD = 1.0f, NAV_BLINK_DUTY = 0.7f;
constexpr float JET_SHADOW_MAX_H = 60.0f;   // no shadow above this height over ground
constexpr float ASSEMBLE_STAGGER = 0.05f, ASSEMBLE_FALL = 0.30f, ASSEMBLE_DROP_PX = 20.0f;

// ================================================================ effects (C)
constexpr int   PARTICLES = 128;
constexpr int   TRAIL_POINTS = 48;          // per wingtip
constexpr float TRAIL_SPACING_U = 3.0f, TRAIL_LIFE_S = 2.4f;
constexpr int   WAKE_MARKS = 32;
constexpr float WAKE_SPACING_U = 2.4f, WAKE_BEHIND_U = 6.0f, WAKE_LIFE_S = 2.6f;
constexpr float WAKE_START_H = 24.0f, WAKE_FULL_H = 7.0f, WAKE_SPEED_FULL = 50.0f;
constexpr float SMOKE_PERIOD = 0.034f, SMOKE_PERIOD_BOOST = 0.012f, SMOKE_LIFE = 1.0f;
constexpr float SPRAY_RATE_BASE = 40.0f, SPRAY_RATE_K = 70.0f;
constexpr int   SPEEDLINES = 20;
constexpr float SPEEDLINE_AHEAD = 250.0f;
constexpr float MIST_MIN = 0.55f, MIST_MAX = 0.75f;

// ================================================================ audio (D synth, A pump)
constexpr int   AUDIO_RATE = 16000;
constexpr int   AUDIO_BLOCK = 384;          // samples per buffer (24 ms)
constexpr int   AUDIO_BUFFERS = 3;
constexpr int   AUDIO_CHANNEL = 0;          // M5 Speaker virtual channel for the stream
constexpr uint8_t VOLUME_DEFAULT = 110, VOLUME_STEP = 16;

// ================================================================ logging (A)
constexpr float STAT_PERIOD_S = 1.0f;
constexpr float IMU_STREAM_HZ = 20.0f;
constexpr float BATTERY_PERIOD_S = 2.0f;
constexpr float SLOW_FRAME_MS = 20.0f;

}  // namespace cfg
}  // namespace kj
```

### `src/core/types.h`

```cpp
// types.h - common value types and enums shared by every group.
// OWNER: group D (game) with sign-off from all groups. SHARED CONTRACT.
//
// World conventions (all groups):
//   * 1 u (world unit) = 1 jet voxel. Jet is 34 u long, 26 u span.
//   * z is up. Sea surface is z = 0.
//   * Heading psi: forward(psi) = (sin psi, -cos psi, 0); right(psi) = (cos psi, sin psi, 0).
//     psi = 0 points to -y ("north"); positive psi turns right (clockwise seen from above).
//   * NOTE: with x east, y south, z up this frame is LEFT-handed (like a screen map with z
//     toward the viewer). Never derive right/up vectors with cross() in world space; use
//     rightOf()/forwardOf() and the jet-axis formulas in render_jet.h. (Device-frame IMU
//     math in tilt.cpp is a separate, right-handed frame - cross() is fine there.)
//   * The world wraps every cfg::WORLD_U (8192 u) in x and y. Stored positions are kept in
//     [0, WORLD_U) with wrapPos(); every position DIFFERENCE must go through wrapd().
#pragma once
#include <stdint.h>
#include <math.h>

namespace kj {

struct Vec3 {
  float x, y, z;
};
inline Vec3 v3(float x, float y, float z) { Vec3 r = {x, y, z}; return r; }
inline Vec3 operator+(const Vec3& a, const Vec3& b) { return v3(a.x + b.x, a.y + b.y, a.z + b.z); }
inline Vec3 operator-(const Vec3& a, const Vec3& b) { return v3(a.x - b.x, a.y - b.y, a.z - b.z); }
inline Vec3 operator*(const Vec3& a, float s) { return v3(a.x * s, a.y * s, a.z * s); }
inline Vec3 operator*(float s, const Vec3& a) { return v3(a.x * s, a.y * s, a.z * s); }
inline Vec3& operator+=(Vec3& a, const Vec3& b) { a.x += b.x; a.y += b.y; a.z += b.z; return a; }
inline float dot(const Vec3& a, const Vec3& b) { return a.x * b.x + a.y * b.y + a.z * b.z; }
inline Vec3 cross(const Vec3& a, const Vec3& b) {
  return v3(a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x);
}
inline float length(const Vec3& a) { return sqrtf(dot(a, a)); }
inline Vec3 normalize(const Vec3& a) {
  float l = length(a);
  return l > 1e-9f ? a * (1.0f / l) : v3(0.0f, 0.0f, 1.0f);
}

struct Rect {
  int16_t x0, y0, x1, y1;  // inclusive-exclusive: [x0,x1) x [y0,y1); empty if x1<=x0
};

enum class Tod : uint8_t { Night = 0, Sunset = 1, Day = 2, Count = 3 };
enum class Assist : uint8_t { Easy = 0, Normal = 1, Pro = 2 };
enum class Sens : uint8_t { Low = 0, Med = 1, High = 2 };

}  // namespace kj
```

### `src/core/mathx.h`

```cpp
// mathx.h - small inline math helpers, hashing, Bayer dither, deterministic RNG.
// OWNER: group D. SHARED CONTRACT (header-only; no .cpp).
// Rule: float only (sinf/cosf/expf/sqrtf/atan2f, literals with f suffix). No double.
#pragma once
#include <stdint.h>
#include <math.h>
#include "config.h"
#include "types.h"

namespace kj {

constexpr float PI = 3.14159265f;
constexpr float TAU = 6.28318531f;
constexpr float DEG = 0.0174532925f;   // degrees -> rad
constexpr float RAD2DEG = 57.2957795f;

inline float clampf(float v, float lo, float hi) { return v < lo ? lo : (v > hi ? hi : v); }
inline int clampi(int v, int lo, int hi) { return v < lo ? lo : (v > hi ? hi : v); }
inline float lerpf(float a, float b, float t) { return a + (b - a) * t; }
inline float signf(float v) { return v < 0.0f ? -1.0f : 1.0f; }
inline float sat(float v) { return clampf(v, 0.0f, 1.0f); }

// First-order easing: x += (target - x) * (1 - exp(-k dt)).
inline float easeFactor(float k, float dt) { return 1.0f - expf(-k * dt); }
inline void easeTo(float& x, float target, float k, float dt) { x += (target - x) * easeFactor(k, dt); }
inline float easeInOutQuad(float t) {
  t = sat(t);
  return t < 0.5f ? 2.0f * t * t : 1.0f - 2.0f * (1.0f - t) * (1.0f - t);
}
inline float smooth01(float t) { t = sat(t); return t * t * (3.0f - 2.0f * t); }

// Angle wrap to [-pi, pi).
inline float wrapPi(float a) {
  a = fmodf(a + PI, TAU);
  if (a < 0.0f) a += TAU;
  return a - PI;
}
// World position wrap to [0, WORLD_U) and world delta wrap to [-WORLD_U/2, WORLD_U/2).
inline float wrapPos(float p) {
  p = fmodf(p, cfg::WORLD_U);
  return p < 0.0f ? p + cfg::WORLD_U : p;
}
inline float wrapd(float d) {
  const float W = cfg::WORLD_U, H = cfg::WORLD_U * 0.5f;
  if (d >= H || d < -H) d -= W * floorf((d + H) / W);
  return d;
}
inline Vec3 wrapPos3(const Vec3& p) { return v3(wrapPos(p.x), wrapPos(p.y), p.z); }
inline Vec3 wrapDelta(const Vec3& a, const Vec3& b) {  // a - b, wrapped in x/y
  return v3(wrapd(a.x - b.x), wrapd(a.y - b.y), a.z - b.z);
}

inline Vec3 forwardOf(float psi) { return v3(sinf(psi), -cosf(psi), 0.0f); }
inline Vec3 rightOf(float psi) { return v3(cosf(psi), sinf(psi), 0.0f); }

// Integer hashes (identical on PC and device; used by world gen -> world CRC parity).
inline uint32_t hash32(uint32_t x) {
  x ^= x >> 16; x *= 0x7feb352dU; x ^= x >> 15; x *= 0x846ca68bU; x ^= x >> 16;
  return x;
}
inline uint32_t hash2i(int32_t x, int32_t y) {
  return hash32((uint32_t)x * 0x9E3779B1U ^ hash32((uint32_t)y + 0x632BE5ABU));
}
inline uint32_t hash3i(int32_t x, int32_t y, int32_t z) {
  return hash32(hash2i(x, y) ^ ((uint32_t)z * 0x85EBCA77U));
}

// 4x4 Bayer matrix, values 0..15. alpha16 in 0..16: pixel passes when BAYER4 < alpha16.
constexpr uint8_t BAYER4[16] = {0, 8, 2, 10, 12, 4, 14, 6, 3, 11, 1, 9, 15, 7, 13, 5};
inline int bayer(int x, int y) { return BAYER4[((y & 3) << 2) | (x & 3)]; }
inline bool bayerPass(int x, int y, int alpha16) { return bayer(x, y) < alpha16; }
inline int alpha16(float a) { return clampi((int)(a * 16.0f + 0.5f), 0, 16); }

// Deterministic xorshift32 RNG (never seed with 0).
struct Rng {
  uint32_t s = 0x1234567u;
  explicit Rng(uint32_t seed = 0x1234567u) : s(seed ? seed : 0x1234567u) {}
  uint32_t next() { s ^= s << 13; s ^= s >> 17; s ^= s << 5; return s; }
  float uniform() { return (float)(next() >> 8) * (1.0f / 16777216.0f); }  // [0,1)
  float range(float a, float b) { return a + (b - a) * uniform(); }
  int irange(int lo, int hiInclusive) { return lo + (int)(next() % (uint32_t)(hiInclusive - lo + 1)); }
  bool chance(float p) { return uniform() < p; }
};

}  // namespace kj
```

### `src/core/input.h`

```cpp
// input.h - raw input frame delivered by the platform, key positions, action keymap.
// OWNER: group D (keymap, actions) + group A (InputFrame filling). SHARED CONTRACT.
//
// Keys are identified by PHYSICAL POSITION kp = y*14 + x from Keyboard.keyList()
// (x 0..13, y 0..3), never by character, so Shift/Ctrl/Fn can never change meaning.
// Layout (Cardputer-Adv):
//   y0: `  1  2  3  4  5  6  7  8  9  0  -  =  BKSP
//   y1: TAB q  w  e  r  t  y  u  i  o  p  [  ]  BSLASH
//   y2: FN SHIFT a s  d  f  g  h  j  k  l  ;  '  ENTER
//   y3: CTRL OPT ALT z x  c  v  b  n  m  ,  .  /  SPACE
#pragma once
#include <stdint.h>

namespace kj {

enum KeyPos : uint8_t {
  KP_BACKTICK = 0, KP_1 = 1, KP_2 = 2, KP_3 = 3, KP_4 = 4, KP_5 = 5, KP_6 = 6, KP_7 = 7, KP_8 = 8,
  KP_9 = 9, KP_0 = 10, KP_MINUS = 11, KP_EQUAL = 12, KP_BKSP = 13,
  KP_TAB = 14, KP_Q = 15, KP_W = 16, KP_E = 17, KP_R = 18, KP_T = 19, KP_Y = 20, KP_U = 21,
  KP_I = 22, KP_O = 23, KP_P = 24, KP_LBRACK = 25, KP_RBRACK = 26, KP_BSLASH = 27,
  KP_FN = 28, KP_SHIFT = 29, KP_A = 30, KP_S = 31, KP_D = 32, KP_F = 33, KP_G = 34, KP_H = 35,
  KP_J = 36, KP_K = 37, KP_L = 38, KP_SEMI = 39, KP_QUOTE = 40, KP_ENTER = 41,
  KP_CTRL = 42, KP_OPT = 43, KP_ALT = 44, KP_Z = 45, KP_X = 46, KP_C = 47, KP_V = 48, KP_B = 49,
  KP_N = 50, KP_M = 51, KP_COMMA = 52, KP_DOT = 53, KP_SLASH = 54, KP_SPACE = 55,
  KP_COUNT = 56
};
inline uint64_t keyBit(int kp) { return 1ull << kp; }

// Names used by the simulator scripts and the serial key echo ('k').
constexpr const char* KEY_NAMES[KP_COUNT] = {
  "BACKTICK", "1", "2", "3", "4", "5", "6", "7", "8", "9", "0", "MINUS", "EQUAL", "BKSP",
  "TAB", "Q", "W", "E", "R", "T", "Y", "U", "I", "O", "P", "LBRACK", "RBRACK", "BSLASH",
  "FN", "SHIFT", "A", "S", "D", "F", "G", "H", "J", "K", "L", "SEMI", "QUOTE", "ENTER",
  "CTRL", "OPT", "ALT", "Z", "X", "C", "V", "B", "N", "M", "COMMA", "DOT", "SLASH", "SPACE"};

// IMU sample in the DEVICE frame (platform already applied cfg::IMU_SRC_*/IMU_SGN_*):
//   X right (toward Enter), Y away from the user (toward the screen edge), Z out of the keys.
//   Lying flat: az ~ +1 g. Accel in g, gyro in deg/s (right-hand rule).
struct ImuSample {
  float ax, ay, az;
  float gx, gy, gz;
  uint32_t usec;   // sensor timestamp (M5 getImuData().usec)
  bool fresh;      // false if M5.Imu.update() reported no new sample this frame
};

struct InputFrame {
  float dt;                // real frame time, s (platform measures, game clamps to cfg::DT_MAX)
  uint64_t keysDown;       // bit kp set = key currently held
  uint64_t keysPressed;    // bit kp set = key went down since the previous InputFrame
                           // (accumulated across all FIFO drains, so taps < 1 frame are kept)
  bool btnA;               // G0 button held
  bool btnAPressed;        // G0 went down since previous frame
  bool imuPresent;         // board is ADV and M5.Imu reported a sensor
  ImuSample imu;
  char serialCmd;          // forwarded single-char serial command for the game, 0 if none
  uint8_t batteryPct;      // 0..100, refreshed every cfg::BATTERY_PERIOD_S
  uint16_t batteryMv;
};

// ------------------------------------------------------------------ actions
enum Act : uint32_t {
  A_LEFT = 1u << 0, A_RIGHT = 1u << 1, A_UP = 1u << 2 /*climb*/, A_DOWN = 1u << 3 /*dive*/,
  A_BOOST = 1u << 4, A_ROLL_L = 1u << 5, A_ROLL_R = 1u << 6, A_ROLL_AUTO = 1u << 7,
  A_CONFIRM = 1u << 8, A_PAUSE = 1u << 9, A_BACK = 1u << 10, A_RECAL = 1u << 11,
  A_TILT = 1u << 12, A_INVERT = 1u << 13, A_TOD = 1u << 14, A_VIEW = 1u << 15,
  A_FPS = 1u << 16, A_IMUDBG = 1u << 17, A_HELP = 1u << 18, A_MUTE = 1u << 19,
  A_VOL_DN = 1u << 20, A_VOL_UP = 1u << 21, A_BRI_DN = 1u << 22, A_BRI_UP = 1u << 23,
  A_MODE1 = 1u << 24, A_MODE2 = 1u << 25, A_LOOP = 1u << 26
};

struct KeyBind {
  uint8_t kp;
  uint32_t acts;
};

// THE keymap (decided; see DESIGN.md section 9). G0 (BtnA) additionally maps to A_PAUSE.
constexpr KeyBind KEYMAP[] = {
  {KP_COMMA, A_LEFT},  {KP_A, A_LEFT},   {KP_SLASH, A_RIGHT}, {KP_D, A_RIGHT},
  {KP_SEMI, A_UP},     {KP_W, A_UP},     {KP_DOT, A_DOWN},    {KP_S, A_DOWN},
  {KP_SPACE, A_BOOST}, {KP_ALT, A_BOOST},
  {KP_Z, A_ROLL_L},    {KP_L, A_ROLL_L}, {KP_X, A_ROLL_R},    {KP_QUOTE, A_ROLL_R},
  {KP_ENTER, A_CONFIRM | A_ROLL_AUTO},
  {KP_P, A_PAUSE},     {KP_BACKTICK, A_PAUSE}, {KP_BKSP, A_BACK},
  {KP_C, A_RECAL},     {KP_T, A_TILT},   {KP_I, A_INVERT},    {KP_N, A_TOD},
  {KP_V, A_VIEW},      {KP_F, A_FPS},    {KP_G, A_IMUDBG},    {KP_H, A_HELP},
  {KP_M, A_MUTE},      {KP_MINUS, A_VOL_DN}, {KP_EQUAL, A_VOL_UP},
  {KP_LBRACK, A_BRI_DN}, {KP_RBRACK, A_BRI_UP},
  {KP_1, A_MODE1},     {KP_2, A_MODE2},  {KP_Q, A_LOOP},
};
constexpr int KEYMAP_COUNT = (int)(sizeof(KEYMAP) / sizeof(KEYMAP[0]));

inline uint32_t keysToActs(uint64_t keys) {
  uint32_t a = 0;
  for (int i = 0; i < KEYMAP_COUNT; ++i)
    if (keys & keyBit(KEYMAP[i].kp)) a |= KEYMAP[i].acts;
  return a;
}

}  // namespace kj
```

### `src/core/frame.h`

```cpp
// frame.h - the logical frame buffer (8-bit palette indices + 8-bit depth) and depth quantiser.
// OWNER: group B. SHARED CONTRACT.
//
// px[y*LW + x]    : palette index, 0 = pal::EMPTY ("not yet written"). Cleared every frame.
// depth[y*LW + x] : quantised camera depth dq(zc), 255 = DEPTH_FAR (nothing). Cleared every frame.
// Depth test rule used by EVERY drawer: a fragment with depth d is visible iff d <= depth[p].
// Opaque geometry (terrain, clouds, rings, jet voxels, flame) writes depth; translucent/dithered
// effects (trails, smoke, spray, glow halos, HUD) test but do not write.
#pragma once
#include <stdint.h>
#include "config.h"

namespace kj {

constexpr int LW = cfg::LW, LH = cfg::LH, LPIX = cfg::LPIX;
constexpr uint8_t DEPTH_FAR = 255;

struct FrameBuf {
  uint8_t* px;     // LPIX bytes
  uint8_t* depth;  // LPIX bytes
};

// zc = distance from the camera along the view direction (u) = dist * den.
// 1 u resolution up to 160 u (jet sits at ~88 +- 30), then 8 u steps; 254 ~ 910 u+.
inline uint8_t depthQ(float zc) {
  if (zc <= 0.0f) return 0;
  if (zc < 160.0f) return (uint8_t)zc;
  int q = 160 + (int)((zc - 160.0f) * 0.125f);
  return (uint8_t)(q > 254 ? 254 : q);
}

inline bool inFrame(int x, int y) { return (unsigned)x < (unsigned)LW && (unsigned)y < (unsigned)LH; }

}  // namespace kj
```

### `src/core/perf.h`

```cpp
// perf.h - per-stage profiler and work counters. Same code on device and in the simulator,
// so [STAT] lines are directly comparable (device times are real; sim times are PC times
// plus an ESP32 estimate computed from the counters).
// OWNER: group A (perf.cpp). SHARED CONTRACT.
#pragma once
#include <stdint.h>

namespace kj {

enum Stage : uint8_t {
  ST_INPUT = 0,  // platform: keyboard drain + IMU read            (recorded by platform)
  ST_SIM,        // game update: tilt, flight, camera, course, fx update, audio params
  ST_PAL,        // palette finalize (fx lerps -> LUT)
  ST_WORLD,      // renderWorld (terrain, sea, clouds)
  ST_SKY,        // renderSky
  ST_OBJ,        // rings, beacons
  ST_JET,        // jet shadow + jet + flame + outline + nav lights
  ST_FX,         // particles, trails, wake, speed lines, mist
  ST_HUD,        // HUD / menus / text
  ST_AUDIO,      // synth render + playRaw   (recorded by platform)
  ST_EXPAND,     // 2x LUT expand            (recorded by platform)
  ST_DMAWAIT,    // time blocked in waitDMA  (recorded by platform; large = display-bound)
  ST_COUNT
};

constexpr const char* STAGE_NAMES[ST_COUNT] = {"in", "sim", "pal", "wld", "sky", "obj",
                                               "jet", "fx", "hud", "aud", "exp", "dmaw"};

// Work counters, cleared at the start of every Game::render(), accumulated per window.
struct RenderStats {
  uint32_t steps;      // world march depth steps
  uint32_t samples;    // world column samples (terrain + cloud lookups)
  uint32_t fills;      // pixels written by the world march
  uint32_t voxels;     // jet voxels transformed
  uint32_t voxWrites;  // jet pixel writes attempted
  uint32_t flamePx;    // flame pixel writes attempted
  uint32_t particles;  // particles drawn
  uint32_t linePx;     // line pixels (rings, trails, speed lines)
  void clear() { steps = samples = fills = voxels = voxWrites = flamePx = particles = linePx = 0; }
};

// ESP32-S3 cost model (cycles per unit) used by the simulator's "esp_est" column.
// Least-squares fit to 71 device [STAT] windows (cardputer-adv, 240 MHz, Ring Run autopilot
// bench at night and day plus title flyover; same counters as the sim), mean error 4-5 % for
// wld and jet. cStep carries the per-step work that has no counter of its own: the row
// conversions and divide (~750 cycles), the 15-chunk classification (~3,000) and the cloud
// slab chunks. A fill is ~46 cycles because most uniform-chunk calls write a single row, so
// the per-call setup is spread over few pixels. Real code runs at ~2-3 cycles per instruction
// on this core (taken branches, load-use stalls), so instruction counts x 1.3 under-estimate.
// The jet weights live in tools/sim/sim_main.cpp (JET_C_*); cVoxel / cVoxWrite / cFlame here
// are the same fit for Profiler::estEspMs. fx is only loosely explained by its counters (~30 %):
// cLinePx is pinned to a plausible 60 cycles (lineF's inner loop) and a particle is ~1,345
// cycles; the fx stage's fixed ~0.28 ms is in the sim's ESP_FIXED_MS.
struct CostModel {
  float cSample = 25.6f, cFill = 46.0f, cStep = 4920.0f, cVoxel = 421.0f, cVoxWrite = 38.5f,
        cFlame = 97.0f, cParticle = 1345.0f, cLinePx = 60.0f, cyclesPerUs = 240.0f;
};

class Profiler {
 public:
  using ClockUs = uint32_t (*)();
  void init(ClockUs clock);
  void begin(Stage s);                 // start timing stage s
  void end(Stage s);                   // stop timing stage s (accumulates into this frame)
  void add(Stage s, uint32_t us);      // add externally measured time
  RenderStats& stats() { return cur_; }
  // Close the frame: frameUs = wall time since previous endFrame (pacing included).
  void endFrame(uint32_t frameUs);
  // True once per cfg::STAT_PERIOD_S of accumulated frame time; then call format() and reset().
  bool windowReady() const;
  // "fps=59.9 ft=16.7/18.2 c1=4.1/6.3 in=0.71 sim=0.20 ... | smp=9402 stp=77 fill=6104 vox=598 vw=4200 fl=120 prt=38 lp=600"
  int format(char* buf, int n) const;
  float estEspMs(const CostModel& cm) const;  // average per-frame estimate from counters
  void reset();
  float fps() const;
  float avgMs(Stage s) const;
  float coreMsAvg() const;   // sum of all stages except ST_DMAWAIT
  float frameMsMax() const;

 private:
  ClockUs clock_ = nullptr;
  uint32_t t0_[ST_COUNT] = {};
  uint32_t frameUs_[ST_COUNT] = {};     // current frame
  uint64_t sumUs_[ST_COUNT] = {};       // window sums
  uint32_t maxCoreUs_ = 0, maxFrameUs_ = 0;
  uint64_t windowUs_ = 0;
  uint32_t frames_ = 0;
  RenderStats cur_ = {};
  RenderStats sum_ = {};
};

}  // namespace kj
```

### `src/core/palette.h`

```cpp
// palette.h - the 256-entry indexed palette: fixed index map, world (fogged) block,
// per-time-of-day base colours, per-frame effects (flash / mist / fade), device LUT.
// OWNER: group B (palette.cpp holds all colour tables). SHARED CONTRACT (index map).
//
// Index map:
//   0..79   fixed entries (never fogged), named in namespace pal below
//   80..255 world block: pal::world(fog, base) = 80 + fog*22 + base, fog 0..7, base 0..21
// Every drawer writes these indices; nothing outside palette.cpp knows actual colours.
#pragma once
#include <stdint.h>
#include "types.h"

namespace kj {
namespace pal {

// ---- fixed (0..79)
constexpr uint8_t EMPTY = 0;          // "not yet written"; LUT shows it as haze (safety net)
constexpr uint8_t HAZE = 1;           // far sea / below-horizon fill
constexpr uint8_t SKY0 = 2;           // 2..9: sky bands, 0 = horizon ... 7 = zenith
constexpr uint8_t STAR_BRIGHT = 10, STAR_MID = 11, STAR_DIM = 12, STAR_WARM = 13, MILKY = 14;
constexpr uint8_t MOON_DISC = 15, MOON_CRATER = 16, MOON_GLOW_IN = 17, MOON_GLOW_OUT = 18;
//   (at sunset MOON_* entries hold the sun disc / glow colours)
constexpr uint8_t FOAM = 19, GLINT_A = 20, GLINT_B = 21, MOONPATH = 22;
constexpr uint8_t SH_SEA = 23, SH_SHALLOW = 24, SH_SAND = 25, SH_GRASS = 26, SH_TREE = 27,
                  SH_ROCK = 28, SH_SNOW = 29;          // shadowed variants (jet + cloud shadow)
constexpr uint8_t JET_OUTLINE = 30;
constexpr uint8_t JET_BODY0 = 31;     // 31..37: body tones 0 (dark) .. 6 (light)
constexpr uint8_t JET_ACCENT0 = 38;   // 38..41: accent tones 0..3
constexpr uint8_t JET_CANOPY0 = 42;   // 42..44: canopy tones 0..2
constexpr uint8_t JET_NOZZLE = 45, JET_GLOW = 46;
constexpr uint8_t NAV_RED = 47, NAV_GREEN = 48, STROBE = 49;
constexpr uint8_t FLAME0 = 50;        // 50..55: flame hot (0) .. cool (5)
constexpr uint8_t TRAIL0 = 56;        // 56..58: trail young .. old
constexpr uint8_t SMOKE0 = 59;        // 59..61: smoke dark .. light
constexpr uint8_t WAKE_A = 62, WAKE_B = 63;  // bioluminescent at night, foam by day
constexpr uint8_t RING_GOLD = 64, RING_GOLD_DARK = 65, RING_CYAN = 66, RING_GLOW = 67;
constexpr uint8_t BEACON = 68, SPEEDLINE = 69;
constexpr uint8_t HUD_WHITE = 70, HUD_AMBER = 71, HUD_RED = 72, HUD_GREEN = 73, HUD_SHADOW = 74,
                  HUD_CYAN = 75;      // HUD entries are exempt from flash/mist/fade
constexpr uint8_t MIST = 76, SPARK = 77, SPRAY = 78, RESERVED79 = 79;

constexpr uint8_t JET_FIRST = JET_OUTLINE, JET_LAST = STROBE;  // 30..49 = "jet body" range
constexpr uint8_t HUD_FIRST = HUD_WHITE, HUD_LAST = HUD_CYAN;

inline bool isSky(uint8_t i) { return i >= SKY0 && i < SKY0 + 8; }
inline bool isJetBody(uint8_t i) { return i >= JET_BODY0 && i <= STROBE; }   // excludes outline
inline bool isFlame(uint8_t i) { return i >= FLAME0 && i < FLAME0 + 6; }

// ---- world block
constexpr uint8_t WORLD0 = 80;
constexpr int WORLD_BASES = 22;
enum Base : uint8_t {
  B_SEA_DEEP = 0, B_SEA_MID, B_SEA_SHALLOW, B_SURF,
  B_SAND, B_SAND_SIDE,
  B_GRASS_LIT, B_GRASS, B_GRASS_DARK, B_GRASS_SIDE,
  B_TREE_TOP, B_TREE, B_TREE_SIDE,
  B_ROCK_LIT, B_ROCK, B_ROCK_DARK, B_ROCK_SIDE,
  B_SNOW, B_SNOW_SIDE,
  B_CLOUD_TOP, B_CLOUD_SIDE, B_CLOUD_UNDER
};
inline uint8_t world(int fog, int base) { return (uint8_t)(WORLD0 + fog * WORLD_BASES + base); }
inline bool isWorld(uint8_t i) { return i >= WORLD0; }
inline int worldFog(uint8_t i) { return (i - WORLD0) / WORLD_BASES; }
inline int worldBase(uint8_t i) { return (i - WORLD0) % WORLD_BASES; }

}  // namespace pal

// The shadow remap is index-based and identical for every Tod; renderWorld (cloud shadows)
// uses this static table so it does not need a Palette instance. Palette::shade() == table.
const uint8_t* shadeTable();

// Per-frame palette effects (all 0 = off). HUD entries are never affected.
struct PalFx {
  float flash;         // 0..1 lerp toward flashRgb (crash 1.0, perfect 0.35, tod change 0.5)
  uint32_t flashRgb;   // 0xRRGGBB
  float mist;          // 0..1 lerp toward the MIST colour (in-cloud)
  float fade;          // 0..1 lerp toward black (screen transitions)
};

class Palette {
 public:
  void init();                       // builds all three base tables (sunset graded from day)
  void setTod(Tod t);                // selects the base table and rebuilds shade map
  Tod tod() const { return tod_; }
  void setFx(const PalFx& fx) { fx_ = fx; }
  void finalize();                   // once per frame: base + fx -> lut2x_ / rgb888_ (~256 lerps)
  // Device LUT: entry = (c << 16) | c, c = byte-swapped RGB565 of the final colour.
  const uint32_t* lut2x() const { return lut2x_; }
  // Final colours as RGB888 triplets (sim PNGs, serial screenshot).
  const uint8_t* rgb888() const { return rgb888_; }
  // Shadow remap: world indices at fog 0..2 and the fixed sea entries map to their SH_* entry;
  // everything else maps to itself. Idempotent.
  uint8_t shade(uint8_t idx) const { return shade_[idx]; }
  uint32_t baseRgb(uint8_t idx) const { return base_[(int)tod_][idx]; }

 private:
  Tod tod_ = Tod::Night;
  PalFx fx_ = {0.0f, 0xFFF4E0u, 0.0f, 0.0f};
  uint32_t base_[3][256] = {};       // 0xRRGGBB per Tod, fog block precomputed
  uint32_t lut2x_[256] = {};
  uint8_t rgb888_[768] = {};
  uint8_t shade_[256] = {};
  // finalize() skips the rebuild while (tod_, fx_) are unchanged since its last run.
  PalFx lastFx_ = {0.0f, 0u, 0.0f, 0.0f};
  bool finalized_ = false;
};

}  // namespace kj
```

### `src/core/world.h`

```cpp
// world.h - procedural island world (stamps on a macro table) + cloud map + queries.
// OWNER: group B (world.cpp). SHARED CONTRACT.
//
// Terrain cell byte = (level << 4) | material. level 0..15 -> cfg::HEIGHT_U[level] u.
// 1 cell = cfg::CELL_U (4) u; world wraps every 2048 cells (8192 u).
// Storage (static .bss in world.cpp, DRAM, never flash-const):
//   stamps[12][64*64] 49,152 B | mips[12][32*32] 12,288 B | macro[32*32] 1,024 B
//   clouds[128*128]  16,384 B (byte = thickness 0..6 layers)
// World generation uses INTEGER math only (Q16 value noise, kj::hash*, kj::Rng) so the
// world CRC is bit-identical on PC (MSVC) and device (xtensa gcc). Both log it at boot.
#pragma once
#include <stdint.h>
#include "config.h"
#include "types.h"

namespace kj {

namespace mat {
enum : uint8_t {
  DEEP = 0, MID = 1, SHALLOW = 2, SURF = 3,    // level is always 0 for these
  SAND = 4, SPECK = 5,                          // level 0 (beach)
  GRASS_L = 6, GRASS = 7, GRASS_D = 8,          // lit / normal / dark (baked slope light)
  TREE_TOP = 9, TREE = 10,
  ROCK_L = 11, ROCK = 12, ROCK_D = 13,
  SNOW = 14, SNOW_D = 15
};
}
inline uint8_t cellLevel(uint8_t c) { return c >> 4; }
inline uint8_t cellMat(uint8_t c) { return c & 15; }
inline bool cellIsWater(uint8_t c) { return (c & 15) <= mat::SURF; }
constexpr uint8_t CELL_DEEP = 0;  // level 0, DEEP

struct Beacon {
  float x, y, z;     // world position of the light (top of its cell)
  uint8_t phase;     // blink phase 0..255
};

class World {
 public:
  // Generates stamps, mips, macro table, cloud map, beacon list. ~150 ms on device [E].
  void generate(uint32_t seed);
  uint32_t crc() const;  // CRC32 over stamps + mips + macro + clouds

  // ---- terrain (cx, cy are cell coordinates; any int, wrapped internally)
  inline uint8_t cell(int cx, int cy) const;
  inline uint8_t cellMip(int mx, int my) const;  // mip cells are 2x2 terrain cells (8 u)
  float groundHeight(float x, float y) const;    // u; 0 over water. Nearest cell (matches visuals)
  bool isWater(float x, float y) const;
  float maxGroundAround(float x, float y, float r) const;  // max over a 3x3 probe at +-r

  // ---- clouds (drift applied through setTime)
  void setTime(float t);                         // updates cloudOffX/Y = drift * t
  inline uint8_t cloudCell(int ccx, int ccy) const;   // raw map cell, wrapped
  int cloudThick(float x, float y) const;        // layers 0..6 at world (x,y), drift included
  bool insideCloud(float x, float y, float z) const;  // z within [BASE, BASE + 3*thick]
  float cloudOffX = 0.0f, cloudOffY = 0.0f;

  // ---- gameplay helpers
  // Open water, land 120..300 u away; yaw faces that land. salt varies the pick (respawns).
  void findSpawn(uint32_t salt, float& x, float& y, float& yaw) const;
  int beaconsNear(float x, float y, float r, Beacon* out, int maxN) const;
  int beaconCount() const { return nBeacons_; }

  // Raw pointers for the renderer's inner loop.
  const uint8_t* stamps() const { return stamps_; }
  const uint8_t* mips() const { return mips_; }
  const uint8_t* macro() const { return macro_; }
  const uint8_t* clouds() const { return clouds_; }

 private:
  uint32_t blockFlags(int bx, int by) const;  // 16x16-cell block: 0x80 all CELL_DEEP, 0x40 all water
  uint8_t* stamps_ = nullptr;  // [STAMP_COUNT][STAMP_N*STAMP_N]
  uint8_t* mips_ = nullptr;    // [STAMP_COUNT][MIP_N*MIP_N]
  uint8_t* macro_ = nullptr;   // [MACRO_N*MACRO_N]; MACRO_OCEAN or (stamp id | MACRO_TRANSPOSE?)
  uint8_t* clouds_ = nullptr;  // [CLOUD_N*CLOUD_N]
  Beacon beacons_[cfg::BEACONS_MAX] = {};
  int nBeacons_ = 0;
};

inline uint8_t World::cell(int cx, int cy) const {
  cx &= cfg::WORLD_CELLS - 1;
  cy &= cfg::WORLD_CELLS - 1;
  uint8_t t = macro_[(cy >> 6) * cfg::MACRO_N + (cx >> 6)];
  if (t == cfg::MACRO_OCEAN) return CELL_DEEP;
  int lx = cx & (cfg::STAMP_N - 1), ly = cy & (cfg::STAMP_N - 1);
  if (t & cfg::MACRO_TRANSPOSE) { int s = lx; lx = ly; ly = s; }
  return stamps_[(t & 15) * (cfg::STAMP_N * cfg::STAMP_N) + (ly << 6) + lx];
}

inline uint8_t World::cellMip(int mx, int my) const {
  mx &= (cfg::WORLD_CELLS / 2) - 1;
  my &= (cfg::WORLD_CELLS / 2) - 1;
  uint8_t t = macro_[(my >> 5) * cfg::MACRO_N + (mx >> 5)];
  if (t == cfg::MACRO_OCEAN) return CELL_DEEP;
  int lx = mx & (cfg::MIP_N - 1), ly = my & (cfg::MIP_N - 1);
  if (t & cfg::MACRO_TRANSPOSE) { int s = lx; lx = ly; ly = s; }
  return mips_[(t & 15) * (cfg::MIP_N * cfg::MIP_N) + (ly << 5) + lx];
}

inline uint8_t World::cloudCell(int ccx, int ccy) const {
  return clouds_[((ccy & (cfg::CLOUD_N - 1)) << 7) | (ccx & (cfg::CLOUD_N - 1))];
}

}  // namespace kj
```

### `src/core/camera.h`

```cpp
// camera.h - level pinhole chase camera with lens shift (no camera pitch or roll).
// OWNER: group D (camera.cpp). SHARED CONTRACT (View + project() are used by B, C, D).
//
// Projection of a world point P (all groups MUST use project() or the identical formula):
//   d = wrapDelta(P, focus);  across = d.x*cpsi + d.y*spsi ... see project() below
//   across = dx*cos(psi) + dy*sin(psi)            (= dot(d, right(psi)))
//   depth  = dx*sin(psi) - dy*cos(psi)            (= dot(d, forward(psi)), + = away from camera)
//   den    = 1 + q*depth   (cull if den < minDen)   m = S/den   (px per u at that depth)
//   sx     = Cx + m*across
//   sy     = Cy - m*((z - alt) + T*depth)
//   zc     = dist*den  (camera-space depth, for depthQ)
// The eye sits dist behind and T*dist above the focus; horizon row = Cy - f*T, f = S*dist.
#pragma once
#include <stdint.h>
#include "types.h"
#include "frame.h"

namespace kj {

class World;
struct JetState;

struct View {
  float fx, fy, alt;     // focus point (x,y wrapped)
  float psi, cpsi, spsi; // view heading and its cos/sin
  float dist;            // eye distance behind the focus (u)
  float q;               // 1/dist
  float S;               // px per u at the focus
  float f;               // focal length px = S*dist
  float T;               // look-down slope (includes terrain/cloud fix)
  float Cx, Cy;          // focus screen position (logical px, includes shake)
  float horizon;         // Cy - f*T (row of the horizon, may be off-screen)
  float eyeX, eyeY, eyeZ;
  float viewDist;        // march far limit (zc)
  float fogStart;        // zc where fog begins
  bool camBelowClouds;   // eyeZ < CLOUD_BASE
};

struct ScreenPt {
  float x, y;   // logical px
  float m;      // px per u at that depth
  float zc;     // camera depth (u)
  uint8_t dq;   // depthQ(zc)
};

inline bool project(const View& v, const Vec3& p, ScreenPt& out, float minDen = 0.15f) {
  float dx = p.x - v.fx, dy = p.y - v.fy;
  const float W = 8192.0f, H = 4096.0f;   // == cfg::WORLD_U (wrapd inlined for speed)
  if (dx >= H) dx -= W; else if (dx < -H) dx += W;
  if (dy >= H) dy -= W; else if (dy < -H) dy += W;
  float across = dx * v.cpsi + dy * v.spsi;
  float depth = dx * v.spsi - dy * v.cpsi;
  float den = 1.0f + v.q * depth;
  if (den < minDen) return false;
  float m = v.S / den;
  out.x = v.Cx + m * across;
  out.y = v.Cy - m * ((p.z - v.alt) + v.T * depth);
  out.m = m;
  out.zc = v.dist * den;
  out.dq = depthQ(out.zc);
  return true;
}

class ChaseCamera {
 public:
  void reset(const JetState& s, const World& w, int viewPreset);
  // dt is real (unscaled) time. shake in 0..1 (game owns decay). boost from JetState.
  void update(const JetState& s, const World& w, float dt, int viewPreset, float shake);
  void setCrashOrbit(bool on, const Vec3& wreck);
  const View& view() const { return v_; }

 private:
  View v_ = {};
  float tilt_ = 0.21f;      // eased base angle (rad)
  float tfix_ = 0.0f;       // eased terrain/cloud correction added to T
  float distEase_ = 88.0f;  // eased distance incl. boost pull
  float f_ = 88.0f, fVel_ = 0.0f;  // FOV spring
  float shakeT_ = 0.0f;
  bool orbit_ = false;
  Vec3 wreck_ = {0.0f, 0.0f, 0.0f};
  Vec3 prevPos_ = {0.0f, 0.0f, 0.0f};  // jet pivot at the previous update (follow prediction)
};

}  // namespace kj
```

### `src/core/render_world.h`

```cpp
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
```

### `src/core/draw.h`

```cpp
// draw.h - shared 2D/3D drawing primitives on the logical FrameBuf (depth-tested).
// OWNER: group C (draw.cpp). SHARED CONTRACT (used by C, D for rings/HUD helpers).
// alpha16: 0..16 Bayer coverage (16 = solid). Depth test: visible iff dq <= depth[p].
#pragma once
#include <stdint.h>
#include "types.h"
#include "frame.h"
#include "camera.h"

namespace kj {

// Single pixel, depth-tested. writeDepth: also store dq.
void drawPoint(FrameBuf& fb, int x, int y, uint8_t dq, uint8_t color, bool writeDepth);
// Bresenham line with linearly interpolated depth; thick 1 or 2 (2 = also x+1/y+1 neighbour).
// Definition is JET_HOT.
void drawLine(FrameBuf& fb, int x0, int y0, uint8_t dq0, int x1, int y1, uint8_t dq1,
              uint8_t color, int thick, int alpha16, bool writeDepth);
// World-space line: projects both ends (culls if either end is behind den < 0.15).
// Returns the number of pixels attempted.
int drawLine3D(FrameBuf& fb, const View& v, const Vec3& a, const Vec3& b, uint8_t color, int thick,
               int alpha16, bool writeDepth);
// Filled disc (r in px, r <= 0.5 draws one px), Bayer coverage alpha16, constant depth dq.
void drawDisc(FrameBuf& fb, int cx, int cy, float r, uint8_t dq, uint8_t color, int alpha16,
              bool writeDepth);
// Dithered glow ring between r0 and r1 (density falls off outward), no depth write.
void drawGlow(FrameBuf& fb, int cx, int cy, float r0, float r1, uint8_t dq, uint8_t color,
              int alpha16Inner);
// Screen-space rectangles without depth test (HUD / menus).
void drawRect(FrameBuf& fb, int x, int y, int w, int h, uint8_t color);
void drawRectDither(FrameBuf& fb, int x, int y, int w, int h, uint8_t color, int alpha16);

}  // namespace kj
```

### `src/core/jet_model.h`

```cpp
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
```

### `src/core/render_jet.h`

```cpp
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
```

### `src/core/fx.h`

```cpp
// fx.h - particles, wingtip trails, wake V, speed lines, in-cloud mist.
// OWNER: group C (fx.cpp). SHARED CONTRACT.
// All storage is fixed-size inside Fx (no allocation). Everything is drawn AFTER the jet,
// depth-tested against the frame depth (which already contains the jet), never writing depth.
#pragma once
#include <stdint.h>
#include "types.h"
#include "frame.h"
#include "camera.h"
#include "perf.h"
#include "config.h"

namespace kj {

struct JetState;
struct JetPose;
class World;

enum class Burst : uint8_t {
  RingPass,    // 16 sparks on the ring rim (pos = ring centre, dir = ring normal, strength = radius)
  Perfect,     // ring pass + extra white sparks
  CloudEntry,  // ring of 12 vapour puffs around the jet
  SkimTouch,   // spray burst at the jet
  Crash,       // fire 24 + debris 14 + smoke 10 (land)
  CrashSea,    // tall white spray column 40 + foam ring
  Ignite       // boost ignition puff at the nozzle
};

struct Particle {
  Vec3 p, v;
  float age, life;
  float size;       // u (projected with m)
  uint8_t kind;     // internal (smoke, spray, vapour, fire, debris, spark)
  uint8_t color;    // base palette index
  uint8_t pad[2];
};

struct TrailPoint {
  Vec3 p;
  float t;          // spawn time
};

struct WakeMark {
  Vec3 p;           // on the sea (z = 0)
  float ax, ay;     // across direction
  float t;          // spawn time
  float k;          // strength at spawn
};

class Fx {
 public:
  void reset();
  // dt is scaled sim time. inCloud from FlightEvents. Spawns smoke/spray/vapour/trails/wake.
  void update(const JetState& s, const JetPose& pose, const World& w, float dt, bool inCloud);
  void burst(Burst b, const Vec3& pos, const Vec3& dir, float strength);
  // Trails, wake, particles, speed lines. Depth-tested, no depth writes.
  void draw(FrameBuf& fb, const View& v, Tod tod, float time, RenderStats& st);
  // Full-frame dithered mist (pal::MIST) at coverage amount (0..1); frame = frame counter.
  void drawMist(FrameBuf& fb, float amount, uint32_t frame);
  float wakeStrength() const { return wake_; }

 private:
  Particle parts_[cfg::PARTICLES] = {};
  int nextPart_ = 0;
  TrailPoint trail_[2][cfg::TRAIL_POINTS] = {};
  int trailHead_[2] = {0, 0};
  float trailDist_ = 0.0f;
  WakeMark wake_marks_[cfg::WAKE_MARKS] = {};
  int wakeHead_ = 0;
  float wakeDist_ = 0.0f;
  Vec3 speed_[cfg::SPEEDLINES] = {};
  float smokeT_ = 0.0f, sprayT_ = 0.0f, vapourT_ = 0.0f;
  float wake_ = 0.0f;       // current wake strength 0..1
  float g_ = 0.0f;          // trail gauge copy
  float speedVis_ = 0.0f;   // speed-line visibility 0..1
  Vec3 vel_ = {0.0f, 0.0f, 0.0f};
  float now_ = 0.0f;
  uint32_t rng_ = 0x2545F491u;
};

}  // namespace kj
```

### `src/core/flight.h`

```cpp
// flight.h - arcade flight model (fixed 1/120 s sub-steps), crash detection.
// OWNER: group D (flight.cpp). SHARED CONTRACT (JetState is read by B, C, D).
#pragma once
#include <stdint.h>
#include "types.h"

namespace kj {

class World;

struct JetState {
  Vec3 pos;            // pivot position; x,y wrapped to [0, WORLD_U)
  Vec3 vel;            // u/s (includes roll dash)
  float yaw;           // heading psi (rad), forward = (sin, -cos, 0)
  float yawRate;       // rad/s
  float bank;          // visual bank (rad), + = right wing down
  float rollAngle;     // barrel-roll stunt angle (rad), added to bank for rendering
  float pitch;         // climb angle (rad), + = nose up; can pass pi/2 only in loops
  float pitchRate;     // rad/s
  float speed;         // u/s along the nose
  float boost;         // 0..1 eased boost amount
  float boostEnergy;   // 0..1 tank
  float energy;        // speed multiplier from climb/dive, 0.8..ENERGY_MAX
  float g;             // 0..1 "pull" gauge: drives trails and audio
  float rollT;         // 0..1 progress of the current barrel roll
  int8_t rollDir;      // -1 left, +1 right, 0 none
  int8_t rollQueued;   // one buffered roll request
  float rollCooldown;
  float loopT;         // 0..1 progress of an auto loop
  bool looping;
  float invuln;        // s of respawn invulnerability left
  float time;          // flight clock (s)
  float heightAboveSurface;  // pos.z - ground (or sea) height under the pivot
  bool overWater;
  bool rolling() const { return rollDir != 0; }
  float totalRoll() const { return bank + rollAngle; }
};

struct FlightInput {
  float turn;    // -1..1 (+ = right), already includes tilt + keys + assist, clamped
  float climb;   // -1..1 (+ = climb)
  bool boost;    // held
  int8_t rollReq;  // -1 / +1 request this step (edge), 0 none
  bool loopReq;    // edge
};

enum class CrashCause : uint8_t { None = 0, Terrain, Sea };

struct FlightEvents {
  bool skimTouch;      // kissed the sea without crashing (shake 0.35, spray, combo kept)
  bool boostIgnite;    // boost rising edge
  bool rollStart;
  bool closeCall;      // passed within CLOSE_CALL_DIST of terrain without crashing
  bool enteredCloud, exitedCloud;
  bool inCloud;
  bool pullUp;         // GPWS warning active
  bool loopDone;
};

class Flight {
 public:
  void reset(const Vec3& pos, float yaw);
  // One fixed sub-step. Returns the crash cause (None if alive). Ignores crashes while invuln > 0.
  CrashCause step(const FlightInput& in, const World& w, Assist assist, float dt, FlightEvents& ev);
  const JetState& s() const { return s_; }
  JetState& mut() { return s_; }
  // Predicted min gap to ground/sea along the current velocity, `sec` ahead.
  float predictedGap(const World& w, float sec) const;

 private:
  JetState s_ = {};
  float easyFloor_ = 0.0f;
  float closeCallCd_ = 0.0f;
  bool wasInCloud_ = false;
  float prevBoostCmd_ = 0.0f;
  bool boostDry_ = false;   // tank ran dry while engaged: needs a fresh boost press
  // private (group D): EASY deliberate-dive timer, sea-contact edge detection
  float diveHold_ = 0.0f;
  bool touching_ = false;
};

}  // namespace kj
```

### `src/core/tilt.h`

```cpp
// tilt.h - IMU tilt steering: gyro-fused gravity ("up") estimate, neutral calibration,
// angles relative to the neutral pose, soft deadzone + curve, output low-pass.
// OWNER: group D (tilt.cpp). Pure C++, used on device and in the simulator (synthetic IMU).
//
// Algorithm (DESIGN.md section 8):
//  1. predict  u <- normalize(u + (u x w) dt), w = (gyro - bias) in rad/s   (device frame,
//     right-handed; sign of the cross product verified by the sim's synthetic rotation test)
//  2. correct  u <- normalize(u + k (a/|a| - u)), k = 1 - exp(-dt/tau),
//     tau = TILT_TAU_STILL if ||a|-1| < TOL_FAST, TILT_TAU_MOVING if < TOL_SLOW, else skip.
//  3. neutral basis zn = n, xn = normalize(X - (X.n)n), yn = zn x xn
//     roll axis = device Y made perpendicular to xn: yd = normalize(Y - (Y.xn)xn), zd = xn x yd
//     rollDeg  = rollSign  * deg(atan2(-(u.xn), max(u.zd, 0.75 u.zn)))   + = right edge down
//                (exact for a roll about the device's own long axis at any hold pose; the
//                 0.75 u.zn floor bounds the over-read for rolls about the horizontal axis)
//     pitchDeg = pitchSign * deg(atan2(-(u.yn), u.zn))   + = top edge pushed away -> dive
//     climb input = -pitchShaped (invert flips it)
//  4. shape  u = clamp((|ang|-dz)/(full-dz), 0, 1); out = sign*u*(LIN + (1-LIN)*u)
//  5. output one-pole low-pass at TILT_OUT_HZ.
#pragma once
#include <stdint.h>
#include "types.h"
#include "input.h"

namespace kj {

struct TiltOut {
  float turn;      // -1..1, + = right
  float climb;     // -1..1, + = climb
  float rollDeg;   // relative to neutral, signed (debug / HUD dot)
  float pitchDeg;
  bool valid;      // calibrated, fresh data, inside the guard range
};

class TiltController {
 public:
  void configure(Sens sens, bool invert, bool drift, int8_t rollSign, int8_t pitchSign);
  // Start a calibration window (TILT_CAL_STILL_S of stillness within windowSec timeout).
  void beginCalibration(float windowSec);
  bool calibrating() const { return calibrating_; }
  float calibProgress() const;      // 0..1 for the on-screen ring
  bool calibrated() const { return calibrated_; }
  bool calibTimedOut() const { return timedOut_; }   // fell back to the stored neutral
  void update(const ImuSample& s, float dt);
  TiltOut out() const { return out_; }
  Vec3 neutral() const { return n_; }
  void setNeutral(const Vec3& n);   // from NVS
  Vec3 up() const { return u_; }
  Vec3 gyroBias() const { return bias_; }

 private:
  Sens sens_ = Sens::Med;
  bool invert_ = false, drift_ = true;
  int8_t rollSign_ = 1, pitchSign_ = 1;
  Vec3 u_ = {0.0f, 0.0f, 1.0f};
  Vec3 n_ = {0.0f, 0.57f, 0.82f};   // typical "held toward the face" neutral until calibrated
  Vec3 bias_ = {0.0f, 0.0f, 0.0f};
  bool initialised_ = false;
  bool calibrating_ = false, calibrated_ = false, timedOut_ = false;
  float calWindow_ = 0.0f, calElapsed_ = 0.0f, stillTime_ = 0.0f;
  Vec3 calSumA_ = {0.0f, 0.0f, 0.0f}, calSumG_ = {0.0f, 0.0f, 0.0f};
  int calN_ = 0;
  float staleT_ = 0.0f, driftHold_ = 0.0f;
  Vec3 nCal_ = {0.0f, 0.57f, 0.82f};   // neutral at last calibration (drift limit reference)
  TiltOut out_ = {0.0f, 0.0f, 0.0f, 0.0f, false};
};

}  // namespace kj
```

### `src/core/course.h`

```cpp
// course.h - ring gates: chained placement, pass/miss detection, magnet assist, drawing,
// off-screen guide. OWNER: group D (course.cpp). Uses draw.h primitives from group C.
#pragma once
#include <stdint.h>
#include "types.h"
#include "frame.h"
#include "camera.h"
#include "perf.h"
#include "config.h"

namespace kj {

struct JetState;
class World;

enum class RingKind : uint8_t { Low = 0, Mid, High };
enum class RingResult : uint8_t { None = 0, Through, Perfect, Missed };

struct Ring {
  Vec3 c;          // centre (wrapped)
  float yaw;       // ring normal = forward(yaw)
  float radius;    // u
  RingKind kind;
  bool live;
  float passAnim;  // 0 = idle; >0 seconds since passed (scale 1 -> 1.6 + fade over 0.3 s)
};

struct RingEvent {
  RingResult result;
  float lateral;     // distance from the centre at the plane crossing (u)
  uint16_t index;    // ring number in this run (1-based)
  Vec3 pos;          // ring centre
  Vec3 normal;
  RingKind kind;
};

class Course {
 public:
  void reset(const JetState& s, const World& w, uint32_t seed);
  // prevPos: jet pivot at the start of this frame (wrapped). Returns at most one event per call.
  RingEvent update(const Vec3& prevPos, const JetState& s, const World& w, float dt);
  // Magnet assist (scaled by cfg::ASSIST_GAIN[assist] and by (1 - |player input|)).
  void assist(const JetState& s, Assist a, float playerTurn, float playerClimb, float& turnAdd,
              float& climbAdd) const;
  // Current ring amber (pulsing), next cyan (dithered), far one = beacon dot + light column.
  void draw(FrameBuf& fb, const View& v, Tod tod, float time, RenderStats& st) const;
  bool current(Ring& out) const;
  uint16_t passed() const { return passed_; }
  // Off-screen indicator for the current ring: edge position + vertical hint (-1 below, 0, +1 above).
  bool guide(const View& v, int& edgeX, int& edgeY, int& vertical) const;

 private:
  Ring rings_[cfg::LIVE_RINGS] = {};
  int cur_ = 0;
  uint16_t passed_ = 0, spawned_ = 0;
  float sinceProgress_ = 0.0f, lastDist_ = 0.0f;
  uint32_t seed_ = 1;
  int8_t lastTurnSign_ = 1;
  // private (group D): the ring just passed, kept for its 0.3 s pass animation
  Ring ghost_ = {};
};

}  // namespace kj
```

### `src/core/hud.h`

```cpp
// hud.h - 3x5 pixel font, HUD overlay, popups, menus, title/calibration/game-over screens.
// OWNER: group D (hud.cpp, font3x5.h). Draws in LOGICAL pixels with pal::HUD_* indices only
// (HUD is exempt from flash/mist/fade). No depth test.
#pragma once
#include <stdint.h>
#include "frame.h"

namespace kj {

constexpr int FONT_W = 3, FONT_H = 5, FONT_ADV = 4;  // glyph advance incl. 1 px spacing

// Printable ASCII 32..95 (upper case; lower case is folded to upper) + a few symbols.
void hudText(FrameBuf& fb, int x, int y, const char* s, uint8_t color, bool shadow = true,
             int scale = 1);
int hudTextWidth(const char* s, int scale = 1);
void hudTextCentered(FrameBuf& fb, int cx, int y, const char* s, uint8_t color, bool shadow = true,
                     int scale = 1);

enum class HudScreen : uint8_t { None, Calibrate, Title, Countdown, Pause, Help, GameOver };
enum class TiltBadge : uint8_t { Keys, TiltOk, NeedsCal, NoImu, Guard };

constexpr int HUD_POPUPS = 4;
struct Popup {
  char text[12];
  float age;        // s; removed at 0.8 s
  int16_t x, y;     // spawn position (logical px), rises 6 px over its life
  uint8_t color;
};

constexpr int MENU_ITEMS = 9;
struct HudData {
  HudScreen screen;
  // play HUD
  bool showPlay;
  uint32_t score;
  uint8_t combo;           // 1..8
  float comboPulse;        // 0..1, decays
  float timeLeft;          // s; < 0 hides the timer (free flight)
  uint16_t ring;           // rings passed
  float speedKt;           // displayed speed
  float alt;               // height above surface (u)
  float boostEnergy;       // 0..1
  bool skimming;
  bool pullUp;
  TiltBadge tilt;
  uint8_t batteryPct;      // icon shown below 15
  // ring guide (off-screen chevron)
  bool guide;
  int16_t guideX, guideY;
  int8_t guideVertical;    // -1 below, 0, +1 above
  // stats overlay (f)
  bool stats;
  float fps, coreMs;
  // IMU debug overlay (g) and calibration screen
  bool imuDebug;
  float dotX, dotY;        // -1..1 tilt dot (turn, -pitch)
  float calibProgress;     // 0..1
  bool calibTimedOut;
  // popups and centre message
  Popup popups[HUD_POPUPS];
  char centre[16];         // e.g. "PERFECT" "PULL UP" "CRASH" "GAME OVER"; empty = none
  float centreAge;
  // title / game over
  uint32_t best;
  uint32_t lastScore;
  uint16_t lastRings;
  uint8_t bestCombo;
  bool newBest;
  int8_t countdown;        // 3,2,1,0(GO)
  // pause menu
  int8_t menuSel;
  const char* menuLabels[MENU_ITEMS];
  const char* menuValues[MENU_ITEMS];
  float time;              // for blinking
};

void drawHud(FrameBuf& fb, const HudData& h);

}  // namespace kj
```

### `src/core/audio.h`

```cpp
// audio.h - portable 16 kHz mono synth (engine, roar, wind, spray + one-shot voices).
// OWNER: group D (audio_synth.cpp). The platform (group A) only pumps Synth::render() into
// M5.Speaker.playRaw on one channel; the simulator writes the same samples to a WAV.
// Thread model: setParams/trigger/render are all called from the game loop task (no locks).
#pragma once
#include <stdint.h>

namespace kj {

enum class Sfx : uint8_t {
  Chime = 0,   // ring pass; pitch = 2^(combo/12)
  Perfect,     // chime + octave
  Miss,        // low two-note
  Roll,        // band-sweep swish 600 -> 3000 Hz, 0.4 s
  Ignite,      // boost thump + engine overshoot
  Slap,        // skim touch
  Whoosh,      // cloud entry
  Crash,       // noise burst tau 0.5 s + 90 Hz drop
  PullUp,      // 1000 Hz square 80 ms (game retriggers every 0.16 s while warning)
  Tick,        // timer < 5 s
  Blip,        // menu / countdown
  Go,          // countdown end
  Count
};

struct AudioParams {
  float speedRatio;   // speed / V_CRUISE
  float boost;        // 0..1
  float wind;         // 0..1.6 (dives louder)
  float spray;        // wake strength 0..1
  bool inCloud;
  bool engineOn;      // false on title-less screens / crash
};

class Synth {
 public:
  static constexpr int RATE = 16000;
  void init(uint32_t seed);
  void setParams(const AudioParams& p) { target_ = p; }
  void trigger(Sfx s, float pitch = 1.0f);
  // Render n mono samples (int16). Parameters ramp linearly across the block (no zipper).
  void render(int16_t* out, int n);
  void setMute(bool m) { mute_ = m; }
  bool muted() const { return mute_; }

 private:
  static constexpr int VOICES = 4;
  // s2: second filter state (group D private addition)
  struct Voice { uint8_t sfx; float t, pitch, phase, state; bool on; float s2; };
  AudioParams target_ = {1.0f, 0.0f, 0.0f, 0.0f, false, false};
  AudioParams cur_ = {1.0f, 0.0f, 0.0f, 0.0f, false, false};
  Voice voices_[VOICES] = {};
  float ph1_ = 0.0f, ph2_ = 0.0f, lp1_ = 0.0f, lp2_ = 0.0f, bp1_ = 0.0f, bp2_ = 0.0f, hp_ = 0.0f;
  uint32_t noise_ = 0x1u;
  bool mute_ = false;
  // private (group D): wind band filter, spray bursts, smoothed cloud / engine gates
  float wd1_ = 0.0f, wd2_ = 0.0f, hpIn_ = 0.0f;
  float burstT_ = 0.0f, burstAmp_ = 0.5f;
  float cloud_ = 0.0f, eng_ = 0.0f;
};

}  // namespace kj
```

### `src/core/expand.h`

```cpp
// expand.h - 2x nearest-neighbour palette expand of the 120x68 index frame to the 240x135
// big-endian RGB565 panel frame. Used by the device present path AND the simulator PNG path,
// so screenshots show exactly what the panel shows.
// OWNER: group A (expand.cpp).
#pragma once
#include <stdint.h>

namespace kj {

// Full frame: out = 240*135 uint16 (64,800 B), 4-byte aligned. Logical row y -> output rows
// 2y and 2y+1, except logical row 67 which emits only output row 134.
// Per row: 120 x uint32 stores of lut2x[px] then memcpy of the 480 B row. Definition JET_HOT.
void expand2x(const uint8_t* px, const uint32_t* lut2x, uint16_t* out);

// Band variant for the DMA-allocation fallback path: writes output rows [outRow0, outRow1)
// into `out` (which holds (outRow1-outRow0)*240 pixels).
void expand2xRows(const uint8_t* px, const uint32_t* lut2x, uint16_t* out, int outRow0,
                  int outRow1);

}  // namespace kj
```

### `src/core/game.h`

```cpp
// game.h - top-level game: state machine, input mapping, subsystem orchestration,
// render order, persistence, telemetry, debug hooks for the simulator.
// OWNER: group D (game.cpp). SHARED CONTRACT with group A (platform/main) and E (sim).
//
// There is exactly ONE Game instance, a static global (it embeds ~20 KB of state; the big
// world/frame arrays are static in their own .cpp files). Never allocate a Game on the stack.
//
// Per-frame call sequence (device main.cpp and the simulator use the SAME order):
//   plat input -> game.update(in) -> game.render() -> audio pump (game.synth())
//   -> present(game.frame().px, game.palette().lut2x()) -> game.profiler().endFrame()
#pragma once
#include <stdint.h>
#include "config.h"
#include "types.h"
#include "input.h"
#include "frame.h"
#include "perf.h"
#include "palette.h"
#include "world.h"
#include "camera.h"
#include "flight.h"
#include "tilt.h"
#include "course.h"
#include "jet_model.h"
#include "render_jet.h"
#include "fx.h"
#include "hud.h"
#include "audio.h"

namespace kj {

enum class GameState : uint8_t { Boot, Calibrate, Title, Countdown, Play, Pause, Crash, GameOver };
enum class Mode : uint8_t { RingRun = 0, Free = 1 };

constexpr uint32_t PERSIST_VERSION = 1;
// Saved in NVS namespace "moonskim" as one blob (key "p"). Platform owns the storage;
// the game owns the content and asks for saves through PlatformApi::save (debounced 2 s,
// never mid-flight except settings changes from the pause menu / serial).
struct Persist {
  uint32_t version;
  uint32_t best;          // best Ring Run score
  uint8_t bestCombo;      // best Free Flight combo
  uint8_t tod;            // Tod
  uint8_t tiltOn;         // 1 = tilt steering enabled
  uint8_t sens;           // Sens
  uint8_t invert;         // invert pitch
  uint8_t assist;         // Assist
  uint8_t volume;         // 0..255
  uint8_t bright;         // 0..255
  uint8_t view;           // camera preset 0..2
  uint8_t mute;
  uint8_t drift;          // neutral-pitch drift on
  int8_t rollSign, pitchSign;  // tilt sign flags (serial r/p)
  uint8_t pad[3];
  float nx, ny, nz;       // last calibrated neutral up-vector
};

inline Persist defaultPersist() {
  Persist p = {};
  p.version = PERSIST_VERSION;
  p.tod = (uint8_t)Tod::Night;
  p.tiltOn = 1;
  p.sens = (uint8_t)Sens::Med;
  p.assist = (uint8_t)Assist::Normal;
  p.volume = cfg::VOLUME_DEFAULT;
  p.bright = cfg::BRIGHT_DEFAULT;
  p.drift = 1;
  p.rollSign = cfg::ROLL_SIGN_DEFAULT;
  p.pitchSign = cfg::PITCH_SIGN_DEFAULT;
  p.nx = 0.0f; p.ny = 0.57f; p.nz = 0.82f;
  return p;
}

// Services the core needs from the platform (device or simulator).
struct PlatformApi {
  uint32_t (*micros)();                 // monotonic microseconds
  void (*log)(const char* line);        // one complete line, no trailing newline needed
  void (*save)(const Persist& p);       // persist now (game already debounced)
};

struct Telemetry {
  GameState state;
  Mode mode;
  uint32_t score;
  uint8_t combo;
  float timeLeft;
  uint16_t rings;
  Vec3 pos;
  float yaw, speed, agl, boost, boostEnergy;
  float tiltRoll, tiltPitch, turn, climb;   // tilt angles (deg) and final stick values
  bool tiltOn, tiltValid, calibrated;
  Vec3 up, neutral;                          // tilt filter state (for the [IMU] line)
  Tod tod;
};

class Game {
 public:
  void init(const PlatformApi& api, const Persist& p, uint32_t seed, bool imuPresent);
  void update(const InputFrame& in);
  void render();

  const FrameBuf& frame() const { return fb_; }
  const Palette& palette() const { return pal_; }
  Synth& synth() { return synth_; }
  Profiler& profiler() { return prof_; }
  Telemetry telemetry() const;
  const Persist& persist() const { return persist_; }

  // Settings the platform applies when they change (brightness PWM, speaker volume).
  uint8_t brightness() const { return persist_.bright; }
  uint8_t volume() const { return persist_.volume; }
  bool muted() const { return persist_.mute != 0; }
  bool imuStream() const { return imuStream_; }   // [IMU] serial stream requested ('g' key / serial 'i')

  // Single-char serial commands handled by the core (see DESIGN.md section 14):
  //  'r' flip roll sign, 'p' flip pitch sign, 'c' recalibrate, 't' tilt on/off, 'n' time of day,
  //  'a' autopilot on/off, 'b' 20 s bench, 'x' crash+respawn, 'X' reset best, 'f' stats overlay,
  //  'i' IMU stream on/off, 'm' dump jet model ASCII. Returns false if not a core command.
  bool command(char c);

  // ---- simulator / debug hooks
  void debugStart(Mode m);                                   // skip calibration+countdown
  void debugTeleport(float x, float y, float z, float yawRad);
  void setTod(Tod t);
  void setAutopilot(bool on);
  void setView(int preset);
  // Named values for sim asserts: "alt","agl","speed","score","combo","crashed","rings",
  // "horizon","state","boost","time_left","jet_w" (jet screen bbox width). false if unknown.
  bool debugVar(const char* name, float& out) const;
  const World& world() const { return world_; }
  const JetState& jet() const { return flight_.s(); }
  const View& view() const { return cam_.view(); }

 private:
  // subsystems
  PlatformApi api_ = {nullptr, nullptr, nullptr};
  Persist persist_ = {};
  World world_;
  Palette pal_;
  JetModel jet_;
  Flight flight_;
  ChaseCamera cam_;
  TiltController tilt_;
  Course course_;
  Fx fx_;
  Synth synth_;
  Profiler prof_;
  FrameBuf fb_ = {nullptr, nullptr};
  HudData hud_ = {};
  // state
  GameState state_ = GameState::Boot;
  Mode mode_ = Mode::RingRun;
  uint32_t seed_ = 1;
  float clock_ = 0.0f, stateT_ = 0.0f, timeScale_ = 1.0f, hitStop_ = 0.0f, simAcc_ = 0.0f;
  float shake_ = 0.0f, flash_ = 0.0f, mist_ = 0.0f;
  uint32_t flashRgb_ = 0xFFF4E0u;
  uint32_t score_ = 0;
  uint8_t combo_ = 1, bestComboRun_ = 1;
  float timeLeft_ = 0.0f, skimTick_ = 0.0f;
  float keyTurn_ = 0.0f, keyClimb_ = 0.0f;
  uint64_t prevKeys_ = 0;
  uint32_t acts_ = 0, actsPressed_ = 0;
  bool imuPresent_ = false, imuStream_ = false, autopilot_ = false, statsOn_ = false;
  bool helpOn_ = false, pausedFromPlay_ = false, newBest_ = false;
  float saveTimer_ = -1.0f;
  int8_t menuSel_ = 0;
  Vec3 prevPos_ = {0.0f, 0.0f, 0.0f};
  Vec3 wreck_ = {0.0f, 0.0f, 0.0f};
  float pullUpBeep_ = 0.0f;
  uint32_t frameNo_ = 0;
  float benchT_ = -1.0f;
  FlightEvents lastEv_ = {};
  Rect jetBounds_ = {0, 0, 0, 0};
  uint32_t jetBoundsFrame_ = 0xFFFFFFF0u;  // frameNo_ when jetBounds_ was last set

  // ---- private helpers and extra state (group D; not part of the contract)
  void enterState(GameState s);
  void startRun(Mode m, bool skipCountdown);
  void placeAttractJet();
  void respawn();
  void crash(CrashCause c);
  void gameOver();
  void updateAttract(float rdt);
  void updatePlay(float rdt);
  void updatePause();
  void handleGlobalKeys();
  void handleEvents(const FlightEvents& ev, bool scoring);
  void handleRing(const RingEvent& e);
  CrashCause runFlight(float sdt, FlightInput fi, FlightEvents& ev, Assist assist);
  FlightInput playerInput(float rdt);
  FlightInput autopilotInput(float rdt, bool useRings);
  void updateMist(float rdt, bool inCloud);
  void onCalibrationDone();
  void startRecal();
  void applyTiltConfig();
  void requestSave();
  void saveNow();
  void addPopup(const char* text, int x, int y, uint8_t color);
  void setCentre(const char* text);
  void buildHud();
  void buildMenu();
  void menuAdjust(int item, int dir);
  void benchTick(float rdt);
  void startBench();
  JetPose pose() const;
  void logf(const char* fmt, ...) const;
  float lastDt_ = 0.0f, assembleT_ = 1.0f, fade_ = 0.0f, slowmoT_ = 0.0f, attractT_ = 0.0f;
  float imuStaleT_ = 0.0f, apRollT_ = 10.0f, stickTurn_ = 0.0f, stickClimb_ = 0.0f;
  bool staleLogged_ = false, calWas_ = false, recal_ = false, calSkipped_ = false;
  bool bootCal_ = false, holdStillMsg_ = false, pendingLoop_ = false, benchPrevAuto_ = false;
  bool countdownCal_ = false;               // the running calibration is the countdown re-zero
  Vec3 cdPrevN_ = {0.0f, 0.57f, 0.82f};     // neutral before the countdown re-zero
  float recalT_ = 0.0f;                     // time since the in-play 'c' recalibration began
  int8_t lastAutoRoll_ = 1, pendingRoll_ = 0, countdownDigit_ = 3;
  uint16_t ringsRun_ = 0;
  uint32_t crashes_ = 0, skimTouches_ = 0, runCount_ = 0;
  uint8_t batteryPct_ = 0;
  uint32_t benchFrames_ = 0, benchWinFrames_ = 0;
  float benchWinT_ = 0.0f, benchFpsMin_ = 0.0f, benchAvg_ = 0.0f;
  char menuVal_[MENU_ITEMS][10] = {};
};

}  // namespace kj
```

### `src/platform/cardputer/plat.h`

```cpp
// plat.h - Cardputer-Adv platform layer (Arduino + M5Cardputer/M5Unified/M5GFX).
// OWNER: group A. The ONLY code that touches M5.Display, M5.Imu, the keyboard, M5.Speaker,
// Preferences and Serial. Threads after setup (one owner per resource):
//   core 1  Arduino loop task: the game, Serial (log ring + commands), Speaker pump, NVS.
//   core 0  "present" task (prio 5): every M5.Display call; holds the SPI transaction.
//   core 0  "input" task (prio 3): every I2C transfer (TCA8418 keyboard, BMI270), BtnA,
//           battery ADC. The ES8311 is configured over I2C only inside Speaker.begin (setup).
//   core 0  M5 speaker mixer task (prio 2).
#pragma once
#include <stdint.h>
#include "core/config.h"
#include "core/input.h"
#include "core/game.h"

namespace plat {

// M5Cardputer.begin (fallback board = ADV, mic off), Serial (non-blocking), display
// (landscape = rotation 0 on M5GFX 0.3.0, brightness, optional 80 MHz bus), DMA frame
// buffer (64,800 B, MALLOC_CAP_DMA | MALLOC_CAP_INTERNAL, allocated FIRST), speaker (task
// pinned to core 0, begin()), NVS load,
// boot log lines [BOOT]/[MEM]/[DISP]/[IMU]/[KBD]/[SPK]/[INPUT]. Starts the input task and the
// present task (which opens startWrite() for good and primes the DMA).
// Returns false only if nothing can be displayed (then main halts with a red screen + [ERR]).
bool begin(kj::Persist& outPersist);

bool isAdv();          // M5.getBoard() == board_M5CardputerADV
bool imuPresent();     // ADV and M5.Imu.getType() != none

// Takes the latest snapshot of the input task (keys down + every key seen since the last
// call, BtnA + wasPressed, newest IMU sample with fresh = new since the last call, battery);
// never waits for I2C. One char from Serial (platform commands are consumed here; core
// commands are forwarded in in.serialCmd). Records ST_INPUT (~0.02 ms).
void pollInput(kj::InputFrame& in, kj::Game& game);

// Start one device poll on the input task (core 0, ~1 ms of I2C). main.cpp calls it from the
// pacing wait ~1.5-2.6 ms before the next frame (right away when late or uncapped), so the
// poll is complete when pollInput runs. Harmless if a poll is still running.
void inputKick();

// Hands the frame to the present task (core 0): blocks (no spin) until the staging buffer is
// free (-> ST_DMAWAIT, 0 unless display-bound), copies px + lut2x into it (-> ST_EXPAND,
// ~0.03 ms) and returns. The task expands and pushes it while core 1 renders the next frame.
void present(const uint8_t* px, const uint32_t* lut2x, kj::Profiler& prof);

// Refill the speaker stream: while M5.Speaker.isPlaying(ch) < 2, synth.render() the next of 3
// rotating buffers and playRaw(buf, 384, 16000, false, 1, ch, false). Records ST_AUDIO.
void audioPump(kj::Synth& synth, kj::Profiler& prof);

// Apply brightness / volume / mute when they differ from the last applied values.
void applySettings(const kj::Game& game);

void savePersist(const kj::Persist& p);   // Preferences "moonskim", key "p" (blob)
void logLine(const char* s);              // core 1 only; queued in a 4 KB ring (see plat_log)
void logFlush();                          // move queued log bytes to USB (non-blocking)
uint32_t micros32();

// Once-per-second [STAT] line, [IMU] stream (20 Hz when game.imuStream()), [KBD] key echo.
void periodicLogs(kj::Game& game, float dt);

// Serial screenshot: [SHOT] begin w=120 h=68 fmt=idx8+pal888, base64 lines, [SHOT] end crc=.
void dumpShot(const uint8_t* px, const uint8_t* rgb888);

}  // namespace plat
```
