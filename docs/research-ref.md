# Technique spec: sprite-stacked jet over islands, chase view (from the reference demo)

Source studied: a local copy of the reference web demo's source (jet-model.ts, voxel-sprite.ts, flame-layer.ts and post.ts read in full; scene.ts read for constants, palettes, buildWorld, updateView, step, axes/project, drawWorld, drawSky, trails, wake, smoke, drawJet and render; the .tsx read for input and layout). I did not run or follow anything in it. Everything below is paraphrased: formulas and tuning numbers, no code and no voxel data.

Units: 1 world unit = 1 jet voxel. The reference jet is 34 wide (x), 46 long (y) and 13 slices tall (z). Its speed of 92 u/s is about 2 jet lengths per second.

---

## 0. TL;DR: what makes the look and feel

1. **The world is drawn by a voxel-space column renderer (front to back, Comanche-style)** through a level camera with a lens shift. The camera has no true pitch, so vertical stays vertical and the horizon is always a horizontal row. Banking rolls the jet, not the horizon, in the chase view.
2. **The heightmap is terraced** (quantised height levels), with a material per cell. This gives a crisp pixel-art look, with tops and walls in two colours.
3. **The jet is a voxel splat ordered by slices.** Every voxel goes through a full 3D yaw/pitch/roll transform and is drawn as a small block of sub-samples, so rolls are real 3D. It has 3 tone palettes picked per wing half from the sun direction, plus a 1 px dark outline.
4. **A 4x4 Bayer ordered dither is used everywhere** for alpha and gradients (sky band edges, trails fading, foam, sun glow). It maps well to RGB565.
5. **Most of the "feel" comes from exponential easing.** Yaw rate eases at 2.6/s, bank at 5/s, boost at 5 up and 2.4 down, and camera heading lags at 3.4/s, so you see the jet's flank in turns. Boost raises speed ×1.8, moves the camera 16 % closer, widens the FOV and lengthens the flames ×2.6.
6. **Haze and fog come in 10 quantised steps toward the horizon colour**, which gives a cheap sense of depth.

---

## 1. Camera model (chase view)

### Projection: one formula for all views
For a world point (x, y, z), take the camera focus point F = (cam.x, cam.y, cam.alt) and the view heading ψ (with cosψ and sinψ):
- across = dx·cosψ − dy·sinψ
- depth = −(dx·sinψ + dy·cosψ), measured from the focus point (positive is away from the camera)
- den = 1 + q·depth. Cull the point if den < 0.12 (the jet uses 0.2).
- m = S / den. This is the pixels per world unit at that depth.
- screenX = Cx + m·across
- screenY = Cy − m·((z − cam.alt) + T·depth)

Here q = 1/chaseDistance, S = the scale at the jet in px per unit, and T = the "tan" (look-down slope).

**Interpretation, the key insight.** This is a level pinhole camera:
- It sits 1/q units behind the focus and T/q units above it.
- Its focal length is f = S/q px.
- Its lens is shifted vertically, so the horizon is at row Y_h = Cy − f·T.
- There is no camera pitch. Looking up or down is done by changing T, which slides the horizon.

Because of this, the sprite-stack slices stay horizontal on screen, and the column renderer stays valid because world verticals stay screen verticals. With q = 0 and T = 1 you get the classic flat 45° overhead sprite-stack view (each layer 1 px up). The demo slides q, S, T and ψ to morph between its views.

### Numbers (defaults)
- chaseDistance = 2.6 jet lengths, so dist = 2.6 × 46 ≈ 120 u. Under boost: dist × (1 − 0.16·boost).
- Jet width on screen = 0.19 × W. So S = max(1, 0.19·W / 34) px per voxel. On a 240-wide screen that is about 1.34 px per voxel, and the jet is about 46 px wide.
- Horizontal FOV is independent of resolution: tan(hFOV/2) = 34 / (2·0.19·2.6·46) ≈ 0.748, so hFOV ≈ 73.6°. Equivalently f = (W/2)/0.748, which is about 160 px at W = 240.
- Chase look-down angle is 12°, so T = tan(12°) ≈ 0.213.
- Climbing tilts the view up the sky: T = tan(clamp(12° − 0.9·sin(pitch), −1.2, 1.2)). The horizon drops in a climb and rises in a dive.
- The jet's screen row is Cy = H/2 + H·(0.14 − 0.03·boost), so it sits low, about 64 % down. Cx = W/2.
- Camera height above the focus is T·dist ≈ 25 u. For W = 240 and H = 135 the horizon lands at about row 86 − 160·0.213 ≈ 52, so roughly the top 39 % of the screen is sky.
- Camera roll is 0 in the chase view. Only the cockpit view rotates the frame (it draws on an oversized square sheet, then rotates it).

### Following and lag
- **Focus point** follows the jet with velocity feed-forward plus a correction: F += v·dt + (target − (F + v·dt))·(1 − e^(−3.5·dt)). It stays essentially on the jet.
- **Altitude**: F.alt += climbRate·dt·(0.6 … 0.97, more when the climb is steep), then F.alt eases toward jet.z at rate 3/s. The jet drifts slightly up or down the screen in climbs and dives.
- **Heading**: chaseHeading eases toward −jet.yaw at rate 3.4/s. In a steady 1.25 rad/s turn it lags about 0.37 rad (≈ 21°), so the jet swings round and shows its side and belly in turns. This sells the banking.
- **Terrain and cloud avoidance**: the camera ground position is F − forward·dist. Sample the ground max over a 3×3 grid at ±9 u there, and set the camera floor = that + 5. If the camera's height would fall inside the cloud slab [CLOUD_ALT − 3, top + 3], push it below (when the jet is below the clouds) or above. Implement this as an extra T offset (fix·q), eased at rate 7/s.

---

## 2. World rendering

### Representation
- A wrapping heightmap, N = 2048², 1 byte height (max ~40) plus 1 byte material.
- A separate wrapping cloud map, 1024², 1 byte per cell: (rank << 4) | thickness 0..8.
- A 256² byte noise table for animation.

### Renderer: front-to-back "voxel space" with a per-column occlusion row
- **Setup**: colY[x] is the highest screen row filled so far in column x, starting at the bottom row. The framebuffer is cleared to 0, meaning "empty".
- **Depth loop**: march depth v from near to far. The step is about one screen row of sea per step: dv = den² / (S·(T + q·alt)), capped at 2.5·den/S (so a step is never coarser than a few px across) and at least 0.5.
- **Per depth step**: precompute the screen row for every height 0..ZTOP, rowNow[z] = floor(R − m·z) with R = Cy − m·(T·v − alt). Keep the previous step's rows in rowPrev.
- **Ground sampling**: walk the scanline across screen x affinely. The world position starts at the left edge and advances by (cosψ, −sinψ)·(den/S) per pixel.
- **Per column**, with h = height[cell]:
  - Wall: from rowPrev[h] up to colY[x], in the side colour.
  - Top: from rowNow[h] up to the new colY, in the top colour.
  - Then set colY[x] to the top row. Only write pixels that are still empty.
- **Early-outs**: skip the column when both rows are already below colY. The march starts where the tallest possible thing touches the bottom row and stops at the far distance or where the ground leaves the top of the frame.
- **Depth buffer**: a per-pixel depth value v is written. This is used later to hide the jet, trails and smoke behind hills and clouds. It is cheap to keep as a 16-bit value.
- **Distances**: viewDist = 1500 + 3·alt, and fog starts at 300 + 2·alt.
- **Fog**: f = (v − fogStart)/(viewDist − fogStart). fogLevel = round(sqrt(f)·10)/10, giving 10 quantised steps that are stronger near the horizon. colour = lerp(colour, haze, fogLevel).
- **Remaining pixels**: after the world pass, empty pixels above the horizon row get sky. Empty pixels below it get the flat haze colour, which acts as the far sea.
- **Cost**: about (number of depth steps ≈ 1–1.5 × rows below the horizon) × W samples, each with one terrain and one cloud lookup. At 240×135 that is about 20–40k samples per frame. On an ESP32-S3 that is fine (≈ 5–10 ms) if the inner loop is integer and fixed-point.

### Island generation
- **Noise**: tileable value noise (smoothstep-interpolated lattice), 6 octaves.
  - Cell sizes 512, 256, 128, 64, 32, 16.
  - Amplitudes 0.55, 1, 0.5, 0.25, 0.11, 0.05. The 256 octave dominates, which gives islands a few hundred units across.
  - Normalise to 0..1.
- **Sea level**: set from a histogram so that a fraction `land` (0.33) of cells is above it.
- **Below sea**, by depth d below sea level (with a tiny hash jitter):
  - > 0.07: deep
  - > 0.03: mid
  - > 0.008: shallow
  - else: surf
  - Height 0 for all of them.
- **Beach**: 0 < d < 0.012 is sand at height 0, with a ~13 % hash "speck" variant.
- **Land**: hr = (d − 0.012)/(1 − sea − 0.012), terraced into 15 levels.
  - hr < 0.5: 8 grass levels, 2 u each.
  - 0.5–0.78: 4 rock levels, steps of 0.07 hr, 3 u each.
  - Above that: snow levels, steps of 0.09 hr, 3 u each.
  - Height = lv·2 for lv ≤ 8, otherwise 16 + (lv − 8)·3, capped near 37.
  - The terraces are what give the stepped pixel-art cliffs.
- **Slope lighting baked into the material**: g = E(x−2, y−2) − E(x+2, y+2), a NW–SE difference. g < −0.007 means lit (light variant). g > 0.007 means shaded (dark variant). Otherwise use the normal variant, with some hash noise sprinkled in.
- **Trees**: on a 5-cell jittered grid over grass with h ≤ 14, gated by a second noise sample and a hash. Each tree is a cone of radius 2–3 cells that raises height by up to r+2 above the ground, coloured tree-top, tree and tree-edge by distance from the centre.
- **Spawn**: open water with some land within 120 u.

### Ocean
- Sea colour depends only on the depth band (deep, mid, shallow, surf).
- **Surf**: on the surf band, animated foam where (noise[cell] + 36·t) mod 256 < 120. That gives a flickering shoreline.
- **Glints**: only near the camera (when units per pixel < 1.8), unshaded water only. Do two noise lookups scrolling in opposite directions (6 u/s and 4 u/s), each stretched 4× along x. If n1 > 243 and n2 > 128, draw a glint: sparkB when n1 > 252, otherwise sparkA. The result is sparse twinkling dashes.
- **Shading**: there is no per-pixel wave shading otherwise. Shade (cloud or jet shadow) multiplies the colour by (0.62, 0.67, 0.80), a bluish shadow.

### Shadows
- The sun direction is a ground offset per unit of height, (sunX, sunY):
  - day (0.5, 0.32)
  - sunset (0.95, 0.5)
  - night (0.3, 0.42), the moon
- **Cloud shadow**: sample the cloud map at the cell offset by sun·CLOUD_ALT, drifting with the clouds. Thickness > 1 shades the cell.
- **Jet shadow**: for each terrain cell near the jet, take (cell − (jet.xy + sun·(jet.z − h))). Rotate it into the jet's frame (inverse yaw), and divide the across axis by max(0.3, |cos roll|) so the shadow narrows when banked. Look up a precomputed 34×46 top-down silhouette mask. If the cell is inside, use the shaded colour.
- This is cheap: a bounding-box test first, and only cells within ±0.75 jet length of the shadow centre are tested.

---

## 3. Sky, sun and moon, stars

- **Gradient**: 8 colour bands from the horizon upward. Interpolate horizon colour A → mid colour M over the lower half of the bands, then M → top colour B. The bands span 0.75·H rows above the horizon.
  - Band index t = (horizon − y)/span·7.
  - Band edges are dithered with Bayer 4x4: bump to the next band when (frac(t) − 0.7)·3.4·16 > bayer.
- **Sun or moon**: drawn only for sunset and night. Day has no disc.
  - It sits opposite the shadow direction. Project that direction into view (du, dv), and draw only when dv > 0.2.
  - screenX = Cx + f·du/dv
  - screenY = horizon − f·k/dv, where k = 0.12 for sunset and 0.3 for night
  - Radius: sunset 0.05·W, moon 0.022·W (about 5 px at W = 240).
  - Solid disc inside r. Out to 1.9r, a glow ring is Bayer-dithered with density (1.9 − d)·14 against the 0..15 threshold.
- **Stars** (night only, not in the lowest 2 bands):
  - Hash (x + starOffset, y), with starOffset = heading·f wrapped to 4096. Stars therefore scroll horizontally exactly with yaw, as they should for a rotating camera at infinity.
  - A star is drawn where the hash byte < 2, a density of about 0.8 %, in the moon colour #E8EEFF.
  - The reference has no twinkle. For originality, add twinkle: modulate 1 in 4 stars between two brightness levels with a slow per-star phase, and give some stars a dimmer #8894C0 tone.
- **Cost**: cheap. It only fills empty pixels above the horizon.

---

## 4. Clouds

- **Representation**: the same column renderer, but a slab floating at CLOUD_ALT = 40 with up to CK = 8 layers. In the world, a cell's cloud spans heights 40 … 40 + thickness − 1.
  - Top: drawn between rowNow and rowPrev of the top layer.
  - Side: walls from rowPrev of each layer k+1 to k, in the per-layer side colour (darker low, lighter high).
  - Underside: when the camera is below, the cloudUnder colour.
  - Rim: the thickness-1 rim is drawn as a checkerboard ((x + y) & 1), which gives a soft dithered edge.
- **Generation**: 200 clusters, each with a rank 0..15. A cluster shows if rank < cover·16; cover 0.45 shows rank < 7.
  - Each cluster has 4–10 round puffs strung along a line. Angle ≈ 0.35 ± 0.65 rad, length (12–36)·scale.
  - Puff radius (6–14)·scale·(1 − 0.45|t|), where t runs along the line.
  - Thickness per cell = sqrt(1 − d²)·maxT, a dome, with maxT = clamp(0.6r, 2, 8). Combine with max.
- **Drift**: (7, 3) u/s.
- **Fly-through**:
  - When the jet is inside the slab altitude, cells within radius ≈ 12 u are cut to 0. Cells within a further 3 u, with a hashed ragged edge, are thinned by 2.
  - Cuts go into a ring buffer and heal (restored from a base copy) after 11 s. This leaves a tunnel behind the jet.
  - Entering a cloud spawns a ring of 14 white vapour puffs. While inside, vapour streams off the wingtips at about 60 puffs per second.
  - In the cockpit view, a white mist is blended over the frame.
  - For a tiny LCD, a simpler fly-through is: while inside, blend the frame 50–80 % toward white with a flicker, and keep a cheap cut ring buffer, or skip cuts entirely.
- **Cost**: one extra map lookup per sample (the cloud map is cheap). The cut ring buffer is 64k entries in the reference, so shrink it to a few hundred.

---

## 5. The jet sprite stack

### Model structure (described, not reproduced; design your own)
- **Grid and pivot**: the grid is 34 (span) × 46 (length) × 13 slices. Slice 0 is the belly and is empty; slice 12 is the fin tips. About 2.5k filled voxels in total. The pivot is about 55 % back from the nose (row 25 of 46), at slice 5 (wing level).
- **Fuselage**: a pointed nose starting at row ~1 in slices 3–4. It widens to about 10–12 voxels by mid-body, and is about 8 voxels thick from slice 1 to slice 7.
- **Main wing**: in slices 3–4 (thin, 2 voxels thick). It is a swept, trapezoidal delta. The root runs from about row 18 to row 40, and it reaches full 34 span around rows 30–34. The wingtips are at ±15.5, row 33, slice 4.
- **Canopy**: slices 6–9, rows ~11–24, a teardrop bubble 4–6 wide, using the canopy palette family.
- **Tail**: twin canted fins at the rear (rows ~38–45, slices 7–12), about 16–18 apart, accent-coloured at the top. There are also small tailplanes.
- **Engines**: two side by side, about ±2 off centre, centre height 4.1, radius 1.6, with the nozzle at row 43.6. The engine-glow family colours sit there.
- **Palette families**, each a tone ramp:
  - body: 9 grey-blue steps, near-black to near-white (key tone #545969)
  - accent: 6 orange steps (key #F0580E)
  - canopy: 6 teal steps (key #1E8CA0)
  - engine glow: 5 steps, dark ember to pale yellow
- **Recolouring**: user recolouring scales each tone by luminance relative to the family's key tone, toward black below the key and toward white above it.
- **For the Cardputer**: design an original model of maybe 24–28 wide × 32–36 long × 9–11 slices (≈ 1–1.5k voxels) and store only shell voxels.

### How it is drawn: true 3D voxel splatting, ordered by slices
- **Jet axes** from yaw ψ, total bank r (bank plus roll-stunt spin) and pitch θ. First roll about the forward axis, then pitch:
  - right = (cosψ·cos r, sinψ·cos r, −sin r)
  - up₀ = (cosψ·sin r, sinψ·sin r, cos r)
  - forward = (sinψ, −cosψ, 0)·cosθ + up₀·sinθ
  - up = up₀·cosθ − (sinψ, −cosψ, 0)·sinθ
- **Per-voxel transform**: transform the three axes into view space once per frame (across, depth, height). Each voxel world offset is a·right + b·forward + (slice − pivotSlice)·up, where a is across from centre and b is pivotRow − row. Then project it with the camera formula. Precompute the voxel lists per slice as (a, b, colourIndex).
- **Splat size**: each voxel is drawn as a block of sub-sample offsets so there are no pinholes when rotated.
  - ns = clamp(ceil(2·pxPerVoxel − 0.24), 2, 8) samples along the right and forward axes.
  - nu = clamp(ceil(1.15·|screen length of the up axis|), 1, 8) samples along up, which fills gaps between slices when the jet is seen edge-on in steep climbs or rolls.
  - Precompute the offsets once per jet per frame, scaled by m/m0 per voxel.
  - At 1.34 px per voxel that is about 3×3×1–2 = 9–18 writes per voxel. That is the main cost: 2.5k voxels × ~12 ≈ 30k writes.
  - Cheaper option: use only shell voxels and a fixed 2×2 block, or project each voxel once and write a ceil(m)×ceil(m) rectangle.
- **Order**: sort slices by |(slice − pivot) − eyeHeightInJetFrame| and draw farthest first (painter's). Within a slice, order does not matter because it is nearly planar.
  - Seen from behind and above, this means belly first, fins last.
  - Afterburner slices drawn from behind go after all body slices, in front of the tail.
- **Occlusion**: skip a pixel when worldDepth[px] < voxelDepth − 1. This hides the jet behind hills and clouds.
- **Lighting**: two palettes per frame, one for each half of the jet (a < 0 and a > 0).
  - Half normal n = up ± 0.4·right. L = (−sunX, −sunY, 1.1).
  - With b = n·L normalised: b > 0.9 (night 0.74) uses the lit palette (one tone up). b < 0.66 (night 0.4) uses the dim palette (one tone down). Otherwise the normal palette.
  - When banked, the raised wing brightens and the lowered one darkens. This is a strong, cheap 3D cue.
  - The reference's pilot model uses per-voxel normals quantised to 27 directions to pick 3 tones. That is optional.
- **Outline**: mark every jet pixel in a mask. Afterwards, scan the jet's bounding box ±4 px. Every empty pixel 4-adjacent to a body pixel, and not behind the world, becomes the outline colour.
  - Outline colour: day #0D0F1A (graded); night a lighter #2C4076 so it reads against a dark sea.
  - Flame pixels are excluded from the outline.
- **Intro flourish** (a technique, optional): the slices fall in from above one after another (0.07 s stagger, 0.36 s quadratic fall each), then the afterburner fades in.

---

## 6. Flame, trails, smoke, wake, spray

### Afterburner, pixel style (the one to use on the device)
- **Length** in voxels: (0.07 + 0.17·afterburner)·46·burn·(1 + 1.6·boost), with afterburner = 0.8. That is ≈ 9.5 at cruise and ≈ 25 at full boost.
- **Flicker**: ×(0.82 + 0.18·sin(47t)·sin(31t)).
- **Geometry**: per engine (±engine offset), step t along the plume from the nozzle backward in 0.5 steps.
  - Radius = R·1.15·(1 + 0.22·boost)·(1 − t/len)^0.75·sliceProfile·(0.9 + 0.2·sin(1.9t − 40·time ± 1)).
  - sliceProfile = sqrt(1 − dy²) for slices near the engine height, so the plume is round in the stack.
  - Curl offset = (t²/len)·(bend·sideways + 0.35·climbTilt·up), with bend = 0.36·clamp(yawRate/maxRate). The plume trails round the inside of turns.
- **Colour** by heat = 0.9·u + 0.55·|across|/rad + 0.3·|dy| − 0.16·boost, through a 6-step ramp: #FFFCE0 (< 0.28), #FFE078 (< 0.5), #FFA42C (< 0.75), #F66014 (< 1.0), #B0301A (< 1.25), #602228. White-hot in the core, cooler at the edges and tail.
- **Splat**: each sample is a ceil(0.55·m)-sized square, z-tested.
- **Not worth porting**: the reference's GPU "smooth flame" (a shader with fbm noise, Bezier plume, shock diamonds and a blue root) is too costly. A pixel version with 1–2 shock-diamond bright dots at fixed spacing along the core under boost would be a cheap nod to it.

### Wingtip trails (vapour)
- Each wingtip's world position is sampled every 2.5 u flown into a 160-entry ring of (x, y, z, time).
- Life = 2.6 s·(0.3 + 0.7·g), where g eases toward clamp(|yawRate|/maxRate + 0.7·(rolling) + 0.9·boost + |climbTilt|, 0, 1) at 2.5/s. Trails show up when you pull g's and fade in level flight. This is a great feel cue.
- Draw them as connected projected line segments, 1 px (2 px when near). Fade with a Bayer threshold: alpha = 1 for age < 0.5, then 2·(1 − age).
- Colour by age: white, then #D6E8FA, then #AACCF0 (graded by time of day). Z-tested.
- **Cost**: cheap. Cut to ~60 points on the device.

### Exhaust smoke
- Dark puffs spawned at 78 % of the flame length, alternating engines, every (0.034 − 0.012·boost) s.
- Size 1.3–2.5, growing ×(1 + 2.4u). Initial push = −0.3·velocity with an exponential stop.
- Alpha = (0.46 + 0.36·boost)·(1 − u)², with a 0.08 fade-in, ×0.55 at night. Life 0.75–1.25 s. Grey 26 → 96.
- Alpha-blended discs. This is moderate cost; use a pool of 32–64 on the device and dither instead of blending.

### Wake and spray (low over water)
- **Strength** = clamp((24 − heightAboveWater)/17, 0, 1)·clamp(speed/50, 0, 1). It starts at 24 u and is full at 7 u (the clearance).
- **Foam marks**: one every 2.4 u flown, at 6 u behind the jet, storing the across-direction. Each mark is drawn as three ellipses lying on the sea:
  - The two side ellipses are offset ±(3 + 27·u^0.7) across, so the V opens out. Radius 1.7 + 3.4u.
  - A centre ellipse (radius 2.6 − 2u) lasts only until u = 0.45.
  - Squash vertically by sin(view slope), min 0.22.
  - Bayer-erode the edges more with age. Blend toward the foam colour with strength 235·k. Life 2.8 s.
- **Spray**: 40 + 70·s white puffs per second, thrown sideways 20–110 u/s and back, life 0.5–1.1 s.
- **Cost**: moderate. Cap at ~40 marks and ~32 spray particles.

---

## 7. Flight model

All easing is first-order: x += (target − x)·(1 − e^(−k·dt)). Clamp dt to 0.05.

- **Speed**: 92 u/s × (1 + (1.8 − 1)·boost) × energy.
  - energy eases at 1.1/s toward 1 − 0.2·sin(climbAngle)·(1 − boost if climbing). Climbs bleed speed, dives add it, and boost cancels the climb penalty.
  - A hand-flown full flip bleeds an extra 0.42 (eased at 2.6/s).
- **Boost**: the input is a hold longer than 0.2 s, or Shift. A click shorter than 0.2 s is a roll.
  - boost eases toward 1 at 5/s and back toward 0 at 2.4/s.
  - Side effects: camera distance −16 %, jet screen row −3 % H, longer flames, denser smoke, longer trails (g), and loop turn rate +15 %.
- **Turning**:
  - want = stickX·maxRate, with maxRate = turn rate = 1.25 rad/s.
  - yawRate eases to want at 2.6/s, and yaw += yawRate·dt.
  - Bank target = clamp(yawRate/maxRate, −1, 1)·52°, eased at 5/s.
  - Bank is purely visual: turn rate is commanded directly, not derived from bank.
  - Pitch also gets +0.1·|yawRate/maxRate|, so the nose comes up slightly in turns.
- **Pointer / tilt mapping** (pointer mode is the analogue of IMU tilt):
  - Turn: t = clamp(offsetX/(0.26·W), −1, 1), deadzone 0.06.
  - Climb: the vertical offset from the "hold" row, deadzone 0.08, rescaled. Full back gives STEEP = 1.4 rad (80°) of climb. Full forward gives DIVE = 0.62 rad (35°). Apply expo: ask = sign·|a|^1.25·max.
  - Inverted-Y (stick-like) is the default.
- **IMU suggestion**: device roll → t (deadzone ≈ 3°, full at ≈ 25–30°). Device pitch relative to a calibrated neutral → climb (deadzone ≈ 4°, full at ≈ 25°, same expo). Low-pass the IMU at about 8–10 Hz. Recalibrate neutral on a key.
- **Climb dynamics**:
  - The commanded angle eases at 8/s toward ask, but its rate is limited to +1.745 rad/s (2π/3.6) going up and 0.85× that going down.
  - dz = speed·sin(angle)·dt. When within 14 u of the floor (going down) or the ceiling (going up), scale dz by distance/14. This is a soft cushion.
  - Horizontal speed = speed·sqrt(1 − (dz/(speed·dt))²).
  - With no input, the jet eases back to cruise altitude: ask = clamp(0.012·(altitude − z), −0.3, 0.35).
  - Idle bob: z += 1.35·cos(0.9t)·dt.
- **Altitude limits**:
  - Defaults: cruise altitude 64 in the code (the control panel default is 110), ceiling 300 in the code (the panel uses 1500), clearance 7.
  - Floor = the max ground height over 5 points along the path (0 … 0.64 s ahead) × 3 lateral offsets (±7 u), plus clearance. It is smoothed faster upward (9/s) than downward (3/s).
  - Below the floor, the jet is lifted (rate-limited to speed·dt). The hard floor is ground + 0.7·clearance, eased at 22/s. So you can skim terrain safely.
- **Barrel roll**:
  - Trigger: a click or tap (or a dedicated key). Direction = sign of the current yawRate.
  - Duration 0.85 s. The extra roll angle = dir·2π·easeInOutQuad(t), added on top of the bank.
  - The camera does not roll. Wingmen copy it with a 0.16 s stagger.
  - The roll pumps g, so the trails flare. The idle autopilot does a roll every 9–16 s.
  - (In the reference, a roll timed as a missile arrives dodges it and triggers slow motion; that is optional for us.)
- **Loops and flips**: hold full back (or full forward) with boost, and the command passes STEEP and keeps rotating at the turn rate, all the way round. Keys F and G fly an automatic 3.6 s loop with angle 2πt − sin 2πt (slow in, fast over the top). In a loop, motion follows the nose: dz = speed·sin(angle), horizontal = cos(angle).
- **Crash rule**: you cannot crash by accident. Only these cause a crash:
  - Committed dive: full forward against the floor, boost ≤ 0.3, held for 0.4 s. Then the jet dives at −0.9 rad and "hits" when the gap is < 3.4 or < max(8, sink rate)·0.4.
  - A loop or flip that meets the ground: descending, with the gap < max(3.4, 0.1·sink rate).
  - A crash triggers an ejection, explosion and respawn sequence. On a tiny device: a flash, shake, debris and a respawn above water.
  - A "pull up" warning shows while committing, or while a loop is less than 1.1 s from impact.

---

## 8. Post effects

The reference (post.ts) works on the full low-res RGBA buffer:
- **Flash**: blend toward warm white by up to 150/256.
- **Bloom**: a "hot colour" lookup on a 15-bit colour key, accumulated at half resolution, a 2× separable box blur (radius ≈ hw/70, min 2), then added with gain 3.4·bloom/(4·255). Default 0.5.
- **Vivid**: saturation × (1 + 0.38v), a contrast curve × (1 + 0.22v) + 4v. Default 0.25.
- **Vignette**: a smoothstep corner mask, darkening up to (setting·150)/256 at the corners. Default 0.3.
- **Grain**: LCG noise ±(28·grain)/2. Default 0.

What is worth it on a 240×135 LCD:
- **Vivid**: free. Bake it into all palettes at build or setup time.
- **Vignette**: cheap if precomputed. Store a per-row and per-column factor, or a 4-level mask applied only in the 16–24 px border. Worth it, subtle (about 0.2).
- **Bloom**: too expensive as a full pass. Fake it by drawing dithered glow halos directly around the flame tip, the nozzles, the moon and the sun (the reference already does this for the sun).
- **Grain**: skip. It looks like noise on an LCD.
- **Flash and shake**: cheap. Shake = offset Cx and Cy by sin(91t) and cos(77t)·3.2·shake. Keep them.
- **Ordered dither (Bayer 4x4)**: use it everywhere as the standard alpha substitute.
- **Colour depth**: the target is RGB565, so pre-build palettes as 565. Every gradient in the reference is already banded (8 sky bands, 10 fog steps, 3 jet tones), so 565 loses nothing.

---

## 9. Palettes (hex)

### Time-of-day grading
Grading is a function applied to the day colours (amount 1 for the world):
- night: r' = 0.22r + 2, g' = 0.30g + 6, b' = 0.50b + 22
- sunset: r' = 0.95r + 0.10g + 12, g' = 0.72g + 0.05r + 4, b' = 0.70b + 0.08r + 10

The jet is graded less: amount 0.5 at sunset and 0.16 at night. Clouds are graded at 0.72 at night.

### Sky, haze and lights
| | Day | Sunset | Night |
|---|---|---|---|
| Sky horizon A | #C6E8FA | #FFC684 | #24386E |
| Sky mid M | #7ABEF0 | #EE7A82 | #121C46 |
| Sky top B | #3E8EDE | #544496 | #070A22 |
| Haze / far sea | #BCE0F6 | #FAB686 | #1C2C5C |
| Sun or moon disc | (none drawn) | #FFF4C8 | #E8EEFF (also the star colour) |
| Disc glow | – | #FFDE96 | #4660A0 |
| Glint A / B | #78BEEC / #ECFAFF | #FFAA6E / #FFE2AA | #466EBE / #AAC8F0 |
| Foam | #F0FCFF | #FFE2D6 | #6E8CC8 |
| Jet outline | #0D0F1A | (graded) | #2C4076 |
| Cockpit light tint | #FFF6DE | #FFAA5C | #96BEFF |

### Materials (day top / side)
Apply the grading above for sunset and night.
- Sea: deep #2468B4, mid #2C7CC6, shallow #42A0D8, surf #68CCD6. Sides are the same as tops.
- Sand #EEDCA4 (speck #DEC88C), side #CCB47A.
- Grass #60B234, dark #46942C, light #86CE4C. Sides #38782C, #326C2A, #3C802E.
- Trees: top #469E40, mid #2A7A36, edge #1C5C2E. Sides #184E2A, #164828, #144026.
- Rock #928C86, dark #76706E, light #B2ACA2. Sides #605A5C, #565256, #686262.
- Snow #F6FAFF, shade #D6E2F4. Sides #BACAE4, #B0C0DC.
- Shadow = the colour × (0.62, 0.67, 0.80).

Night examples, precomputed:
- Sea: deep #0A2570, mid #0C2B79, shallow #103682, surf #194381.
- Sand #364868, grass #173B30, rock #223059, snow #385195.

### Clouds (day)
- Top: #DAE6F7 for the thinnest layer, then #FFFFFF from thickness 2 up.
- Sides: from #9EB6DE (lowest layer) to #CADAF0 (highest).
- Underside: #8AA0C8.
- Night: top ≈ #7183B3, side ≈ #475F9E.

### Effects
- Flame ramp: #FFFCE0, #FFE078, #FFA42C, #F66014, #B0301A, #602228.
- Trails: #FFFFFF, #D6E8FA, #AACCF0, graded ×0.8 by time of day.
- Smoke: grey (26 → 96, slightly blue). Spray: #FFFFFF / #CEE8FA.
- Explosion: #FFFAE0, #FFD660, #FA8C28, #CE4218, #322E32.
- Jet defaults: body #545969, accent #F0580E, canopy #1E8CA0.

---

## 10. What is expensive, and the embedded adaptation (ESP32-S3FN8, no PSRAM)

- **Memory is the real constraint.** There is about 320 KB of usable SRAM, and the 240×135 RGB565 framebuffer alone is 64.8 KB.
  - The 2048² terrain (4 MB) and 1024² cloud map (1 MB) are impossible. Use instead:
    - (a) a 256² wrapping map with one packed byte per cell (4-bit height level plus 4-bit material: 64 KB), sampled with a world-units-per-cell scale of 2–4, or a 128² map with bilinear "level" sampling; or
    - (b) store it as a const array in flash (8 MB), generated by a PC tool at build time.
  - Clouds: a 128² nibble map (8 KB) at 4–8 u per cell, or procedural puffs from a list of about 40 circles.
  - Depth buffer: 8 or 16-bit (32–65 KB). Or only test occlusion for jet, trail and smoke pixels by storing an 8-bit quantised depth (32 KB).
- **CPU budget at 60 fps is 16.7 ms.**
  - World column render ≈ 5–9 ms (fixed-point, rows precomputed per depth step, inner loop is two lookups plus a short fill).
  - Jet splat ≈ 2–4 ms with about 1.2k shell voxels and 2×2 blocks.
  - SPI DMA push of 64.8 KB at 80 MHz ≈ 6.5 ms. Double-buffer or split into halves so the push overlaps the render: render into one strip while the other is DMA'd, or use two 32 KB half-buffers.
  - Target 50–60 fps. The second core can run audio, IMU and keyboard.
- **Drop or cheapen**: smooth GPU flames, bloom, grain, the 64k cloud-cut buffer, blended smoke (use dithered smoke instead), per-voxel normals, and the cockpit roll-sheet. Keep the chase view only, with no rotated sheet.
- **Keep, because they carry the feel**: the lens-shift level camera, terraced islands with lit and shaded variants, 10-step fog, Bayer-dithered sky bands, night stars with heading-locked scroll (plus our own twinkle), the moon with a dithered halo, sea glints and surf flicker, jet shadow on the sea, per-wing-half lighting plus outline, eased yaw, bank and boost, the camera heading lag, g-dependent wingtip trails, the pixel afterburner, the low-altitude wake V, and the barrel roll.
- **Audio hooks the reference exposes**, worth mirroring on the ES8311: engine level (boost), speed ratio, spray (wake strength), dive alarm, cloud whoosh, and one-shots for roll, swish and crash.

## 11. IP note
Do not reuse:
- the RLE base64 jet data, its exact palette ramps or its pivot and engine constants
- the pilot model and the code structure or text

The tuning constants above are design parameters and fine to borrow as starting values. The jet model must be our own: a different silhouette is suggested, for example single fin vs twin, different wing planform and canopy placement, in our own colours.
