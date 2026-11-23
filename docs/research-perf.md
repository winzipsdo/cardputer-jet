# Graphics performance report: ESP32-S3 software rendering for the Cardputer-ADV jet game

Labels used below: **[M]** means a measured or documented fact from a cited source. **[E]** means my own engineering estimate; check these with `esp_cpu_get_cycle_count()` microbenchmarks on the device.

## 0. Key facts about the hardware

- **[M] Cardputer and Cardputer-ADV display setup in M5GFX** (from the generated board data, identical for both models):
  - Bus: `SPI3_HOST`, `bus_freq_write = 40000000` (40 MHz), read 16 MHz, `bus_three_wire = true` (no MISO; DC is a separate pin).
  - Panel: ST7789V2, native 135x240 (portrait), `offset_x = 52`, `offset_y = 40`, `invert = 1`.
  - DMA: `SPI_DMA_CH_AUTO`.
  - Pins: SCLK 36, MOSI 35, DC 34, CS 37, RST 33.
  - None of these are the SPI2 IO_MUX pins (9 to 14), so the signals go through the GPIO matrix.
- **[M] ESP-IDF SPI master on S3:**
  - The GPIO matrix behaves the same as IO_MUX at 40 MHz or below. The docs give no S3 guarantee above 40 MHz.
  - DMA buffers must be in internal DMA-capable RAM and 32-bit aligned (start and length). Otherwise the driver copies them into temporary buffers.
  - `max_transfer_sz` defaults to 4092 bytes per transaction.
  - Per-transaction overhead: about 26 µs (interrupt + DMA), 11 µs (polling + DMA), and about 2 µs to build the DMA linked list.
- **[M] ESP-IDF speed guide:**
  - Float is always slower than integer, even with the single-precision FPU. `double` is emulated in software and is very slow.
  - Code normally runs from the flash cache. IRAM code always runs at full speed.
  - Use `-O2` for performance.
  - QIO flash is almost twice as fast as DIO.
- **[M] Float division:** the Xtensa FPU has no single-instruction divide. Division is `recip0.s`/`div0.s` plus Newton-Raphson steps, or a libgcc call. One author found that replacing division with multiplication by a reciprocal was their biggest speed gain (llandsmeer blog).
- **[E] No PSRAM (S3FN8):**
  - Every buffer sits in internal SRAM, which is uncached, low-latency and DMA-capable. Flash-cache and PSRAM bandwidth problems do not apply.
  - Expect about 300 KB of free heap under Arduino. **Memory, not CPU, is the binding constraint.**
- **[M] Reference demo:**
  - World is 2048² with wrapping, `VIEW_DIST = 1500`, `FOG_START = 300`, 10 fog steps.
  - The terrain is a front-to-back depth-slice voxel-space renderer with a per-column occlusion row and LOD stepping ("about one screen row of sea per step").
  - The chase camera never rolls. Only the jet sprite banks, and the horizon moves only with pitch. Only the cockpit view rotates a larger sheet.
  - **Implication:** classic voxel space (yaw, pitch, altitude) is enough for the chase view. A barrel roll is just a jet-sprite rotation.

## 1. CPU cost per technique (240 MHz, LX7, in-order, single issue)

| Technique | Cycles | 240x135 (32,400 px) | 120x68 (8,160 px) | Verdict |
|---|---|---|---|---|
| Voxel space (Comanche), int 16.16, y-buffer, LOD steps | about 20-30 cycles per sample [E] (2 loads, address mask, `mull`, compare and branch); span fill about 3-4 cycles per px [E] | 240 cols x about 200 steps = 48k samples, about 1.2M cycles, **about 5-5.5 ms** [E] | 120 x about 100 = 12k samples, **about 1.3-1.5 ms** [E] | **Use this** |
| Mode-7 ocean or cloud plane (per-row reciprocal, per-pixel fixed-point step) | about 5-8 cycles per px [E] | about 0.5 ms for the ground area [E] | about 0.15 ms [E] | Good for a cloud deck or water texture |
| Per-pixel heightmap raymarch (about 64 steps x about 10 cycles) | about 640 cycles per px [E] | **about 86 ms** [E] | about 22 ms [E] | Reject |
| Procedural noise per pixel or per sample | 50-200 or more cycles [E] | over 10 ms | marginal | Reject. Generate the world at boot (about 100-300 ms [E]) |

**[M] Outside data points**
- An ESP32-S3 voxel terrain in the style of the 1993 Mars demo renders in about 60 ms per frame (hackaday.io "explorer"). Its resolution is not stated and it is not optimised for speed, so it only bounds the problem loosely.
- An integer-only S3 rasterizer claims 650 triangles at 60 fps on 480x320 SPI.
  - A full 480x320 frame over SPI is about 31 ms even at 80 MHz.
  - So the claim implies partial updates; treat it as compute throughput only.

**Hot-loop rules**
- **Integers in inner loops.**
  - Use 16.16 map stepping: `x += dx; y += dy`, then `idx = ((y>>16)&255)<<8 | ((x>>16)&255)`.
  - Project with `sy = horizon + (((camH - h) * invz) >> 16)`.
  - Floats are fine per frame and per depth step.
- **Division.**
  - Compute `invz[step]` once per depth step, which is at most about 250 divides per frame (under 0.1 ms even at about 100 cycles each [E]).
  - Do the same once per row for mode-7.
  - A reciprocal table is only needed if you add per-pixel perspective.
  - Avoid `/` and `%` in loops. The S3 has a hardware integer divider, but it is multi-cycle [E].
- **Build flags.**
  - Mark the 3-5 hot functions `IRAM_ATTR` (terrain column or row loop, span fill, LUT expand, jet splat). This removes the risk of I-cache misses.
  - Arduino-ESP32 defaults to `-Os`. Use `build_unflags = -Os` and `build_flags = -O2`.
  - Set `board_build.f_cpu = 240000000L` and QIO 80 MHz flash.
- **Data placement.**
  - Keep the heightmap and colour map in DRAM, not as `const` data in flash. Flash-mapped data goes through an about 32 KB D-cache, and voxel sampling across a 64 KB+ map would miss often [E].
- **Wrapping world in limited RAM.**
  - A reference-scale 2048² map (4 MB) cannot fit.
  - Option A: a 256² 8-bit map at 8 world units per texel. View distance 1500 then becomes about 190 texels.
  - Option B: island "stamps". About 8 islands of 64² (32 KB) placed on a 32² cell table (1 KB) for a 2048² effective world, at about 5 extra cycles per sample [E]. Most cells are ocean (islands 0.33).
  - Add a 128² max-filtered mip level (16 KB) for far samples to stop shimmer, which shows more at low resolution.
- **Lighting and fog through the palette, at zero per-pixel cost.**
  - Bake moonlit slope shading into the colour map at world generation.
  - Write `index = base | (fogLevel << 4)`, for example 16 base colours x 16 fog levels, with the fog level chosen once per depth step.
- **Early exit.** Stop a column, or the whole depth loop, once every y-buffer entry has reached the top row or the fog end.

## 2. Display throughput

**Transfer times**
- **[M/E]** A full frame is 240x135x2 = 64,800 B = 518,400 bits.
- **40 MHz** (stock M5GFX): 12.96 ms of raw transfer, about 13.5 ms in practice. That caps the game at **about 74 fps** however fast it renders.
- **80 MHz:** 6.48 ms, about 6.9 ms in practice, so **about 145 fps** is possible.
- **[E]** The S3 GPSPI clock comes from an 80 MHz source, so the realistic choices are 80, 40, 26.7 or 20 MHz. A request for "60 MHz" ends up at 40.
- **[M]** 80 MHz is widely reported to work on ST7789 boards, including over long cables with TFT_eSPI. M5's own Cardputer config uses 40 MHz, presumably to be safe.
- **Suggestion:** make 80 MHz a build option (`SPI_80MHZ`) with 40 as the default fallback. The link is 3-wire with no MISO, so readback cannot verify it; check visually.
  - To set it: either define your own `lgfx::LGFX_Device` (Panel_ST7789 plus Bus_SPI with the values above, `freq_write` = 80 MHz), or reconfigure the M5GFX bus after `begin()`. Check the API in the installed version.

**Key point:** 2x upscaling does **not** reduce SPI time. The panel always receives 240x135x2 bytes, so 40 MHz SPI caps the frame rate at about 74 fps either way. Low resolution only saves CPU time and memory.

**Overlap is mandatory at 40 MHz.**
- Without overlap: 16.7 ms − 13.5 ms leaves about 3 ms per frame for everything else.
- With overlap: frame time = max(render, push).

**Recommended scheme: indexed frame buffers plus banded DMA with ping-pong buffers.**
- **Core 1** (Arduino `loop`) renders into the *back* 8-bit index buffer.
- **Core 0** runs a pinned display task. It takes the *front* buffer and does:
  - `startWrite()` and `setAddrWindow(0,0,240,135)`
  - For each band:
    - `waitDMA()`
    - LUT-expand into ping-pong buffer A or B
    - `pushPixelsDMA`/`writePixelsDMA` (name varies by version)
- Ping-pong buffers are allocated with `heap_caps_malloc(MALLOC_CAP_DMA)` and are 4-byte aligned.
- Use about 8 logical rows per band, which is 16 output rows x 480 B = 7.7 KB, about 9 transactions per frame. Overhead is under 0.3 ms [E].
- Pre-byte-swap the LUT (store big-endian 565, `lgfx::swap565_t`) so the library does no per-pixel swapping.
- Handshake with two binary semaphores (frame ready, buffer free). Waiting on a semaphore yields, which keeps the IDLE0 task watchdog satisfied.

**LUT expand cost [E]**
- 2x mode: keep a 256 x uint32 LUT with each entry `(c<<16)|c`, already doubled and swapped. That is about 3 cycles per logical pixel (load byte, load word, store word). Copy the 480 B line once for the vertical duplicate. Total about 0.15 ms per frame.
- Native mode: about 0.5 ms per frame.

**Palette swaps** (day, sunset, night), boost or hit flashes, and the fog colour are all just a different 1 KB LUT, which costs nothing.

**Split rendering across both cores** is not needed at 120x68. At native resolution, keep it as a fallback: have core 0 render half the terrain columns while the DMA runs.

**[E] Optional:** the panel's native scan order is portrait 135-wide, so with rotation 0 each panel row is one landscape column. A column-major buffer could be pushed without transposing. Internal SRAM is uncached, though, so row-major strided fills cost the same. Keep row-major.

**[E] Tearing:** there is no TE pin. At 60 fps or more, tearing is mild and acceptable.

## 3. Resolution choice

| | 120x68 logical (2x2 px) | 240x135 native |
|---|---|---|
| Look | Chunky pixel art (closest to the reference's pixel size 3). About 120 effective PPI on the 1.14" panel. Jet about 28-32 px wide (use width about 0.25, not 0.19, so it stays readable). | Fine detail. Jet about 46 px. Looks less "pixel art". |
| Render CPU (core 1) | **about 3.5 ms** [E] | **about 9-10 ms** [E] |
| Index buffers (x2) | 16 KB | 65 KB |
| fps at 40 MHz SPI | about 74 (display-bound) | about 60-74 (bound by both) |
| fps at 80 MHz SPI | 140+ possible, so lock at 60 | about 100 |

**Recommendation:**
- Make `LOGICAL_W`/`LOGICAL_H` compile-time constants. Ship 120x68: it has headroom for clouds, particles and audio, and matches the aesthetic.
- Render 68 rows and emit 135 output rows (drop the duplicate of the last row).
- Draw the HUD in logical pixels with a 3x5 font. Avoid mixing resolutions, which breaks the pixel-art consistency.

## 4. Jet rendering (sprite stack vs alternatives)

1. **Runtime slice stacking with inverse-mapped rotation per slice.** Cost is about slices x bounding-box px x about 12 cycles: 32 x 30x12 is about 0.6 ms at 120x68, about 2.3 ms at native [E]. Stacking along the length (cross-sections) gives roll for free but only covers about one axis well.
2. **Recommended: runtime voxel-shell splatting.**
   - Use your own original model, about 24x40x10 voxels, keeping only surface voxels (about 800-1500).
   - Per voxel: a fixed-point 3x3 rotation (yaw, pitch, roll), project, a z-test in a small jet-local z-buffer (about 48x48, under 5 KB), then plot 1 px (2x2 when near).
   - About 30-40 cycles per voxel, so **about 0.15-0.25 ms** [E].
   - Handles barrel rolls, loops, bank and relative yaw continuously.
   - Follow with a dark pixel-art outline pass over the bounding box (about 2k px x 5 cycles, negligible).
   - Keep 1 voxel at no more than 1 px at gameplay size to avoid holes.
3. **Pre-rendered atlas.**
   - Full set: 32 roll x 7 relative yaw x 5 pitch at 40x32x8bpp = 1120 frames, about **1.4 MB**. Flash only, and it needs a custom partition; reading via the cache is fine because the blits are small.
   - Boot-time RAM cache: 32 roll frames at neutral yaw and pitch, about 40 KB.
   - Not worth it, because option 2 already costs under 0.3 ms and is continuous. Use an atlas only if profiling disagrees.
4. **Thruster flame and boost:** a palette-cycled 6-10 px plume of additive-looking indices drawn after the jet, about 0.05 ms [E].

## 5. Recommended architecture and frame budget

**Tasks**
- **Core 1:** game loop. Fixed-timestep physics (120 Hz) with an input snapshot, then render, then hand the frame off.
- **Core 0:**
  - Display task (high priority).
  - Input task at 200 Hz: BMI270 accel and gyro, 12 B over I2C at 400 kHz is about 0.35 ms blocking [E], so keep it off core 1. TCA8418 keyboard events go through the interrupt or FIFO.
  - Audio mixer feeding the ES8311 over I2S DMA.

**Budget: 60 fps target (16.7 ms), 120x68, SPI at 40 MHz**

| Stage (core 1) | ms [E] |
|---|---|
| Sim, IMU smoothing, camera | 0.3 |
| Night sky: per-row gradient memset (shifted by pitch), about 200 precomputed stars projected by yaw and pitch, moon | 0.15 |
| Voxel terrain + ocean (h=0) with LOD, y-buffer, palette fog | 1.3-1.5 |
| Ocean moon-glint or sparkle (hash(x,y,t) in a band under the moon, plus palette-cycled wave index) | 0.2 |
| Clouds (depth-sorted dithered impostors, or a mode-7 plane) | 0.5 |
| Jet splat + outline + flame + shadow on water | 0.4 |
| Particles, missiles, HUD | 0.3 |
| **Total render** | **about 3.5 (21% of budget)** |

| Stage (core 0, in parallel) | ms |
|---|---|
| LUT + 2x expand (CPU) [E] | 0.15 |
| SPI DMA transfer, CPU idle [M-derived] | 13.0-13.5 (6.5-6.9 at 80 MHz) |
| I2C input, audio mixing [E] | about 1-2 per frame-time |

**Result [E]:**
- Display-bound at about 74 fps at 40 MHz. Lock to 60 with `esp_timer` pacing for even motion.
- At 80 MHz, 60 fps has about 10 ms of spare time per frame.
- Native 240x135 still reaches 60 if SPI runs at 80 MHz, or with split rendering at 40 MHz.

**Memory (120x68) [E]**

| Item | Size |
|---|---|
| Heightmap 64 KB + colour/material 64 KB + mips 32 KB, or island stamps about 40 KB total | 160 KB (or about 40 KB with stamps) |
| Index buffers | 16 KB |
| DMA bands (2 x 7.7 KB) | 15 KB |
| LUTs | 3 KB |
| Jet model + z-buffer | under 15 KB |
| Audio buffers | about 8 KB |
| **Total** | **about 220 KB of about 300 KB** |

At native resolution, use stamps or derive colour from height and slope (dropping the 64 KB colour map).

**Measure first on the device:**
- Cycles per voxel sample.
- `pushPixelsDMA` band timing at 40 and 80 MHz.
- Visual stability at 80 MHz.
- I2C IMU read time.

Log all four over USB-CDC once per second.

## Sources
- [M5GFX generated board data (Cardputer / CardputerADV specs)](https://raw.githubusercontent.com/m5stack/M5GFX/master/src/board_detect/m5/generated/esp32s3_board_data.hpp) and [esp32s3_construct.inl](https://raw.githubusercontent.com/m5stack/M5GFX/master/src/board_detect/m5/esp32s3_construct.inl)
- [ESP-IDF SPI Master (ESP32-S3)](https://docs.espressif.com/projects/esp-idf/en/stable/esp32s3/api-reference/peripherals/spi_master.html)
- [ESP-IDF Speed Optimization (ESP32-S3)](https://docs.espressif.com/projects/esp-idf/en/stable/esp32s3/api-guides/performance/speed.html)
- [llandsmeer: ESP32 FPU and fast division](https://blog.llandsmeer.com/tech/2021/04/08/esp32-s2-fpu.html)
- [Three IPS Displays with ST7789 (80 MHz reliability, TFT_eSPI)](https://osrtos.com/projects/three-ips-displays-with-st7789)
- [Hackaday.io "explorer": ESP32-S3 voxel terrain about 60 ms per frame](https://hackaday.io/project/190074-explorer)
- [Hackaday: 3D rasterizer for embedded devices (S3, 650 triangles at 60 fps claim)](https://hackaday.com/2026/09/16/a-3d-rasterizer-for-embedded-devices/)
- [s-macke VoxelSpace (algorithm reference)](https://s-macke.github.io/VoxelSpace)
- Reference demo `scene.ts` (local, untrusted; used only for constants and technique, nothing copied): `C:\Users\winzi\AppData\Local\Temp\claude\ac57a37b8289de43\cea93141-5671-47fb-8431-9aff4089c98a\scratchpad\ref\scene.ts`
