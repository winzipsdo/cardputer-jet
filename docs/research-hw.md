# Cardputer-Adv + PlatformIO: verified facts

**Status: a test build compiled.** I built a test project that calls every API below with `pio run`. The result was SUCCESS: RAM 8.6% (28,132 B static), Flash 17.5% of the 3.2 MB app partition. Project: `C:\Users\winzi\AppData\Local\Temp\claude\ac57a37b8289de43\cea93141-5671-47fb-8431-9aff4089c98a\scratchpad\buildtest\` (`platformio.ini`, `src\main.cpp`, `build.log`). Nothing was flashed or run on hardware, so runtime behaviour is still unverified.

I read the library source at the current HEADs: M5Unified 96576a1 (0.3.0), M5GFX 28d4794 (0.3.0), M5Cardputer bf64d31 (tag 1.2.0 has the same `src/`). Clones are in `...\scratchpad\libsrc\`.

## 1. platformio.ini (compiled OK)
```ini
[env:cardputer-adv]
platform = platformio/espressif32@8.0.0      ; official platform, ships Arduino core 3.3.12
board = m5stack-cardputer                      ; board json already has USB_MODE=1, CDC_ON_BOOT=1
framework = arduino
board_upload.flash_size = 8MB                  ; board json wrongly says 4MB; Adv has 8MB
board_upload.maximum_size = 8388608
board_build.partitions = default_8MB.csv       ; app partition 3,342,336 B
monitor_speed = 115200
upload_speed = 1500000                         ; ignored by USB-Serial/JTAG, harmless
build_flags = -DARDUINO_USB_MODE=1 -DARDUINO_USB_CDC_ON_BOOT=1 -O2
build_unflags = -Os
lib_deps =
  m5stack/M5GFX@0.3.0
  m5stack/M5Unified@0.3.0
  https://github.com/m5stack/M5Cardputer.git#1.2.0
```

**Installed locally:**
- `%USERPROFILE%\.platformio\platforms\espressif32` is version 8.0.0.
- Its `framework-arduinoespressif32` is 4.30312.261116, which is Arduino 3.3.12. It also has `-esp32s3-libs` 3.3.12, toolchain-xtensa 14.2.0+20260121 and esptool 5.4.0. All were installed by my test build.
- The official platform supports Arduino 3.x now, so the pioarduino fork is not needed.

**Library versions:**
- **M5Cardputer:** the PlatformIO registry only has 1.1.1 (2025-09-28). It supports Adv but its `KeysState` has no `up/down/left/right/esc/F-keys`. The GitHub tag 1.2.0 (2026-06-08) adds the Fn layer. Its `library.json` still says 1.1.1, so pio prints "M5Cardputer@1.1.1+sha.2d4fa66".
- **Arduino core:** the library does not require core 3.x. M5's own Adv docs page uses `espressif32@6.7.0` (core 2.x) with `board = esp32-s3-devkitc-1`. Their Arduino docs say "board manager >= 3.2.2, M5Cardputer >= 1.1.0, M5Unified >= 0.2.8, M5GFX >= 0.2.10".
- **IRremote warning (harmless):** tag 1.2.0 declares `"IRremote": "*"`, which is ambiguous, so pio picked z3t0/IRremote@5.0.0 and printed a warning. It is never compiled because nothing includes it.
- **Board json:** `m5stack-cardputer` uses variant `m5stack_cardputer` (USB_VID 0x303a, PID 0x1001) and defines `ARDUINO_M5STACK_CARDPUTER`.
- **Debug level flag:** adding `-DCORE_DEBUG_LEVEL` gives a "redefined" warning, which is benign.

## 2. Init and how Adv is detected
```cpp
#include <M5Cardputer.h>
auto cfg = M5.config();
cfg.serial_baudrate = 115200;                          // M5 calls Serial.begin
cfg.fallback_board  = m5::board_t::board_M5CardputerADV; // =24; used only if detection is inconclusive
cfg.internal_mic    = false;                           // optional; mic shares I2S pins with the speaker
M5Cardputer.begin(cfg, true);                          // = M5.begin(cfg) + Keyboard.begin()
M5Cardputer.Display.setRotation(1);                    // landscape 240x135
```
- **Detection** (M5GFX `src/board_detect/m5/esp32s3.inl`, `cardputer_family_detector_t`):
  1. It probes ST7789 RDDID over SPI.
  2. It checks pulls on GPIO 8/9.
  3. It looks for an I2C ACK from the TCA8418 at 0x34 on SDA=8 / SCL=9. An ACK selects `board_M5CardputerADV`.
- **Gotcha:** a key held or a weak line during boot can leave detection unanswered. It then falls back to `ctx.fallback`, or to the non-Adv Cardputer, and logs "Cardputer variant unanswered". Setting `cfg.fallback_board` covers this.
- **Keyboard reader:** `Keyboard_Class::begin()` uses `M5.getBoard()` to pick `TCA8418KeyboardReader` for ADV and `IOMatrixKeyboardReader` for the original Cardputer.
- **Board check:** `M5.getBoard()==m5::board_t::board_M5CardputerADV` (enum value 24, from `lgfx/boards.hpp`).

## 3. IMU (BMI270, Adv only)
- **Enabled by default.** `cfg.internal_imu = true`, and `M5.begin` calls `M5.Imu.begin(&In_I2C, board)`. It finds BMI270 at 0x69, or 0x68 as a fallback, on In_I2C (I2C_NUM_1, SDA 8 / SCL 9, 400 kHz).
- **You must poll it yourself.** `M5.update()` / `M5Cardputer.update()` do not call `Imu.update()`.
```cpp
if (M5.Imu.update()) {                 // returns sensor_mask_t; 0 if no new sample
  auto d = M5.Imu.getImuData();        // d.accel.x/y/z in g, d.gyro.x/y/z in deg/s, d.usec
}
// or M5.Imu.getAccel(&ax,&ay,&az); M5.Imu.getGyro(&gx,&gy,&gz);
// also: setAxisOrder(...), setCalibration(a,g,m), saveOffsetToNVS()/loadOffsetFromNVS()
```
- **Scaling:** accel ±8 g (`aRes = 8/32768`), gyro ±2000 dps (`gRes = 2000/32768`), from `imu/BMI270_Class.inl`.
- **Output data rate:** the driver does not write ACC_CONF / GYR_CONF, so the chip reset defaults apply. Per the Bosch datasheet that is accel 100 Hz and gyro 200 Hz. This is from memory of the datasheet, not checked against it.
- **Read cost:** each `update()` is one register read (INT_STATUS_1) plus a 20-byte burst at 400 kHz, roughly 0.6–0.8 ms (estimated).
- **Axes (official M5 diagram, `m5stack-doc.oss-cn-shenzhen.aliyuncs.com/1178/IMU-Cardputer-Adv.jpg`):**
  - X+ points right along the keyboard rows, toward Enter.
  - Y+ points away from the user, toward the screen edge.
  - Z+ points up out of the keys.
  - M5Unified applies no Cardputer-specific axis remap, so these raw axes are what you get.
- **Mapping to the landscape screen (rotation 1):** screen-right = +X and screen-up = +Y. Lying flat, az ≈ +1 g. These signs are inferred, so check them on the device by printing values over serial:
  - Right edge down gives ax < 0.
  - Far (screen) edge down gives ay < 0.
  - Suggested mapping: roll ≈ atan2(-ax, az), pitch ≈ atan2(-ay, az). Gyro follows the right-hand rule.

## 4. Keyboard (TCA8418 at 0x34, INT on GPIO11)
**API:**
- `M5Cardputer.update()` runs `M5.update()`, then `Keyboard.updateKeyList()`, then `updateKeysState()`.
- `Keyboard.isChange()` only compares the count of keys pressed.
- `Keyboard.isPressed()` returns how many keys are down.
- `Keyboard.isKeyPressed(char)`
- `Keyboard.keyList()` returns `std::vector<Point2D_t>` with x 0..13, y 0..3.
- `getKeyValue(pt).value_first`
- `Keyboard.keysState()` has `.fn .shift .ctrl .opt .alt .enter .space .tab .backspace .word`. Tag 1.2.0 adds `.up .down .left .right .esc .f1..f12`, but those are set only while Fn is held.

**Layout** (`Keyboard.h`), rows y=0..3:
- y=0: `` ` 1 2 3 4 5 6 7 8 9 0 - = BKSP ``
- y=1: `TAB q w e r t y u i o p [ ] \`
- y=2: `FN SHIFT a s d f g h j k l ; ' ENTER`
- y=3: `CTRL OPT ALT z x c v b n m , . / SPACE`
- Fn-layer arrows: `;`=UP, `.`=DOWN, `,`=LEFT, `/`=RIGHT.
- The G0 button (GPIO0) is `M5Cardputer.BtnA` / `M5.BtnA`, read by `M5.update()`.

**Gotchas:**
1. `isKeyPressed(c)` compares against the shifted character when Shift or Ctrl is held (`;` becomes `:`). For game input, scan `keyList()` and compare `getKeyValue(p).value_first` or the (x, y) position.
2. Arrows only appear in `keysState` with Fn held. Use the raw `; , . /` keys for steering.

**Polling cost** (`TCA8418.cpp`):
- The reader is interrupt-driven. The ISR on GPIO11 (CHANGE) only sets a flag.
- When the flag is clear, `update()` does no I2C at all, so polling every frame is free.
- When the flag is set, it handles one FIFO event per call with three I2C transactions (getEvent, write INT_STAT, read INT_STAT), roughly 0.3 ms at 400 kHz (estimated).
- Because it is one event per call, a chord or a quick press-and-release takes several frames. Calling `M5Cardputer.Keyboard.updateKeyList()` 3–4 extra times per frame drains the FIFO cheaply (the TCA8418 FIFO holds 10 events).
- `updateKeysState()` clears and rebuilds some vectors every frame, which is cheap.

**Shared bus:** the keyboard, BMI270 and ES8311 all share In_I2C. The lgfx I2C layer has a per-port mutex, but keep all I2C access on one task to be safe.

## 5. Display (ST7789V2)
From `board_detect/m5/generated/esp32s3_board_data.hpp` and `esp32s3_construct.inl`:
- **Bus:** SPI3_HOST, 3-wire, write clock 40 MHz, read 16 MHz, `SPI_DMA_CH_AUTO`.
- **Pins:** SCLK 36, MOSI 35, DC 34, CS 37, RST 33, backlight PWM GPIO38 (256 Hz, channel 7, offset 16).
- **Panel:** 135x240, offset (52, 40), rotation_offset 1, invert 1. **Correction (fixer pass):** with this offset, M5GFX 0.3.0 `Panel_LCD::setRotation` gives internal rotation (r + 1) & 3, so `setRotation(1)` is PORTRAIT 135x240 here and landscape 240x135 is `setRotation(0)` (the older M5 examples predate the rotation_offset). The firmware tries 0, then 1, and checks width/height.
- **No SCT on SPI3:** segmented transfer is SPI2-only on the S3, so a DMA push > 32,768 B busy-waits the CPU for all but the last 32 KB segment. The firmware pushes the frame as two halves of <= 32 KB (DESIGN §3).

**Fastest full-frame push:**
```cpp
uint16_t* fb[2]; // each 240*135*2 = 64,800 B
fb[i] = (uint16_t*)heap_caps_malloc(64800, MALLOC_CAP_DMA | MALLOC_CAP_INTERNAL);
M5.Display.startWrite();              // once, keep the transaction open
// per frame:
M5.Display.waitDMA();                 // previous transfer finished -> its buffer is free
M5.Display.pushImageDMA(0,0,240,135, fb[cur]);  // returns immediately; render into fb[cur^1]
cur ^= 1;
```
- **Why keep `startWrite()` open:** `Panel_LCD::end_transaction()` calls `_bus->wait()`. A push wrapped in its own start/endWrite therefore blocks until the DMA finishes.
- **Byte order:** with the default `setSwapBytes(false)`, a `uint16_t*` buffer is sent as `swap565_t`, meaning big-endian RGB565 bytes. That is the native panel order and goes straight to DMA with no conversion. Store pixels pre-swapped (`__builtin_bswap16(rgb565)`, ideally baked into the palette). `setSwapBytes(true)` forces a per-pixel conversion on the CPU. `M5Canvas::pushSprite` also uses DMA (`push_sprite` calls `pushImage(..., true)`).
- **DMA buffer memory:** buffers that are not DMA-capable silently fall back to CPU output. Allocate with `MALLOC_CAP_DMA` in internal RAM.
- **Transfer time (calculated, not measured):** 64,800 B × 8 / 40 MHz = 12.96 ms per frame, so at most about 77 fps on the bus. With double buffering, rendering overlaps the transfer.
- **Optional 80 MHz overclock** (6.5 ms per frame): this compiled, but I have not run it on hardware. 80 MHz is above the ST7789 spec of about 62.5 MHz, although ST7789 panels are often reported to work at 80 MHz. The ESP32-S3 SPI clock steps are 80, 40, 26.7 and 20 MHz.
```cpp
auto* bus = static_cast<lgfx::Bus_SPI*>(M5.Display.getPanel()->getBus());
auto c = bus->config(); c.freq_write = 80000000; bus->config(c); bus->init();
```

## 6. Speaker (ES8311 codec + NS4150B amp, 8 Ω 1 W)
- **I2S setup** (`M5Unified.inl`): BCK 41, WS 43, DOUT 42, I2S_NUM_1, magnification 16. The enable callback `_speaker_enabled_cb_cardputer_adv` writes ES8311 registers over I2C (0x18) to power up the DAC at 0 dB.
- **Starts lazily** on the first tone or play. Call `M5.Speaker.begin()` in `setup()` so the first sound doesn't hitch.
- **API** (`Speaker_Class.hpp`):
  - `tone(freq, ms=UINT32_MAX, channel=-1, stop_current=true)`
  - `playRaw(const int16_t*/int8_t*/uint8_t*, len, sample_rate=44100, stereo=false, repeat=1, channel=-1, stop_current=false)`
  - `playWav`, `isPlaying(ch)`, `stop(ch)`
  - `setVolume(0..255)` (default 64), `setChannelVolume(ch, v)`
  - 8 channels, output at 48 kHz. The playback task has priority 2 and is not pinned to a core.
  - For streamed buffers, alternate 2 buffers with `setBufferReleaseCallback()`, or use 3 buffers without it.
- **Mic conflict:** the mic uses the same BCK/WS pins (DIN 46). Use `Speaker.end()` before `Mic.begin()`, as the M5Cardputer mic example does.
- **Headphones:** M5 docs say plugging into the 3.5 mm jack disables the speaker amp.

## 7. Memory
- **No PSRAM.** ESP32-S3FN8 is 8 MB in-package flash with no PSRAM (Espressif ordering table; M5's page doesn't mention PSRAM).
- Board json `maximum_ram_size` is 327,680. Static RAM in the test build was 28 KB.
- Two 64,800 B DMA framebuffers (129.6 KB) should fit easily without WiFi/BT. I have not measured the free heap or the largest DMA block: print `heap_caps_get_largest_free_block(MALLOC_CAP_DMA)` at boot (the test sketch already prints it).

## 8. Battery and brightness
- **Battery:**
  - `M5.Power.getBatteryLevel()` returns 0..100 from ADC1 on GPIO10 × 2.0, using `level = (mv-3300)*100/(4150-3350)` clamped.
  - `getBatteryVoltage()` returns mV.
  - There is no charge-state sensing (plain ADC, `pmic_adc`).
  - Each call does an ADC read, so call it about once per second.
- **Brightness:** `M5.Display.setBrightness(0..255)` (PWM on GPIO38).
- **Battery capacity:** 1500 mAh (1750 mAh before 2026-07-13, per the M5 docs).

## 9. Upload and serial
- **Port:** the native USB-Serial/JTAG enumerates as VID 0x303A / PID 0x1001 (`m5stack-stamps3.json` hwids and the variant's `pins_arduino.h`), shown on Windows as a `COMx` "USB Serial Device". The Windows display name is unverified.
- **Auto-reset:** Espressif docs say the USB-Serial/JTAG controller "is able to put the ESP32-S3 into download mode automatically", so `pio run -t upload` needs no button.
- **When auto-reset fails:** if the app crashes early, reconfigures the USB pins (GPIO19/20) or deep-sleeps, the port disappears. Manual download mode per M5 docs:
  1. Set the power switch to OFF.
  2. Hold G0 and plug in USB-C.
  3. Release G0.
  4. After flashing, press reset or power-cycle.
- **Serial:** CDC_ON_BOOT=1 means `Serial` is USB CDC. Unverified for core 3.3.12: HWCDC writes might stall briefly when no host is attached, so rate-limit debug prints, for example to one FPS/heap line per second.

## Unverified (check on hardware)
- IMU signs in landscape: confirm with a serial dump while tilting.
- BMI270 ODR (100/200 Hz datasheet defaults) and the I2C timings: these are estimates.
- Whether the 80 MHz SPI overclock is stable.
- Free heap and largest DMA block at runtime.
- Real fps.
- HWCDC behaviour with no host attached.

## Sources
- https://github.com/m5stack/M5Cardputer (src/M5Cardputer.cpp, src/utility/Keyboard/Keyboard.{h,cpp}, KeyboardReader/TCA8418.cpp, releases/tag/1.2.0)
- https://github.com/m5stack/M5Unified (src/M5Unified.hpp/.inl, utility/IMU_Class.*, imu/BMI270_Class.inl, Speaker_Class.hpp/.inl, Power_Class.inl)
- https://github.com/m5stack/M5GFX (src/board_detect/m5/esp32s3.inl, esp32s3_construct.inl, generated/esp32s3_board_data.hpp, lgfx/v1/LGFXBase.hpp, LGFX_Sprite.hpp, platforms/esp32/Bus_SPI.*, panel/Panel_LCD.inl, lgfx/boards.hpp)
- https://docs.m5stack.com/en/core/Cardputer-Adv (specs, pin map, download mode, PIO example)
- https://docs.m5stack.com/en/arduino/m5cardputer/imu (Adv IMU example and axis diagram)
- https://docs.m5stack.com/en/arduino/m5cardputer/keyboard
- https://docs.espressif.com/projects/esp-idf/en/stable/esp32s3/api-guides/usb-serial-jtag-console.html
- https://www.espressif.com/en/products/socs/esp32-s3/documentation (S3FN8: 8 MB flash, PSRAM N/A)
- Local: `%USERPROFILE%\.platformio\platforms\espressif32\platform.json` and `boards\m5stack-cardputer.json`; `pio pkg show` output from the registry.
