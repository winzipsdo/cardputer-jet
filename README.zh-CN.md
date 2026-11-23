# MOONSKIM — Cardputer-Adv 像素战机

[English](README.md) | **简体中文**

一个跑在 **M5Stack Cardputer-Adv** 上的像素风战机飞行小游戏：夜空星海、月光海面、体素岛屿与云层，体素战机 "Kestrel"，**倾斜机身（BMI270 IMU）即可操控**，可翻滚、加速、穿环计分。设备上稳定 60 fps。

灵感来自一个 sprite stacking 网页 demo。

| 夜晚 | 穿环 | 压坡度 |
|---|---|---|
| ![](docs/shots/01_night_moon_glitter.png) | ![](docs/shots/02_ring_run.png) | ![](docs/shots/03_bank_trails.png) |
| **黄昏** | **白天** | **标题** |
| ![](docs/shots/04_sunset.png) | ![](docs/shots/05_day_islands.png) | ![](docs/shots/06_title.png) |

## 玩法

- **Ring Run**（Enter）：45 秒倒计时，穿过前方的环加时、加分，连续穿环叠连击（最高 ×8），贴海低飞、翻滚、加速穿环有额外奖励。最高分存进 NVS。
- **Free Flight**（2）：自由飞行，不计时。
- 撞岛或高速扎海会坠毁，迅速复活继续。

## 操控（设备）

横握 Cardputer，开局出现 **HOLD STILL** 时保持平稳——此时的姿势就是"中立位"。

| 操作 | 方式 |
|---|---|
| 转弯 | 左右倾斜机身（或 `,` `/`、`A` `D`） |
| 俯冲 / 爬升 | 上沿往外压 / 往回拉（或 `.` `;`、`S` `W`） |
| 加速 | `Space` / `Alt` |
| 桶滚 | `Z` / `X`（或 `L` / `'`） |
| 开始 / 确认 | `Enter`；`2` 自由飞行 |
| 暂停 | `P` / `` ` `` / G0 键 |
| 重新校准倾斜 | `C` |
| 倾斜开关 / 俯仰反转 | `T` / `I` |
| 昼夜切换 / 视角 | `N` / `V` |
| 帧率统计 / 帮助 | `F` / `H` |
| 静音 / 音量 / 亮度 | `M` / `-` `=` / `[` `]` |

## 构建与烧录

需要 [PlatformIO](https://platformio.org/)。

```powershell
pio run -e cardputer-adv -t upload           # 编译并烧录
pio device monitor -e cardputer-adv          # 串口日志（115200）
```

其他环境：`cardputer-adv-bench`（不锁帧 + 开机自动跑分）、`cardputer-adv-debug`（IMU 数据流）、`cardputer-adv-spi80`（80 MHz SPI 超频）。

串口单字符命令：`?` 帮助、`b` 20 秒自动驾驶跑分、`g` 截图（配合 `python tools/grab_shot.py COM3 out.png`）、`i` IMU 数据流、`r`/`p` 翻转横滚/俯仰方向、`h` 内存、`v` 重打启动日志。

## 在电脑上预览

核心代码（`src/core`）是与平台无关的 C++17，可以在 Windows 上直接跑（需要 Visual Studio 2022 Build Tools）：

```powershell
tools\play\build.bat                          # 可交互窗口版：鼠标模拟倾斜，键盘同设备
tools\play\out\moonskim_pc.exe

tools\sim\build.bat wx test                   # 脚本化模拟器：22 个场景、截图、确定性校验
```

## 技术要点

- 120×68 调色板索引帧 + 深度缓冲，2× 放大到 240×135；core 0 上的推屏任务用 DMA 异步推屏，与 core 1 的渲染完全重叠。
- Comanche 式前后向体素地形 march，海面、岛屿、云层一次画完；整数化世界生成（PC 与设备 CRC 一致）。
- 体素战机：914 个外壳体素，背面剔除、无除法透视、26 法线光照、描边、尾焰、影子。
- 陀螺仪融合的倾斜控制：自动校准、软死区、响应曲线、中立位漂移补偿。
- 16 kHz 软件合成音效。
- 设备实测（20 s 跑分）：fps_min 59.8 / avg 60.0，core-1 每帧约 8 ms。

## 大小

| | 大小 |
|---|---|
| 固件（`firmware.bin`） | 约 792 KB（占 3.2 MB 应用分区的 23.6%） |
| 游戏代码（渲染、物理、玩法、音效、平台层） | 约 181 KB |
| 游戏常量数据（调色板表、字体、键位表等） | 约 27 KB |
| 其余（ESP-IDF、Arduino、FreeRTOS、USB、M5 显示/IMU/音频驱动库） | 约 580 KB |
| 源码（`src/`） | 约 1.3 万行 C++ |
| 内存 | 静态约 196 KB，开机后剩余堆约 52 KB |
| PC 预览版（`moonskim_pc.exe`） | 约 475 KB |

没有任何图片或音频资源文件：
- 914 个体素的战机在开机时由代码生成（约 29 ms）。
- 岛屿、海面和云层在开机时生成（约 333 ms），全部用整数运算，每次生成的世界完全一样。
- 星空和银河由程序生成。
- 所有音效都以 16 kHz 实时合成。

详细设计见 [docs/DESIGN.md](docs/DESIGN.md)。
