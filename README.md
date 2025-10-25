
# Pico DMA/PIO Stepper Controller (Real-Time Position + Look-Ahead Blending)

Arduino sketch for **Raspberry Pi Pico 2 W (RP2350)** that drives **X/Y/Z steppers** with **PIO + DMA**, a trapezoid/**S-curve** profile, **look-ahead blending** (smooth cornering), and **true real-time position tracking** derived from the DMA transfer counter.

It’s designed as a **building block** for CNC/robotics projects: copy, trim, or extend.

---

## Features

* **3 axes** (X/Y/Z) with **DIR** on GPIO **0–2** and **STEP** on GPIO **3–5**
* **PIO pulse engine** with a fixed high time (**20 µs**), plus per-event extra delay in µs
* **Single-channel DMA** streaming of 32-bit motion “words” into the PIO TX FIFO
* **Real-time position** tracking from DMA progress (no per-step ISRs)
* **Trapezoid / S-curve** timing (quintic smoothstep) along a **master axis** + Bresenham DDA for followers
* **Look-ahead blending** across segments (non-zero junction speed)
* Tiny **G-code subset** over Serial: `G0`, `G1`, `G28`, `G92`, `M114`, `M0`, `M2`, `M110`, `M111`, `M120`, `M121`
* **RP2350-safe DMA abort** (clears channel EN bit before `abort()` to avoid retriggers)
* **Queue** of moves with clean start/stop semantics

> Works on **RP2040** too, but RP2350 errata handling is included here by default.

---

## Hardware & Pinout

* Board: **Raspberry Pi Pico 2 W (RP2350)**
* Logic level: **3.3 V** (A4988/DRV/TMC drivers typically accept 3.3 V logic)
* Pins:

  * **DIR**: X = GPIO **0**, Y = GPIO **1**, Z = GPIO **2**
  * **STEP**: X = GPIO **3**, Y = GPIO **4**, Z = GPIO **5**
  * Optional **Enable**: configure per your driver (not used by default)
  * Optional **Homing sensors**: `HOME_X_PIN = 20`, `HOME_Y_PIN = 21` (pull-ups enabled, active-low)

---

## How it Works (quick)

Each “motion word” (32-bit) encodes one step event and its timing:

```
[0..2]   DIR mask  (X/Y/Z)
[3..5]   STEP mask (X/Y/Z)
[6..31]  extra delay in µs after the fixed 20 µs high
```

* The **PIO** program sets STEP high for **20 µs**, then waits the **extra delay**.
* **DMA** streams these words straight into the PIO TX FIFO.
* **Real-time X/Y/Z** is computed by parsing only the words that have actually **left** DMA → PIO, using the channel’s `transfer_count`. No guessing, no step ISRs.

**Look-ahead blending** adjusts the **end speed** of segment *A* and the **start speed** of segment *B* to a common **corner speed** (based on corner angle, accel limit, and “blend length”). This removes micro-stops at junctions.

---

## Repository Layout

```
/ (repo root)
├─ Pico2W_Simple_Stepper3.ino     # main sketch (your file name may differ)
├─ stepper_1sm.pio                # PIO program (auto-compiled to header)
└─ README.md
```

---

## Build

1. Install the **Earle Philhower “Arduino-Pico”** core.
2. In Arduino IDE:

   * Board: **Raspberry Pi Pico 2 W**
   * Upload over USB as usual.
3. Open Serial Monitor at **115200** baud.

---

## Usage (G-code)

Examples:

```
G1 X2000 Y1000 Z0 F2500
M114
G0 X0 Y0 Z0
G92 X0 Y0 Z0
```

* Positions are **absolute steps** (not mm).
* `F` is **steps/second** along the master axis.
* `G0` uses “rapid” delays defined in the sketch.
* `G92` sets the current absolute position (the sketch does a soft stop first).
* Homing demo:

  ```
  M120     ; enable homing sensors (GPIO 20 & 21, active-low)
  G28      ; move to origin (0,0,0)
  M121     ; disable homing sensors
  ```
* Stops:

  * `M2` — **force stop** (queue preserved)
  * `M0` — **force stop + clear queue**

---

## Tweak Guide (what to change & where)

Below are the main knobs exposed in the sketch and how they affect motion.

### Motion & Blending

* `d_start_us`, `d_cruise_us`, `d_end_us` (per move)
  Inter-step **periods** in microseconds. `d_cruise_us = 1e6 / F` where **F** is steps/s.
  Start/end are the entry/exit periods; with blending enabled, these become the **corner period** at the junction.

* `accel_frac`, `decel_frac` (per move, 0..1)
  Fractions of the master-axis step count used for accel and decel segments of the trapezoid/S-curve.

* **Look-ahead blending knobs** (globals):

  ```cpp
  static const float A_MAX_STEPS_S2   = 15000.0f; // max acceleration in steps/s^2
  static const float BLEND_LEN_STEPS  = 50.0f;    // blend length around corners (steps)
  static const float MIN_CORNER_SPEED = 0.0f;     // floor for corner speed (steps/s)
  static const float MAX_FEED_STEPS_S = 12000.0f; // global speed cap (steps/s)
  ```

  * Increase `A_MAX_STEPS_S2` to allow higher corner speeds (if your machine can handle it).
  * Increase `BLEND_LEN_STEPS` to make corners smoother/longer; decrease for tighter path tracking.
  * Set `MIN_CORNER_SPEED` to a small non-zero value (e.g. 100–300) to avoid “sticky” slowdowns on tiny angles.
  * Use `MAX_FEED_STEPS_S` as a global limiter to keep everything within driver limits.

* **S-curve shaping**
  Timing per phase is derived from a quintic smoothstep (6t<sup>5</sup> &minus; 15t<sup>4</sup> + 10t<sup>3</sup>). If you want sharper ramps, replace the easing function.

### Pins & Polarity

* **Pins** are fixed in the PIO setup:

  * DIR: 0,1,2 — STEP: 3,4,5
* **Invert DIR** if an axis moves the wrong way:

  ```cpp
  #define DIR_INV_MASK  (0u)  // e.g. (1u<<1) to invert Y
  ```
* **DIR setup time**
  Ensure your first step after a direction change has enough delay (≥5–10 µs). In this design, the **entry period** (`d_start_us`) naturally provides that gap at segment boundaries. If you ever toggle DIR mid-segment, add a no-STEP word with only DIR bits + extra delay.

### Timing / PIO

* **Pulse width** (20 µs) is encoded in the PIO program. To change it, edit the `.pio` and regenerate the header. Keep driver requirements (typically ≥ 2–5 µs) in mind.
* **Tick rate** is set to **1 MHz** (1 tick = 1 µs). You can change `clkdiv_1MHz()` if you want finer/coarser timing, but keep the word packing consistent.

### DMA / Buffers

* `BUFFER_SIZE` (default **512**)
  How many motion words per DMA burst. Smaller buffers → more frequent real-time updates; larger buffers → lower CPU overhead.
* `MOVE_QUEUE_SIZE` (default **32**)
  How many moves can be queued.
* **Stops**

  * `stop_motion_soft()` waits for DMA + PIO to finish naturally (no abort). Good for `G92` or “wait until idle”.
  * `stop_motion_force()` is an **RP2350-safe abort** (clears EN bit, aborts, resets PIO). Used by `M0/M2` and E-stop.

### Homing

* `HOME_X_PIN = 20`, `HOME_Y_PIN = 21` (active-low, pull-ups on).
  The ISR calls **emergency stop**, clears the queue, and prints the position. Adapt to your sensors and add homing sequences as needed.

### Units / Steps-per-mm

Positions and feeds are **in steps**. To work in physical units, wrap parsing to convert mm ↔ steps:

```cpp
constexpr float X_STEPS_PER_MM = 80.0f;
int32_t steps_from_mm(float mm) { return (int32_t)lrintf(mm * X_STEPS_PER_MM); }
```

Apply per axis when parsing `G0/G1`.

---

## Look-Ahead Blending (what you get)

v<sub>corner</sub> &le; min(
  v<sub>cruise,prev</sub>,
  v<sub>cruise,new</sub>,
  &radic;((A<sub>max</sub> &middot; L<sub>blend</sub>)/(2 &middot; sin(&theta;/2)))
)

Then it sets:
`prev.d_end_us = new.d_start_us = 1e6 / v_corner`
This smooths the velocity through the junction and removes micro-stops. You can tune corner behavior by adjusting the four blending knobs above.

---

## Troubleshooting

* **Nothing moves**

  * Check STEP pin (GPIO 3) with a scope/LED during a move.
  * Ensure driver **EN** is in the enabled state (many drivers are EN=LOW).
  * Confirm PIO state machine is enabled and mapped to pins 3..5.

* **Wrong direction on an axis**
  Set the appropriate bit in `DIR_INV_MASK`.

* **Stutters at segment boundaries**
  Don’t reset the PIO SM between moves. In `start_next_move()` the sketch only re-arms DMA; the SM stays enabled.

* **Weird behavior after an E-stop**
  RP2350 requires clearing the DMA channel’s **EN bit** *before* `dma_channel_abort()`. The sketch already does this in `stop_motion_force()`.

* **Missed steps at high feed**
  Increase `d_start_us`/`d_end_us` (for more conservative ramps) or reduce `MAX_FEED_STEPS_S`. Also verify your driver’s max step rate and minimum pulse width.

---

## Safety

This is firmware that can move real hardware. Use emergency stops and homing sensors. Validate limits, currents, and mechanics before running at speed.

---

## License

MIT (or your preferred license).

---

## Credits

* Built by **Sjoerd Leewis** with the Earle Philhower Arduino-Pico core.
* Thanks to the Raspberry Pi PIO/DMA docs and community examples.

---

### Want to extend?

PRs/issues welcome: arcs (G2/G3), multi-segment forward/backward look-ahead, per-axis jerk limits, steps-per-mm calibration, and path planners in physical units.
