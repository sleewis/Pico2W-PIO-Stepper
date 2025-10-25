
# Pico DMA/PIO Stepper Controller (with Real-Time Position Tracking)

Small, focused Arduino sketch for **Raspberry Pi Pico 2 W (RP2350)** that drives **three stepper axes** using **PIO + DMA**, with a trapezoid/S-curve motion profile and **true real-time position tracking** derived from the DMA transfer counter.

This project is meant as a **building block** for your own CNC/robotics work: copy, trim, or extend as needed.

## Highlights

* **3 axes** (X/Y/Z) with **DIR** (GPIO 0–2) and **STEP** (GPIO 3–5)
* **PIO pulse engine**: fixed STEP high time (20 µs), per-event extra delay in µs
* **DMA streaming** of 32-bit “motion words” into the PIO TX FIFO
* **Real-time position** from DMA transfer count (no guessing, no per-step ISR)
* **Trapezoid / smooth S-curve** timing (quintic smoothstep) along the **master axis** + Bresenham DDA for followers
* Tiny **G-code subset** over Serial: `G0`, `G1`, `G28`, `G92`, `M114`, `M0`, `M2`, `M110`, `M111`, `M120`, `M121`
* **RP2350-safe DMA abort** (handles errata): clears channel EN bit before `abort()` to avoid retriggers
* Queue of moves with simple start/stop semantics

> Also works on RP2040 with minor caveats (see *Notes*).

---

## Hardware & Pinout

* Board: **Raspberry Pi Pico 2 W (RP2350)**
* Logic level: **3.3 V** (most stepper drivers accept 3.3 V logic)
* Pins:

  * **DIR**: X=GPIO **0**, Y=GPIO **1**, Z=GPIO **2**
  * **STEP**: X=GPIO **3**, Y=GPIO **4**, Z=GPIO **5**
  * Optional **Enable** pin: wire to your driver if needed (set LOW to enable)
  * Optional **homing sensors**: `HOME_X_PIN = 20`, `HOME_Y_PIN = 21` (pull-ups enabled)

> Ensure your driver’s **DIR setup time** is ≥ 5–10 µs before the first STEP of a block. The sketch handles this at the command level.

---

## How it works (in one minute)

1. The planner builds per-step **32-bit motion words**:

   ```
   [bit 0..2]  DIR mask (X,Y,Z)
   [bit 3..5]  STEP mask (X,Y,Z)
   [bit 6..31] extra delay (µs) after a fixed 20 µs high
   ```
2. A **PIO program** drives the STEP pins high for 20 µs, then waits the “extra delay”, and repeats.
3. **DMA** streams the motion words to the PIO TX FIFO.
4. The sketch computes **real-time positions** by comparing `transfer_count` with the active buffer and parsing only the words that have actually left DMA → PIO. This gives **exact X/Y/Z** without interrupts per step.

---

## Build & Flash

1. Install the **Earle Philhower** “Arduino-Pico” core.
2. In Arduino IDE:

   * **Board**: *Raspberry Pi Pico 2 W*
   * USB upload as usual.
3. Files in the repo:

   * `Pico2W_Simple_Stepper3.ino` (main sketch; your file name may differ)
   * `stepper_1sm.pio` (the PIO program; compiled automatically to a header)

Open Serial Monitor at **115200** baud.

---

## Quick Start

Send a few G-code lines over Serial:

```
G1 X2000 Y1000 Z0 F2500
M114
G0 X0 Y0 Z0
G92 X0 Y0 Z0
```

* `G1` uses **F** as steps/second (along the master axis).
* `G0` runs with the “rapid” delays defined in the sketch.
* `M114` prints the **live** position from DMA progress.

Homing (simple demo):

```
M120   ; enable homing sensors (GPIO 20, 21, active-low)
G28    ; move to origin (0,0,0)
M121   ; disable homing sensors
```

Stops:

```
M2     ; force stop (queue preserved)
M0     ; force stop + clear queue
```

---

## Tuning (where to tweak)

In the sketch:

* **Profile timing**

  * `d_start_us`, `d_cruise_us`, `d_end_us` — start/cruise/end period (µs)
  * `accel_frac`, `decel_frac` — fraction of the master-axis steps for accel/decel
  * S-curve shape uses a smooth quintic (`smoothstep5`); edit if you want a different profile
* **Queue / buffer**

  * `BUFFER_SIZE` (default 512) controls how often we refill DMA (more frequent = finer “real-time” granularity)
* **Pins**

  * `DIR_INV_MASK` to flip DIR polarity per axis if a motor runs the wrong way

---

## Safety & Stopping

There are **two** stop paths in the code:

* **Soft stop** (`stop_motion_soft`)
  Let the current DMA burst and PIO FIFO **finish naturally** (no abort). Use when you just want to wait until idle before the next move.

* **Force stop** (`stop_motion_force`)
  For **E-stop / mid-move cancel**. On RP2350 we:

  1. **disable IRQ** for the channel and clear pending
  2. **clear the DMA EN bit** in `CTRL_TRIG` (errata workaround)
  3. call `dma_channel_abort()`
  4. clear spurious “done” IRQ if any
  5. reset the **PIO** SM and FIFOs

`M0` and `M2` use the **force stop**. `start_next_move()` **does not abort**; it starts the next move only when idle (recommended).

---

## Real-Time Position (what “TRUE” means here)

* The sketch advances X/Y/Z **only** for motion words that have actually **left the DMA channel** (i.e., were pushed to PIO).
* On buffer completion (DMA IRQ), it processes any **remaining** words from the active buffer.
* Asking for `M114` or the periodic status first calls `update_realtime_positions_from_dma()` to catch up to hardware.
* This produces **deterministic positions** even at high step rates, without software jitter.

---

## Notes & Limits

* **RP2350 vs RP2040**
  The force-stop path includes a register-level EN-bit clear before `abort()` to avoid retriggering on RP2350. On RP2040 you may see a “late” completion IRQ after abort; the sketch already acks/guards for that.

* **Timing**
  The PIO SM is configured at **1 MHz** (1 tick = 1 µs). The STEP high is fixed at **20 µs**; “extra delay” encodes the rest of the inter-step period.

* **Units**
  All positions are **steps** (absolute). Feed `F` is **steps/s**.

* **Homing**
  The homing demo is minimal—adapt to your switches/logic and add homing sequences as needed.

* **Drivers**
  Most A4988/DRV/TMC drivers accept 3.3 V logic. If your driver has **EN**, pull it to the correct level to enable outputs.

---

## Project Structure

```
/ (repo root)
├── Pico2W_Simple_Stepper3.ino    # main sketch (name may differ)
├── stepper_1sm.pio               # PIO program (compiled to .h by the core)
└── README.md
```

---

## License

MIT. See `LICENSE` (add your preferred license here).

---

## Credits

* Created by **Sjoerd Leewis**.
* Built for the Earle Philhower Arduino-Pico core.
* Thanks to the Raspberry Pi silicon docs and community examples around PIO + DMA streaming.

---

### Got ideas?

PRs and issues are welcome—especially improvements to the planner (arc support, look-ahead blending, jerk-limits per axis) or extra G-code commands.
