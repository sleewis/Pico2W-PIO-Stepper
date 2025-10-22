Pico Stepper Controller with Move Queue
A high-performance stepper motor controller for Raspberry Pi Pico 2W (RP2350) using PIO and DMA for smooth, non-blocking motion control with G-code support.

Features
🔄 Multi-Axis Control: Simultaneous control of 3 stepper motors (X, Y, Z)

⚡ Hardware Acceleration: Uses RP2350's PIO (Programmable I/O) for precise step timing

🚀 DMA-Driven: Direct Memory Access for zero-CPU-overhead step generation

📋 Move Queue: Queue multiple moves for continuous, non-blocking motion

🎛️ G-code Support: Standard G-code commands for easy integration

📈 Advanced Motion: Quadratic acceleration/deceleration profiles for smooth motion

📊 Real-time Monitoring: Live position display with queue status

Hardware Requirements
Board: Raspberry Pi Pico 2W (RP2350)

IDE: Arduino IDE with Earle Philhower core

Stepper Drivers: Any step/direction drivers (DRV8825, TMC2209, A4988, etc.)

Power: Appropriate power supply for your stepper motors

Pin Connections

GPIO	Function	Stepper Driver

0	X-DIR	DIR pin

1	Y-DIR	DIR pin

2	Z-DIR	DIR pin

3	X-STEP	STEP pin

4	Y-STEP	STEP pin

5	Z-STEP	STEP pin

Note: Connect stepper driver ENABLE pins as needed for your setup.

Installation
Install Arduino Core: Use the Earle Philhower Pico Arduino core

Add PIO Program: Ensure stepper_1sm.pio.h is in your project directory

Upload Code: Compile and upload the sketch to your Pico 2W

Connect Serial: Open Serial Monitor at 115200 baud

G-code Commands

Motion Commands

G0 X.. Y.. Z.. - Rapid move (queued)

G1 X.. Y.. Z.. F.. - Controlled move with feedrate (queued)

G28 - Home to origin (0,0,0)

Control Commands

M0 - Stop motion and clear queue

M2 - Stop motion (preserve queue)

M110 - Clear move queue

M111 - Show queue status

M114 - Report current position

Examples

gcode

G1 X1000 Y500 Z0 F800    ; Move to (1000,500,0) at 800 steps/sec

G1 X0 Y0 Z0 F1000        ; Return to origin

M114                     ; Check position


Key Features

Move Queue System

32-move queue capacity

Automatic sequencing - moves execute continuously

Non-blocking - send commands while motors are moving

Queue management - stop, clear, or check status anytime

Advanced Motion Control

Quadratic easing for smooth acceleration/deceleration

Trapezoidal velocity profiles with configurable curves

Bresenham DDA algorithm for multi-axis interpolation

Configurable acceleration (35% by default)

Hardware Optimization

PIO state machine handles precise step timing

DMA transfers free CPU for other tasks

Real-time position tracking during motion

Efficient buffer management with interrupt-driven refills

Performance

Step Rate: Up to 50,000+ steps/second

Pulse Width: Fixed 20μs step pulses in PIO

Timing Resolution: 1μs precision

Update Rate: 100ms position display updates

Configuration

Tuning Motion Parameters

Adjust in the handleLine function:

cpp

m.accel_frac = 0.35f;   // Acceleration distance (35% of move)

m.decel_frac = 0.35f;   // Deceleration distance (35% of move)

m.d_start_us = 2000;    // Start delay (slower acceleration)

m.d_cruise_us = 1000;   // Cruise speed delay

Direction Inversion

If motors move backwards:

cpp

#define DIR_INV_MASK (1u<<0)  // Invert X axis

#define DIR_INV_MASK (1u<<1)  // Invert Y axis  

#define DIR_INV_MASK (1u<<2)  // Invert Z axis

Project Structure

text

pico-stepper-controller/

├── Pico_stepper_1sm_v2.ino    # Main Arduino sketch

├── stepper_1sm.pio.h          # PIO assembly program

└── README.md                  # This file


Applications

3D Printers & CNC Machines

Robotics & Automation

Precision positioning systems

Laser cutters & plotters

Custom motion control projects

Troubleshooting

Motors not moving?

Check driver power and enable pins

Verify DIR/STEP connections

Check serial baud rate (115200)

Jerk motion?

Increase acceleration fractions (0.4-0.6)

Adjust start/end delays

Queue full?

Increase MOVE_QUEUE_SIZE in code

Send M110 to clear queue

License

Open source - feel free to modify for your projects!

Contributing

Contributions welcome! Please feel free to submit pull requests or open issues for bugs and feature requests.

Happy Making! 🛠️ If this project helps you, please give it a ⭐ on GitHub!
