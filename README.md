# Autonomous Boat Controller

## Overview

A complete autonomous boat system combining GPS waypoint navigation, motion stabilization via accelerometer feedback, and real-time motor control. The system runs on a Jetson Nano and controls two thrusters via an Arduino Nano motor controller.

## System Architecture

### Modular Design Philosophy

The system is split into two independent units to maximize reliability and processing power:

#### **Jetson Nano (Brains)**

- Reads all sensors (GPS, IMU, camera)
- Performs all heavy processing (sensor fusion, error correction, path planning)
- Makes control decisions
- Logs all data
- Sends high-level motor commands over serial

**Advantages:**

- Unlimited compute for complex algorithms
- Can handle real-time computer vision
- Easy to debug and iterate on logic
- Not constrained by microcontroller memory

#### **Arduino Nano (Motor Controller)**

- Receives simple serial commands (M0/M1/T0/T1/S)
- Converts to PWM signals immediately
- Drives ESC outputs with predictable timing
- Minimal processing = predictable behavior

**Advantages:**

- Simple, reliable, low-latency response
- Won't get bogged down if Jetson is busy with vision processing
- Easy to swap firmware if needed
- Can operate standalone for manual testing

### Hardware

- **Jetson Nano** - Main controller (GPS, IMU, vision processing)
- **Arduino Nano** - Motor ESC controller (receives commands via USB serial)
- **Neo 7M GPS** - 1Hz location data (USB serial)
- **MPU6050** - Accelerometer/Gyro for orientation (I2C)
- **Camera** - Image recognition (future integration)
- **Dual Thrusters** - Differential drive for movement and turning

### Communication Flow

```
Jetson Nano                          Arduino Nano
├─ Read GPS (1Hz)
├─ Read MPU6050 (20Hz)
├─ Calculate corrections (10Hz)
└─ Send serial command ─────────→ Parse command
                                  ├─ Extract direction/PWM
                                  └─ Output PWM to ESCs
```

## How It Works

### 1. GPS Waypoint Navigation (Future)

- Reads Neo 7M GPS data continuously
- Parses NMEA RMC sentences for lat/lon/heading
- (Will calculate bearing to target waypoint and issue forward commands)

### 2. Orientation Correction (Active)

- **Target Heading**: From GPS (current boat heading)
- **Actual Heading**: From MPU6050 gyro (yaw rate)
- **Error Calculation**: `error = desired - actual`
- **Proportional Control**: If error exceeds deadzone, send turn command proportional to error magnitude

#### Example

```
Desired heading: 45° (Northeast)
Actual heading:  50° (drifting right)
Error:          -5° (correct with left turn)
→ Send: T01600 (turn left at 1600 µs PWM)
```

### 3. Motor Commands

Commands sent via serial (115200 baud) to Arduino Nano:

| Command | Function      | Example                       |
| ------- | ------------- | ----------------------------- |
| M0XXXX  | Move Forward  | M01800 = forward at 1800µs    |
| M1XXXX  | Move Backward | M11200 = backward at 1200µs   |
| T0XXXX  | Turn Left     | T01500 = left turn at 1500µs  |
| T1XXXX  | Turn Right    | T11600 = right turn at 1600µs |
| S00000  | Stop          | S00000                        |

PWM Range: **1100µs (min movement) to 2000µs (max speed)**

### ⚠️ ESC Reversing Capability

**IMPORTANT**: Not all ESCs support reversing (M1 backward command). Before using the boat:

1. **Test your ESC** - Send M1 commands manually to see if thrusters reverse
2. **Check ESC documentation** - Some ESCs have firmware options for bidirectional mode
3. **If your ESC doesn't reverse** - Set the `ENABLE_REVERSAL` flag in `boat_controller.py`:

```python
ENABLE_REVERSAL = False  # Set to False if ESC cannot reverse
```

When `ENABLE_REVERSAL = False`:

- M1 (backward) commands are ignored
- Only forward (M0), turns (T0/T1), and stop (S) are used
- The boat can still correct orientation via differential turning

## Installation

### Requirements

```bash
python3
pip install -r requirements.txt
```

### On Jetson Nano

```bash
cd ~/boat_control
pip install -r requirements.txt
python3 boat_controller.py
```

## Configuration (Edit Top of `boat_controller.py`)

```python
DEADZONE = 10.0              # Degrees - how much drift before correcting (test to find sweet spot)
ERROR_GAIN = 1.0             # PWM change per degree of error (tune for stability)
LOOP_FREQ = 10               # Main loop speed (Hz)
CORRECTION_PWM_MIN = 1100    # Minimum motor PWM (tested minimum)
CORRECTION_PWM_MAX = 2000    # Maximum motor PWM
ENABLE_REVERSAL = True       # Set to False if your ESC cannot reverse
```

### Tuning Guide

- **If boat oscillates**: Reduce DEADZONE or ERROR_GAIN
- **If boat drifts too much**: Increase ERROR_GAIN or reduce DEADZONE
- **Too sluggish**: Increase ERROR_GAIN
- **Too twitchy**: Reduce ERROR_GAIN or increase DEADZONE

## Data Logging

All data is logged to a **single CSV file per session** at:

```
./logs/boat_log_YYYYMMDD_HHMMSS.csv
```

The timestamp is set when the script starts, so all entries collected during one run go to the same file. A new file is created each time you start `boat_controller.py`.

Each row contains:

- **timestamp** - When the entry was logged
- **GPS latitude, longitude, heading** - Position and desired bearing
- **gyro_rate** - Yaw rotation rate (degrees/sec)
- **accel_pitch, accel_roll** - Boat tilt angles
- **yaw_fused** - Estimated heading from sensor fusion
- **yaw_error** - Difference between desired and actual heading
- **motor_command** - What command was sent to thrusters

**Use this for post-flight analysis:**

- Plot yaw_error vs time to see drift patterns
- Compare desired vs actual heading for tuning feedback
- Identify sensor calibration issues
- Validate proportional gain settings

## File Structure

```
boat_bypass_t1/
├── boat_bypass_t1.ino          # Arduino motor controller firmware
│                               # - Ultra-simple: parse serial commands, output PWM
│                               # - No sensor processing, no complex logic
│
├── boat_controller.py          # Jetson main controller script
│                               # - All heavy lifting here
│                               # - GPS parsing, IMU fusion, error correction, logging
│
├── requirements.txt            # Python dependencies
├── SerialParser.h              # (Arduino auxiliary header)
└── logs/                       # CSV log files (auto-created)
```

### Separation of Concerns

| Component       | Responsibility             | Why?                                                     |
| --------------- | -------------------------- | -------------------------------------------------------- |
| **Arduino**     | Parse serial → PWM output  | Keep simple & reliable. No delays from sensor processing |
| **Jetson**      | All sensor reading & logic | Unlimited CPU, can handle complex algorithms easily      |
| **Serial Link** | Thin command protocol      | Fast (115200 baud), simple to parse, minimal overhead    |

**Result:** Even if Jetson is busy with vision processing, motor commands execute immediately with predictable timing.

## Architecture Notes

### Threading

- **GPS Reader Thread**: Continuously reads Neo 7M (1Hz updates)
- **IMU Reader Thread**: Continuously reads MPU6050 (20Hz sampling)
- **Main Loop**: 10Hz control loop with proportional correction
- **Motor Sender**: Synchronous serial to Arduino

Thread-safe data exchange via locks prevents race conditions.

### Future Integration: Camera

When adding image recognition:

1. Spawn camera thread (like GPS/IMU threads)
2. Store vision data (detected objects, obstacles) in global dict with lock
3. Main loop checks vision data and adjusts motor commands accordingly
4. Log vision detections alongside GPS/IMU data

## Troubleshooting

**No GPS data?**

- Check Neo 7M is getting power and has serial connection
- Verify `/dev/ttyUSB1` is correct port (use `ls /dev/ttyUSB*`)

**Motors not responding?**

- Verify Arduino is receiving commands: check USB connection
- Confirm motor PWM values are in valid range (1100-2000)
- Check ESC power supply voltage

**Excessive drift/oscillation?**

- Adjust DEADZONE and ERROR_GAIN (see Tuning Guide)
- Verify MPU6050 calibration (accel may need offset adjustment)

**Log file not created?**

- Ensure `./logs/` directory exists or script has write permissions
- Check disk space on Jetson

---

**Built for autonomous marine robotics. Extending rapidly.**
