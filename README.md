**Note:** Apologies for the numerous commits to this repository. The device is remote and the only way to test was for my buddy to run `git pull` on their machine, as their network doesn't support SSH or conventional remote work ports. This made the initial project setup and testing process quite involved, resulting in frequent commits during development and debugging.

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

## Installation & Running

### Requirements

```bash
python3
pip install -r requirements.txt
```

### On Jetson Nano

**Before running, verify your serial port assignments:**

```bash
ls /dev/tty*
```

You'll typically see:

- `/dev/ttyACM0` - GPS module (appears as ACM even though connected via USB)
- `/dev/ttyUSB0` - Arduino (USB-to-serial adapter)

Update `GPS_PORT` and `MOTOR_PORT` in `boat_service.py` if your system differs.

### Launch the Web Service

```bash
cd ~/boat_control
pip install -r requirements.txt
python3 boat_service.py
```

Once running, access the web interface from any device on the same network:

```
http://<jetson-ip>:5000
```

Or locally on the Jetson:

```
http://localhost:5000
```

### Web Interface Features

**Manual Control Mode (Default):**

- 🎮 Quick-access buttons: Forward, Back, Left, Right, Stop
- 🎚️ Speed slider (0-100%)
- 🔓 Arm/Disarm system before sending commands
- 📊 Live sensor display: GPS, heading, pitch/roll
- 🔄 Real-time motor command feedback

**Controls:**

- Hold forward/back/left/right buttons to move (release to stop)
- Adjust speed slider before sending commands
- View current GPS position and boat orientation
- Monitor last command sent to Arduino

### Autonomous Mode (Future)

Once implemented, switch from the web interface to unlock:

- 🗺️ Waypoint mission planning
- 📍 GPS navigation with heading correction
- 📊 Live mission status tracking
- 💾 Mission history and logs

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
├── boat_service.py             # Jetson main unified service
│                               # - Flask web server (localhost:5000)
│                               # - Manual + Autonomous control (mode toggle)
│                               # - GPS parsing, IMU fusion, error correction
│                               # - CSV logging, real-time dashboard
│
├── templates/
│   └── control.html            # Web interface (served by Flask)
│                               # - Control buttons, speed slider
│                               # - Live sensor display
│                               # - Arm/disarm & mode switching
│
├── boat_controller.py          # (Legacy) Autonomous-only controller
│                               # - Use boat_service.py instead
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

## Known Issues

### GPS Antenna Performance by Region

The Neo 7M GPS module's built-in antenna has limited range and may struggle in certain regions. **Users in regions far from the equator or with challenging RF environments (e.g., West Africa, high latitudes) may experience weak or no satellite lock.**

**Example:** Testing in Ghana required an external GPS antenna for reliable signal acquisition. Without it, the module would fail to lock or maintain fix.

**Solution: Use an Active Antenna**

- **Strongly recommended:** Purchase an active (powered) GPS antenna instead of passive
- With active antenna: Lock time reduced to **~5 seconds max**, even when lying flat on the boat
- Active antennas have built-in low-noise amplifiers, significantly improving sensitivity
- Position the antenna with clear sky view (away from buildings, trees, water)
- Ensure your power supply can handle the additional current draw (~30mA typical)
- Test antenna performance before deploying the full system

## Troubleshooting

**No GPS data?**

- Check Neo 7M is getting power and has serial connection
- **Verify the GPS port:** Run `ls /dev/tty*` and confirm GPS appears on `/dev/ttyACM0` (or note its actual port)
  - GPS modules typically appear as `/dev/ttyACM0` even when connected via USB
  - Update `GPS_PORT` in `boat_controller.py` if needed
- **Regional note:** In areas with poor satellite coverage (equatorial/high latitude regions), consider using an external GPS antenna

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
