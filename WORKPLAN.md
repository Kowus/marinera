# Boat Control System - Features & Workplan

**Last Updated:** May 5, 2026

---

## Current System Status: ✅ FULLY OPERATIONAL (Hardware)

### ✅ Completed Features

#### Hardware Integration
- **CSI Camera**: 640x480@30fps MJPEG streaming (~100-200ms latency local, network-dependent for remote)
- **Neo 7M GPS**: NMEA RMC parsing, live lat/lon/heading, 9600 baud
- **MPU6050 IMU**: Gyro/accel continuous at 20Hz, I2C 0x68
- **Motor Controllers**: Left/right PWM via Arduino (1100-2000µs range)
- **Vortex Device**: GPIO control via serial (V1=ON, V0=OFF)
- **Sensor Fusion**: Complementary filter (98% gyro + 2% accel) for stable heading

#### Manual Control (Web Dashboard)
- Real-time camera feed with overlay
- Vertical speed slider (10-100%)
- Joystick grid (Forward, Back, Left, Right, Stop)
- Live sensor display (GPS, heading, pitch, roll, compass)
- System arm/disarm with vortex integration
- Dry-run mode for testing without motors
- Manual/Autonomous mode selector

#### Hardware Control
- Motor commands: Forward (M0), Backward (M1), Turn-Left (T0), Turn-Right (T1), Stop (S)
- Vortex integration: Arming enables vortex, disarming disables it
- Reset command (R) to Arduino on system arm
- 115200 baud serial to Arduino Nano

#### Logging & Monitoring
- CSV logging with timestamp, GPS, sensor data, motor commands
- Live web dashboard with sensor telemetry
- Dry-run testing mode

---

## 🧪 Phase 1: Sensor Fusion Validation (Next - Before Autonomous)

**Objective:** Verify gyro/accel fusion produces stable, accurate heading for autonomous steering

**Test Procedure:**
1. Deploy on water with manual control active
2. Monitor dashboard:
   - Watch "Heading (Fused)" and compass needle
   - Look for smooth updates (no jitter, no sudden jumps)
   - Verify stability when stationary (should not drift rapidly)
   - Test tracking during manual turns (should follow smoothly)
3. Analyze results:
   - Review `logs/boat_log_*.csv` after test
   - Compare `gps_heading` vs `yaw_fused` (should be similar when moving straight)
   - Check `gyro_rate` and accel trends for noise/stability

**Success Criteria:**
- ✅ Fused heading updates at ~10Hz without stuttering (PASS)
- ✅ Stable ±5° when holding stationary (PASS)
- ⚠️ Smooth tracking during manual turns (LIMITED by current propellers)

**Tuning Parameters** (if needed):
- `GYRO_WEIGHT = 0.98` (currently favors gyro, low-passes accel noise)
- `ACCEL_WEIGHT = 0.02` (currently minimal accel correction)
- `DEADZONE = 10.0°` (allowed heading drift before error correction)

**Validated Performance:**
- Heading sensor fusion is stable and accurate for autonomous control
- Straight-line stability excellent
- Turning performance will improve significantly with propeller upgrade (current props limit steering authority)

---

## 🤖 Phase 2: Minimalist Autonomous Features (After Sensor Validation)

### Feature 1: Heading Hold (Priority: 🔴 HIGH - simplest to implement & test)

**Purpose:** Boat maintains a set heading without drifting

**How It Works:**
1. User selects target heading via web UI (0-360°)
2. System calculates heading error: `error = target - actual`
3. Steering loop adjusts left/right motor PWM proportional to error
4. Uses `ERROR_GAIN` to tune responsiveness

**Web UI Changes Needed:**
- Heading Hold toggle button
- Target heading input (spinner 0-360°, or click compass to set)
- Status display: "Target: 45° | Actual: 43° | Error: 2°"

**Motor Control Logic:**
- Error < DEADZONE (±10°): Hold straight (M0 = base forward)
- Error > +DEADZONE: Turn right (reduce right PWM, increase left)
- Error < -DEADZONE: Turn left (reduce left PWM, increase right)
- PWM adjustment: `correction = error * ERROR_GAIN` (capped 1100-2000)

**Testing:**
- Dry-run first (verify command logic doesn't cause erratic behavior)
- Manual test: set heading, verify boat turns to target & holds
- Fine-tune `ERROR_GAIN` (0.5-2.0) based on responsiveness

**Implementation Complexity:** Low (~50 lines in motor loop)

---

### Feature 2: Waypoint Navigation (Priority: 🟡 MEDIUM)

**Purpose:** Navigate to GPS waypoints sequentially

**How It Works:**
1. User uploads waypoints via web UI (array of [lat, lon])
2. System calculates bearing to first waypoint
3. Heading Hold logic guides boat toward bearing
4. When within `WAYPOINT_RADIUS` (default 5m), advance to next waypoint
5. Mission complete when all waypoints visited

**Web UI Changes Needed:**
- Waypoint list editor (add/remove/reorder)
- Mission upload button
- Live mission status: "Waypoint 2/4 | Distance: 12m"
- Map shows waypoints + route

**Motor Control Logic:**
- Calculate GPS bearing to target: `bearing = atan2(lon_delta, lat_delta)` (convert to 0-360)
- Set Heading Hold target to bearing
- Track distance to waypoint (Haversine formula)
- Auto-advance when distance < threshold

**Testing:**
- Dry-run with dummy GPS coords
- Manual water test with 2-3 waypoints
- Verify bearing calculations match compass

**Implementation Complexity:** Medium (~100 lines for bearing calc + waypoint manager)

---

### Feature 3: Box Mission (Priority: 🟢 MEDIUM-LOW - demo after Heading Hold + Waypoint Nav work)

**Purpose:** Execute predefined rectangular route as autonomous demo

**How It Works:**
- 4 hardcoded waypoints forming a square/rectangle around deployment area
- User clicks "Start Box Mission" button
- System executes Waypoint Navigation with these 4 points

**Configuration:**
- `BOX_MISSION_ORIGIN`: Starting GPS lat/lon (user's position)
- `BOX_SIZE`: Distance in meters (e.g., 50m square)
- `BOX_MISSION_WAYPOINTS`: Auto-calculated from origin + size

**Testing:**
- Verify Heading Hold & Waypoint Nav first
- Deploy in open water away from obstacles
- Monitor map for correct route execution

**Implementation Complexity:** Very Low (~20 lines, mostly reuses Waypoint Nav)

---

## 🔧 Motor Calibration & Tuning (Deferred to After Prop Upgrade)

**⚠️ CRITICAL DEPENDENCY:** Propeller upgrade is the top priority bottleneck

**Currently Known Issues:**
- ❌ Current props severely limit turning authority (impacts autonomous steering)
  - **Root Cause:** Rush-designed blades capture water sideways instead of cutting through it
  - Result: More sideways displacement than forward thrust → sluggish steering response
- Current props sub-optimal for efficiency/speed
- Motor PWM ranges may need recalibration with new props
- Steering responsiveness directly limited by propeller performance

**Expected Improvement with New Props:**
- Proper blade pitch/angle to cut water → efficient thrust vectoring
- Significantly improved turning authority (3-5x better expected)
- Heading Hold will respond much faster to steering corrections
- Waypoint navigation turns will be smooth and quick

**Impact on Autonomous Features:**
- **Heading Hold:** Will work but may oscillate or respond slowly with current props
- **Waypoint Nav:** Turning to waypoint bearings will be sluggish
- **Box Mission:** Turns will take longer, potentially missing waypoint radius

**Post-Propeller-Upgrade Workflow:**
1. Install upgraded propellers
2. Re-calibrate motor PWM ranges (verify forward/back/turn work smoothly)
3. Re-test manual control (should feel significantly more responsive)
4. Test all autonomous modes with new props
5. Fine-tune steering parameters below based on actual performance

**Parameters to Tune Later:**
```python
MOTOR_PWM_BASE = 1800          # Adjust after new props tested
CORRECTION_PWM_MIN = 1100      # Min PWM for movement
CORRECTION_PWM_MAX = 2000      # Max PWM for turn corrections
ERROR_GAIN = 1.0               # Steering responsiveness (0.5-2.0)
DEADZONE = 10.0                # Acceptable heading drift (degrees)
ENABLE_REVERSAL = True         # Verify with new props
```

**Tuning Procedure** (after props):
1. Manual motor test - verify forward/back/turn commands
2. Note any asymmetry (left vs right motor speed)
3. Adjust `MOTOR_PWM_BASE` to match desired speed
4. Test Heading Hold with small `ERROR_GAIN` (0.5), increase if needed
5. Adjust `DEADZONE` if steering is over/under-responsive

---

## 📋 Implementation Checklist

### Phase 1: Sensor Fusion Test ✅ COMPLETE
- [x] System deployed and tested
- [x] Sensor fusion validation complete (water test)
- [x] CSV logs analyzed and stable
- ⚠️ **Note:** Turning performance limited by current propellers; will improve with upgrade

### Phase 2: Heading Hold 🏗️ PLANNED
- [ ] Add heading hold toggle to web UI
- [ ] Add target heading input (spinner or compass click)
- [ ] Implement steering loop in motor control thread
- [ ] Dry-run testing
- [ ] Water testing with manual mode validation
- [ ] Tune ERROR_GAIN based on prop performance
- [ ] Commit: `feat: Add heading hold autonomous mode`

### Phase 3: Waypoint Navigation 🏗️ PLANNED
- [ ] Add waypoint editor to web UI
- [ ] Implement bearing calculation (atan2 + compass correction)
- [ ] Implement distance-to-waypoint check (Haversine)
- [ ] Implement waypoint sequencing logic
- [ ] Dry-run testing with dummy GPS
- [ ] Water testing with real GPS
- [ ] Commit: `feat: Add waypoint navigation mode`

### Phase 4: Box Mission 🏗️ PLANNED
- [ ] Calculate box waypoints from origin + size
- [ ] Add mission start button to web UI
- [ ] Wire to Waypoint Navigation logic
- [ ] Water testing
- [ ] Commit: `feat: Add box mission demo mode`

### Post-Deployment: Motor Tuning ⏸️ DEFERRED
- [ ] Install upgraded propellers
- [ ] Re-calibrate motor PWM ranges
- [ ] Re-test manual control
- [ ] Fine-tune ERROR_GAIN, DEADZONE, MOTOR_PWM_BASE
- [ ] Re-validate all autonomous modes with new props

---

## 🎯 Success Metrics

**Sensor Fusion:**
- Heading stable ±5° when stationary
- Smooth tracking during manual turns

**Heading Hold:**
- Boat maintains set heading within ±10° for 30+ seconds
- No motor oscillation or jitter

**Waypoint Navigation:**
- Accurate bearing calculation (within 5°)
- Smooth transitions between waypoints
- Detects waypoint arrival within acceptable radius

**Box Mission:**
- Completes 4-waypoint circuit without user intervention
- Returns near deployment area ±10m

---

## 📝 Notes

- ✅ Sensor fusion validated in water - ready for autonomous steering control
- ⚠️ **Propeller upgrade is critical:** Will significantly improve steering response for Heading Hold & Waypoint Nav
- All autonomous features inherit manual safety: arm/disarm via web, dry-run mode available
- Vortex control integrated: enables on arm, disables on disarm
- Logging captures all sensor data for post-mission analysis
- Motor params tunable without code recompilation (edit boat_service.py CONFIG section)

---

## 🚀 Recommended Path Forward

1. **Immediate (Next 1-2 weeks):** Implement Heading Hold autonomous mode
   - Can be dry-run tested now without water
   - Will be ready to test as soon as propellers arrive
   - Good "proof of concept" before scaling to waypoint nav
   - Use low `ERROR_GAIN` (0.5-0.7) initially to account for weak current props

2. **Parallel:** Install new propellers
   - Test manual control with new props
   - Recalibrate PWM ranges if needed
   - Measure improvement in turning responsiveness

3. **Follow-up:** Validate Heading Hold with new props
   - Increase `ERROR_GAIN` to 1.0-1.5 with better props
   - Fine-tune `DEADZONE` for responsiveness

4. **Then:** Implement Waypoint Nav and Box Mission
   - Should be smooth once Heading Hold validated

---

## 🔗 Key Code Locations

| Component | File | Lines |
|-----------|------|-------|
| Motor command loop | boat_service.py | 315-410 |
| Sensor fusion | boat_service.py | 350-375 |
| GPS parsing | boat_service.py | 175-220 |
| IMU reading | boat_service.py | 250-290 |
| Web API endpoints | boat_service.py | 495-600 |
| Web dashboard | templates/control.html | 1-1100 |
| Arduino motor driver | boat_bypass_t1.ino | 1-150 |
| Vortex control | boat_bypass_t1.ino | 55-120 |
