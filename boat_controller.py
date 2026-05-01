#!/usr/bin/env python3
"""
Boat Autonomous Controller - Jetson Nano
Reads GPS (Neo 7M), Accelerometer (MPU6050), and controls motors via Arduino Nano
"""

import serial
import time
import csv
import math
from datetime import datetime
from threading import Thread, Lock

# Try importing MPU6050 - graceful fallback if not available
try:
    from mpu6050 import mpu6050
    MPU_AVAILABLE = True
except ImportError:
    MPU_AVAILABLE = False

# Check for serial
try:
    import serial
    SERIAL_AVAILABLE = True
except ImportError:
    SERIAL_AVAILABLE = False

# ============================================================================
# CONFIG - TUNE THESE DURING TESTING
# ============================================================================
DEADZONE = 10.0              # degrees - acceptable drift before correction
ERROR_GAIN = 1.0             # PWM adjustment per degree of error
LOOP_FREQ = 10               # Hz
MOTOR_PWM_BASE = 1800        # Base forward command
CORRECTION_PWM_MIN = 1100    # Min PWM for movement (motors stall below this)
CORRECTION_PWM_MAX = 2000    # Max PWM for turn corrections
ENABLE_REVERSAL = True       # Set to False if ESC cannot reverse (backward command)

# Complementary Filter (fuses gyro + accel for robust heading)
GYRO_WEIGHT = 0.98           # How much to trust gyro integration (vs accel)
ACCEL_WEIGHT = 0.02          # How much to trust accel-estimated heading

# Serial/I2C ports
MOTOR_PORT = "/dev/ttyUSB0"  # Arduino motor controller
MOTOR_BAUD = 115200          # Arduino serial speed
GPS_PORT = "/dev/ttyUSB1"    # Neo 7M GPS
MPU_ADDRESS = 0x68           # MPU6050 I2C address

# Logging
LOG_DIR = "./logs"
LOG_FILE = f"{LOG_DIR}/boat_log_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv"

# ============================================================================
# Globals with Thread Locks
# ============================================================================
gps_lock = Lock()
accel_lock = Lock()
motor_lock = Lock()

gps_data = {
    'latitude': 0.0,
    'longitude': 0.0,
    'heading': 0.0,
    'timestamp': time.time()
}

accel_data = {
    'yaw': 0.0,
    'pitch': 0.0,
    'roll': 0.0,
    'timestamp': time.time()
}

# Yaw fusion (complementary filter state)
yaw_integrated = 0.0          # Integrated gyro over time
yaw_last_time = time.time()   # For dt calculation
yaw_fused = 0.0               # Final fused heading estimate

motor_state = {
    'last_command': 'S00000',
    'timestamp': time.time()
}

# Global motor serial connection
motor_serial = None

# ============================================================================
# Motor Command Sender
# ============================================================================
def send_motor_command(command):
    """Send command to Arduino motor controller"""
    global motor_serial
    
    if motor_serial is None or not motor_serial.is_open:
        return
    
    try:
        motor_serial.write(f"{command}\n".encode('utf-8'))
        with motor_lock:
            motor_state['last_command'] = command
            motor_state['timestamp'] = time.time()
    except Exception as e:
        print(f"[MOTOR Error] Failed to send command: {e}")

# ============================================================================
# GPS Reader (NMEA parsing)
# ============================================================================
def read_gps_thread():
    """Read Neo 7M GPS data continuously"""
    global gps_data
    
    try:
        gps_serial = serial.Serial(GPS_PORT, 9600, timeout=1)
        print(f"[GPS] Connected to {GPS_PORT}")
    except Exception as e:
        print(f"[ERROR] Failed to open GPS: {e}")
        return
    
    while True:
        try:
            if gps_serial.in_waiting:
                line = gps_serial.readline().decode('utf-8', errors='ignore').strip()
                
                # Parse RMC sentence (lat, lon, heading)
                if line.startswith('$GNRMC') or line.startswith('$GPRMC'):
                    parts = line.split(',')
                    if len(parts) > 8 and parts[2] == 'A':  # Valid fix
                        try:
                            # Latitude
                            lat_str = parts[3]
                            lat_deg = float(lat_str[:2])
                            lat_min = float(lat_str[2:]) / 60.0
                            lat = lat_deg + lat_min
                            if parts[4] == 'S':
                                lat = -lat
                            
                            # Longitude
                            lon_str = parts[5]
                            lon_deg = float(lon_str[:3])
                            lon_min = float(lon_str[3:]) / 60.0
                            lon = lon_deg + lon_min
                            if parts[6] == 'W':
                                lon = -lon
                            
                            # Heading (track angle)
                            heading = float(parts[8]) if parts[8] else 0.0
                            
                            with gps_lock:
                                gps_data['latitude'] = lat
                                gps_data['longitude'] = lon
                                gps_data['heading'] = heading
                                gps_data['timestamp'] = time.time()
                            
                            print(f"[GPS] Lat: {lat:.6f}, Lon: {lon:.6f}, Heading: {heading:.1f}°")
                        except (ValueError, IndexError):
                            pass
        except Exception as e:
            print(f"[GPS Error] {e}")
            time.sleep(0.1)

# ============================================================================
# Accelerometer Reader (MPU6050)
# ============================================================================
def read_accel_thread():
    """Read MPU6050 IMU data continuously"""
    global accel_data
    
    if not MPU_AVAILABLE:
        print("[WARNING] MPU6050 reader disabled - library not available")
        return
    
    try:
        mpu = mpu6050(MPU_ADDRESS)
        print(f"[MPU6050] Connected at I2C address 0x{MPU_ADDRESS:02x}")
    except Exception as e:
        print(f"[ERROR] Failed to initialize MPU6050: {e}")
        return
    
    while True:
        try:
            accel = mpu.get_accel_data()
            gyro = mpu.get_gyro_data()
            
            # Gyro Z-axis is yaw rate (degrees/sec)
            yaw_rate = gyro['z']
            
            # Calculate pitch and roll from accelerometer
            ax, ay, az = accel['x'], accel['y'], accel['z']
            pitch = math.atan2(ax, math.sqrt(ay**2 + az**2)) * 180 / math.pi
            roll = math.atan2(ay, math.sqrt(ax**2 + az**2)) * 180 / math.pi
            
            with accel_lock:
                accel_data['pitch'] = pitch
                accel_data['roll'] = roll
                accel_data['yaw'] = yaw_rate
                accel_data['timestamp'] = time.time()
            
            time.sleep(0.05)  # 20 Hz sampling
        except Exception as e:
            print(f"[IMU Error] {e}")
            time.sleep(0.1)

# ============================================================================
# Complementary Filter for Yaw Estimation
# ============================================================================
def fuse_yaw_heading(gyro_rate, pitch, roll, dt):
    """
    Complementary filter: combines gyro integration with accel-estimated heading.
    
    Gyro: Fast response, drifts over time
    Accel: Slow response, stable long-term
    
    Result: Best of both worlds
    """
    global yaw_integrated, yaw_fused
    
    # Integrate gyro (dead reckoning)
    yaw_integrated += gyro_rate * dt
    
    # Estimate heading from accelerometer (assuming level boat)
    # In water, pitch/roll are small, so this is a rough estimate
    accel_heading = math.atan2(pitch, roll) * 180 / math.pi
    
    # Complementary filter: mostly trust gyro, nudge with accel
    yaw_fused = GYRO_WEIGHT * yaw_integrated + ACCEL_WEIGHT * accel_heading
    
    # Normalize to 0-360
    if yaw_fused < 0:
        yaw_fused += 360
    elif yaw_fused >= 360:
        yaw_fused -= 360
    
    return yaw_fused

# ============================================================================
# Main Control Loop
# ============================================================================
def main_loop():
    """10 Hz main control loop with sensor fusion and motor commanding"""
    global yaw_integrated, yaw_last_time, yaw_fused
    
    import os
    os.makedirs(LOG_DIR, exist_ok=True)
    
    with open(LOG_FILE, 'w', newline='') as f:
        writer = csv.writer(f)
        writer.writerow([
            'timestamp', 'latitude', 'longitude', 'gps_heading',
            'gyro_rate', 'accel_pitch', 'accel_roll', 'yaw_fused',
            'yaw_error', 'motor_command'
        ])
        
        print(f"[LOG] Writing to {LOG_FILE}")
        
        loop_start = time.time()
        cycle = 0
        
        try:
            while True:
                cycle_start = time.time()
                current_time = cycle_start
                
                # Get latest sensor data
                with gps_lock:
                    desired_heading = gps_data['heading']
                    gps_lat = gps_data['latitude']
                    gps_lon = gps_data['longitude']
                
                with accel_lock:
                    gyro_rate = accel_data['yaw']
                    pitch = accel_data['pitch']
                    roll = accel_data['roll']
                
                # Complementary filter: fuse gyro + accel
                dt = current_time - yaw_last_time
                if dt > 0:
                    yaw_last_time = current_time
                    actual_heading = fuse_yaw_heading(gyro_rate, pitch, roll, dt)
                else:
                    actual_heading = yaw_fused
                
                # Calculate yaw error (wrapped to ±180°)
                yaw_error = desired_heading - actual_heading
                while yaw_error > 180:
                    yaw_error -= 360
                while yaw_error < -180:
                    yaw_error += 360
                
                # Orientation correction
                motor_command = 'S00000'  # Default: stop
                
                if abs(yaw_error) > DEADZONE:
                    # Proportional correction: larger error = stronger turn
                    correction_magnitude = min(abs(yaw_error) * ERROR_GAIN, 500)
                    correction_pwm = int(CORRECTION_PWM_MIN + correction_magnitude)
                    correction_pwm = max(CORRECTION_PWM_MIN, min(CORRECTION_PWM_MAX, correction_pwm))
                    
                    # Determine direction: 0=left, 1=right
                    direction = '1' if yaw_error > 0 else '0'
                    motor_command = f"T{direction}{correction_pwm}"
                
                # Send motor command
                send_motor_command(motor_command)
                
                # Log data
                with motor_lock:
                    last_cmd = motor_state['last_command']
                
                writer.writerow([
                    datetime.now().isoformat(),
                    f"{gps_lat:.8f}",
                    f"{gps_lon:.8f}",
                    f"{desired_heading:.2f}",
                    f"{gyro_rate:.2f}",
                    f"{pitch:.2f}",
                    f"{roll:.2f}",
                    f"{actual_heading:.2f}",
                    f"{yaw_error:.2f}",
                    last_cmd
                ])
                f.flush()
                
                # Maintain 10 Hz loop
                cycle_time = time.time() - cycle_start
                sleep_time = (1.0 / LOOP_FREQ) - cycle_time
                if sleep_time > 0:
                    time.sleep(sleep_time)
                
                cycle += 1
                if cycle % 10 == 0:
                    elapsed = time.time() - loop_start
                    print(f"[MAIN] Cycle {cycle} | "
                          f"Actual: {actual_heading:.1f}° | "
                          f"Error: {yaw_error:+.1f}° | "
                          f"Command: {motor_command}")
        
        except KeyboardInterrupt:
            print("\n[SHUTDOWN] Stopping...")
            send_motor_command('S00000')

# ============================================================================
# Motor Connection Setup
# ============================================================================
def setup_motor_connection():
    """Establish connection to Arduino and send reset command"""
    global motor_serial
    
    try:
        print(f"[MOTOR] Connecting to {MOTOR_PORT} @ {MOTOR_BAUD} baud...")
        motor_serial = serial.Serial(MOTOR_PORT, MOTOR_BAUD, timeout=1)
        print(f"[MOTOR] Connected!")
        
        # Wait for Arduino to boot
        time.sleep(1)
        
        # Send reset command to arm thrusters
        print("[MOTOR] Sending reset command (R000) to arm thrusters...")
        send_motor_command('R000')
        
        # Wait for reset to complete
        time.sleep(1)
        
        print("[MOTOR] Ready for commands")
        return True
    
    except Exception as e:
        print(f"[ERROR] Failed to connect to motor controller: {e}")
        return False

# ============================================================================
# Dependency Check
# ============================================================================
def check_dependencies():
    """Check required libraries and warn if missing"""
    print("="*60)
    print("Boat Autonomous Controller - Jetson Nano")
    print("="*60)
    print("\n[STARTUP] Checking dependencies...\n")
    
    errors = []
    warnings = []
    
    if not SERIAL_AVAILABLE:
        errors.append("pyserial - install with: pip install pyserial")
    else:
        print("✓ pyserial")
    
    if not MPU_AVAILABLE:
        warnings.append("mpu6050 - IMU disabled. Install with: pip install mpu6050-raspberrypi")
        print("✗ mpu6050 (optional - IMU features disabled)")
    else:
        print("✓ mpu6050")
    
    if errors:
        print("\n[ERROR] Missing required dependencies:")
        for err in errors:
            print(f"  - {err}")
        print(f"\nInstall all with: pip install -r requirements.txt")
        return False
    
    if warnings:
        print("\n[WARNING] Optional dependencies not found:")
        for warn in warnings:
            print(f"  - {warn}")
    
    print()
    return True

# ============================================================================
# Startup
# ============================================================================
if __name__ == "__main__":
    if not check_dependencies():
        exit(1)
    
    print(f"Config: Deadzone={DEADZONE}°, Gain={ERROR_GAIN}, Loop={LOOP_FREQ}Hz\n")
    
    # Connect to motor controller and arm thrusters
    if not setup_motor_connection():
        exit(1)
    
    # Start background threads
    gps_thread = Thread(target=read_gps_thread, daemon=True)
    accel_thread = Thread(target=read_accel_thread, daemon=True)
    
    gps_thread.start()
    accel_thread.start()
    
    # Give threads time to initialize
    time.sleep(2)
    
    # Start main control loop
    main_loop()
