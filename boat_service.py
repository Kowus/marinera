#!/usr/bin/env python3
"""
Unified Boat Control Service - Jetson Nano
Web portal + Manual/Autonomous control from single Flask app
"""

import serial
import time
import csv
import math
import threading
from datetime import datetime
from threading import Thread, Lock
from flask import Flask, render_template, jsonify, request
from queue import Queue

# Try importing OpenCV for camera - graceful fallback if not available
try:
    import cv2
    CAMERA_AVAILABLE = True
except ImportError:
    CAMERA_AVAILABLE = False
    print("[WARNING] OpenCV not available - camera feed will be disabled")

# Try importing MPU6050 - graceful fallback if not available
try:
    from mpu6050 import mpu6050
    MPU_AVAILABLE = True
except ImportError:
    MPU_AVAILABLE = False

# ============================================================================
# CONFIG - TUNE THESE DURING TESTING
# ============================================================================
DEADZONE = 10.0              # degrees - acceptable drift before correction
ERROR_GAIN = 1.0             # PWM adjustment per degree of error
LOOP_FREQ = 10               # Hz
MOTOR_PWM_BASE = 1800        # Base forward command
CORRECTION_PWM_MIN = 1100    # Min PWM for movement (motors stall below this)
CORRECTION_PWM_MAX = 2000    # Max PWM for turn corrections
ENABLE_REVERSAL = True       # Set to False if ESC cannot reverse

# Complementary Filter (fuses gyro + accel for robust heading)
GYRO_WEIGHT = 0.98           # How much to trust gyro integration (vs accel)
ACCEL_WEIGHT = 0.02          # How much to trust accel-estimated heading

# Serial/I2C ports
MOTOR_PORT = "/dev/ttyUSB0"  # Arduino motor controller
MOTOR_BAUD = 115200          # Arduino serial speed
GPS_PORT = "/dev/ttyACM0"    # Neo 7M GPS (check your system with: ls /dev/tty* before running)
MPU_ADDRESS = 0x68           # MPU6050 I2C address

# Logging
LOG_DIR = "./logs"
LOG_FILE = f"{LOG_DIR}/boat_log_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv"

# ============================================================================
# Flask App Setup
# ============================================================================
app = Flask(__name__)

# ============================================================================
# Globals with Thread Locks
# ============================================================================
gps_lock = Lock()
accel_lock = Lock()
motor_lock = Lock()
state_lock = Lock()

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

yaw_integrated = 0.0          # Integrated gyro over time
yaw_last_time = time.time()   # For dt calculation
yaw_fused = 0.0               # Final fused heading estimate

motor_state = {
    'last_command': 'S00000',
    'timestamp': time.time()
}

# Control mode and command queue
control_state = {
    'mode': 'MANUAL',          # MANUAL or AUTONOMOUS
    'speed': 50,               # 0-100%
    'enabled': False,          # System armed?
    'dry_run': False           # Dry run mode (no serial, just log)
}

motor_queue = Queue()          # Commands from web interface
motor_serial = None

# Camera feed (MJPEG stream)
camera = None
camera_lock = Lock()
camera_frame = None

# ============================================================================
# Motor Command Sender
# ============================================================================
def send_motor_command(command, dry_run=False):
    """Send command to Arduino motor controller (or simulate in dry-run)"""
    global motor_serial
    
    # Always update state for display
    with motor_lock:
        motor_state['last_command'] = command
        motor_state['timestamp'] = time.time()
    
    # Parse command for logging
    direction_map = {'M0': 'Forward', 'M1': 'Backward', 'T0': 'Left', 'T1': 'Right', 'S': 'Stop'}
    prefix = command[:2]
    direction = direction_map.get(prefix, 'Unknown')
    pwm = command[2:] if len(command) > 2 else 'N/A'
    
    # Log the command
    if dry_run:
        print(f"[DRY RUN] {direction:10} | PWM: {pwm:5} | Full: {command}")
        return True
    
    # Send to Arduino
    if motor_serial is None or not motor_serial.is_open:
        print(f"[MOTOR Error] No serial connection. Command would be: {direction} (PWM: {pwm})")
        return False
    
    try:
        motor_serial.write(f"{command}\n".encode('utf-8'))
        print(f"[MOTOR] {direction:10} | PWM: {pwm:5} | Full: {command}")
        return True
    except Exception as e:
        print(f"[MOTOR Error] Failed to send command: {e}")
        return False

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
    """
    global yaw_integrated, yaw_fused
    
    # Integrate gyro (dead reckoning)
    yaw_integrated += gyro_rate * dt
    
    # Estimate heading from accelerometer
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
def control_loop():
    """Main control loop - processes web commands and sensor data"""
    global yaw_integrated, yaw_last_time, yaw_fused, control_state
    
    import os
    os.makedirs(LOG_DIR, exist_ok=True)
    
    with open(LOG_FILE, 'w', newline='') as f:
        writer = csv.writer(f)
        writer.writerow([
            'timestamp', 'latitude', 'longitude', 'gps_heading',
            'gyro_rate', 'accel_pitch', 'accel_roll', 'yaw_fused',
            'yaw_error', 'motor_command', 'mode'
        ])
        
        print(f"[LOG] Writing to {LOG_FILE}")
        
        loop_start = time.time()
        cycle = 0
        
        try:
            while True:
                cycle_start = time.time()
                current_time = cycle_start
                
                # Check for new commands from web interface
                motor_command = None  # Don't default to stop - only send real commands
                
                if not motor_queue.empty():
                    motor_command = motor_queue.get()
                    # Debug: log what we actually received
                    print(f"[QUEUE] Received: {repr(motor_command)} (type: {type(motor_command).__name__})")
                
                with state_lock:
                    mode = control_state['mode']
                    enabled = control_state['enabled']
                    dry_run = control_state['dry_run']
                
                # Validate command before sending
                if motor_command:
                    # Only send valid motor commands (M0, M1, T0, T1, S, R)
                    if isinstance(motor_command, str) and motor_command and motor_command[0] in ['M', 'T', 'S', 'R']:
                        if enabled:
                            send_motor_command(motor_command, dry_run=dry_run)
                        elif motor_command.startswith('S'):
                            # Allow stop commands even when disarmed
                            send_motor_command(motor_command, dry_run=dry_run)
                    else:
                        print(f"[QUEUE] Invalid command ignored: {repr(motor_command)}")
                
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
                
                # Calculate yaw error for reference (not used in manual mode)
                yaw_error = desired_heading - actual_heading
                while yaw_error > 180:
                    yaw_error -= 360
                while yaw_error < -180:
                    yaw_error += 360
                
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
                    last_cmd,
                    mode
                ])
                f.flush()
                
                # Maintain loop frequency
                cycle_time = time.time() - cycle_start
                sleep_time = (1.0 / LOOP_FREQ) - cycle_time
                if sleep_time > 0:
                    time.sleep(sleep_time)
                
                cycle += 1
                if cycle % 10 == 0:
                    print(f"[LOOP] Cycle {cycle} | Mode: {mode} | "
                          f"Heading: {actual_heading:.1f}° | "
                          f"Cmd: {last_cmd}")
        
        except KeyboardInterrupt:
            print("\n[SHUTDOWN] Stopping...")
            send_motor_command('S00000')

# ============================================================================
# Web Routes
# ============================================================================

@app.route('/')
def index():
    """Serve main control page"""
    return render_template('control.html')

@app.route('/api/status')
def get_status():
    """Get current system status"""
    global control_state
    
    with gps_lock:
        lat = gps_data['latitude']
        lon = gps_data['longitude']
        gps_heading = gps_data['heading']
    
    with accel_lock:
        pitch = accel_data['pitch']
        roll = accel_data['roll']
        yaw_rate = accel_data['yaw']
    
    with motor_lock:
        last_cmd = motor_state['last_command']
    
    with state_lock:
        mode = control_state['mode']
        speed = control_state['speed']
        enabled = control_state['enabled']
        dry_run = control_state['dry_run']
    
    return jsonify({
        'gps': {
            'latitude': round(lat, 8),
            'longitude': round(lon, 8),
            'heading': round(gps_heading, 1)
        },
        'imu': {
            'pitch': round(pitch, 1),
            'roll': round(roll, 1),
            'yaw_rate': round(yaw_rate, 1)
        },
        'heading': round(yaw_fused, 1),
        'motor_command': last_cmd,
        'mode': mode,
        'speed': speed,
        'enabled': enabled,
        'dry_run': dry_run,
        'enable_reversal': ENABLE_REVERSAL
    })

@app.route('/api/command', methods=['POST'])
def send_command():
    """Handle motor commands from web interface"""
    global control_state
    
    data = request.json
    action = data.get('action')
    speed = data.get('speed', 50)
    
    # Validate action
    if not action:
        return jsonify({'status': 'error', 'message': 'Missing action parameter'}), 400
    
    # Clamp speed to valid range (slider is 10-100%)
    speed = max(10, min(100, int(speed)))
    
    with state_lock:
        control_state['speed'] = speed
    
    # Map speed (10-100%) to PWM range (1100-2000)
    # At 10%: 1100 PWM (tested minimum), at 100%: 2000 PWM (maximum)
    pwm = int(CORRECTION_PWM_MIN + (speed - 10) / 90.0 * (CORRECTION_PWM_MAX - CORRECTION_PWM_MIN))
    pwm = max(CORRECTION_PWM_MIN, min(CORRECTION_PWM_MAX, pwm))
    
    command = None
    
    if action == 'forward':
        command = f'M0{pwm}'
    elif action == 'backward' and ENABLE_REVERSAL:
        command = f'M1{pwm}'
    elif action == 'left':
        command = f'T0{pwm}'
    elif action == 'right':
        command = f'T1{pwm}'
    elif action == 'stop':
        command = 'S00000'
    
    if command:
        motor_queue.put(command)
        return jsonify({'status': 'ok', 'command': command})
    
    return jsonify({'status': 'error', 'message': 'Unknown action'}), 400

@app.route('/api/arm', methods=['POST'])
def arm_system():
    """Arm/disarm the system"""
    global control_state
    
    data = request.json
    armed = data.get('armed', False)
    
    with state_lock:
        is_dry_run = control_state['dry_run']
        control_state['enabled'] = armed
    
    if armed:
        # Send reset command to Arduino
        send_motor_command('R000', dry_run=is_dry_run)
        status_msg = "[DRY RUN] Armed" if is_dry_run else "[SYSTEM] Armed - reset command sent to Arduino"
        print(status_msg)
        return jsonify({'status': 'ok', 'message': 'System armed', 'dry_run': is_dry_run})
    else:
        # Send stop command
        motor_queue.put('S00000')
        print("[SYSTEM] Disarmed")
        return jsonify({'status': 'ok', 'message': 'System disarmed'})

@app.route('/api/mode', methods=['POST'])
def set_mode():
    """Set control mode (MANUAL/AUTONOMOUS)"""
    global control_state
    
    data = request.json
    mode = data.get('mode', 'MANUAL')
    
    if mode not in ['MANUAL', 'AUTONOMOUS']:
        return jsonify({'status': 'error', 'message': 'Invalid mode'}), 400
    
    with state_lock:
        control_state['mode'] = mode
    
    print(f"[MODE] Switched to {mode}")
    return jsonify({'status': 'ok', 'mode': mode})

@app.route('/api/dry-run', methods=['POST'])
def set_dry_run():
    """Toggle dry-run mode (simulates commands without sending to serial)"""
    global control_state
    
    data = request.json
    enabled = data.get('enabled', False)
    
    with state_lock:
        control_state['dry_run'] = enabled
    
    status = "ENABLED" if enabled else "DISABLED"
    print(f"[DRY RUN] {status} - Commands will NOT be sent to Arduino")
    return jsonify({'status': 'ok', 'dry_run': enabled, 'message': f"Dry run {status}"})

# ============================================================================
# Video Feed (MJPEG Camera Stream)
# ============================================================================
def generate_frames():
    """Generate MJPEG frames from camera for web streaming"""
    global camera, camera_frame
    
    if not CAMERA_AVAILABLE or camera is None:
        return
    
    while True:
        try:
            with camera_lock:
                if camera_frame is None:
                    continue
                
                # Encode frame as JPEG
                ret, buffer = cv2.imencode('.jpg', camera_frame)
                if not ret:
                    continue
                
                frame_bytes = buffer.tobytes()
                
            # Yield as MJPEG stream
            yield (b'--frame\r\n'
                   b'Content-Type: image/jpeg\r\n'
                   b'Content-Length: ' + str(len(frame_bytes)).encode() + b'\r\n\r\n'
                   + frame_bytes + b'\r\n')
            
            time.sleep(0.033)  # ~30 FPS
            
        except Exception as e:
            print(f"[CAMERA] Frame generation error: {e}")
            time.sleep(0.1)

@app.route('/video_feed')
def video_feed():
    """Stream camera feed as MJPEG"""
    if not CAMERA_AVAILABLE or camera is None:
        return "Camera not available", 503
    
    return (__import__('flask').Response(generate_frames(),
                                         mimetype='multipart/x-mixed-replace; boundary=frame'),)

def camera_reader_thread():
    """Background thread that continuously reads frames from camera"""
    global camera, camera_frame
    
    if not CAMERA_AVAILABLE:
        print("[CAMERA] OpenCV not available - camera disabled")
        return
    
    try:
        print("[CAMERA] Initializing camera (device 0)...")
        camera = cv2.VideoCapture(0)  # Try /dev/video0
        
        if not camera.isOpened():
            print("[CAMERA] Failed to open camera device 0")
            camera = None
            return
        
        # Set camera properties
        camera.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        camera.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        camera.set(cv2.CAP_PROP_FPS, 30)
        
        print("[CAMERA] Camera initialized (640x480 @ 30 FPS)")
        
        while True:
            ret, frame = camera.read()
            if ret:
                with camera_lock:
                    camera_frame = frame
            else:
                print("[CAMERA] Failed to read frame")
                time.sleep(0.1)
                
    except Exception as e:
        print(f"[CAMERA] Error: {e}")
    finally:
        if camera:
            camera.release()
            print("[CAMERA] Camera released")

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
        
        print("[MOTOR] Ready (arm via web interface to start)")
        return True
    
    except Exception as e:
        print(f"[ERROR] Failed to connect to motor controller: {e}")
        return False

# ============================================================================
# Startup
# ============================================================================
if __name__ == "__main__":
    print("="*60)
    print("Boat Service - Unified Web Control + Autonomous Platform")
    print("="*60)
    print()
    
    # Connect to motor controller
    if not setup_motor_connection():
        print("[FATAL] Cannot start without motor connection")
        exit(1)
    
    # Start background threads
    gps_thread = Thread(target=read_gps_thread, daemon=True)
    accel_thread = Thread(target=read_accel_thread, daemon=True)
    camera_thread = Thread(target=camera_reader_thread, daemon=True)
    control_thread = Thread(target=control_loop, daemon=True)
    
    gps_thread.start()
    accel_thread.start()
    camera_thread.start()
    control_thread.start()
    
    # Give threads time to initialize
    time.sleep(2)
    
    print()
    print("="*60)
    print("🌊 Web Interface Ready!")
    print("="*60)
    print(f"Access at: http://<jetson-ip>:5000")
    print("Default on local network: http://localhost:5000")
    print()
    print("💡 TIP: Toggle 'Dry Run' in web interface to test commands")
    print("   without sending to Arduino (perfect for pre-flight checks)")
    print()
    print("Press Ctrl+C to stop")
    print("="*60)
    print()
    
    # Start Flask web server (blocking)
    app.run(host='0.0.0.0', port=5000, debug=False, use_reloader=False)
