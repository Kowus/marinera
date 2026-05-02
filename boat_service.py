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
import numpy as np

# Try importing GStreamer for CSI camera - graceful fallback if not available
GSTREAMER_AVAILABLE = False
try:
    import gi
    gi.require_version('Gst', '1.0')
    from gi.repository import Gst, GLib
    GSTREAMER_AVAILABLE = True
except (ImportError, OSError, SystemError) as e:
    # OSError/SystemError catches "Illegal instruction" crashes during import
    print(f"[WARNING] GStreamer not available ({type(e).__name__}: {e})")
    print("[WARNING] Camera feed will be disabled")
    print("[TIP] To fix 'Illegal Instruction' on Jetson Nano 2GB:")
    print("      1. Run: bash run_boat.sh (disables NEON)")
    print("      2. Or: sudo apt-get install --reinstall libgstreamer1.0-0")
    print("      3. Or: Use USB camera instead of CSI (fallback method)")
    GLib = None
    Gst = None

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
CAMERA_PREVIEW = False       # Enable preview window on connected Jetson display (disabled due to pipeline format issues)
JPEG_QUALITY = 40            # JPEG encoding quality (1-100): lower = faster/more latency, higher = better quality
CAMERA_WIDTH = 640           # Force low resolution to reduce CPU load (1280x720 JPEG encoding too slow on Nano 2GB)
CAMERA_HEIGHT = 480
CAMERA_FPS = 30              # Keep at 30fps for stable encoding
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

def send_vortex_command(state, dry_run=False):
    """Send vortex on/off command to Arduino (state: True=ON, False=OFF)"""
    global motor_serial
    
    command = 'V100000' if state else 'V000000'
    status = 'ON' if state else 'OFF'
    
    if dry_run:
        print(f"[DRY RUN] Vortex: {status:3} | Full: {command}")
        return True
    
    if motor_serial is None or not motor_serial.is_open:
        print(f"[VORTEX Error] No serial connection. Command would be: {status}")
        return False
    
    try:
        motor_serial.write(f"{command}\n".encode('utf-8'))
        print(f"[VORTEX] {status:3} | Full: {command}")
        return True
    except Exception as e:
        print(f"[VORTEX Error] Failed to send command: {e}")
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
        # Enable vortex when arming
        send_vortex_command(True, dry_run=is_dry_run)
        status_msg = "[DRY RUN] Armed with vortex enabled" if is_dry_run else "[SYSTEM] Armed - reset command sent to Arduino, vortex enabled"
        print(status_msg)
        return jsonify({'status': 'ok', 'message': 'System armed', 'dry_run': is_dry_run})
    else:
        # Send stop command
        motor_queue.put('S00000')
        # Disable vortex when disarming
        send_vortex_command(False, dry_run=is_dry_run)
        print("[SYSTEM] Disarmed - vortex disabled")
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
# PLACEHOLDER IMAGE - for when camera is unavailable
# ============================================================================
def generate_placeholder_frame():
    """Generate a simple placeholder JPEG when camera is not available"""
    import base64
    # Minimal 1x1 gray JPEG (base64 encoded)
    placeholder_jpeg = base64.b64encode(b'\xff\xd8\xff\xe0\x00\x10JFIF\x00\x01\x01\x00\x00\x01\x00\x01\x00\x00\xff\xdb\x00C\x00\x08\x06\x06\x07\x06\x05\x08\x07\x07\x07\t\t\x08\n\x0c\x14\r\x0c\x0b\x0b\x0c\x19\x12\x13\x0f\x14\x1d\x1a\x1f\x1e\x1d\x1a\x1c\x1c $.\' ",#\x1c\x1c(7),01444\x1f\'9=82<.342\xff\xc0\x00\x0b\x08\x00\x01\x00\x01\x01\x11\x00\xff\xc4\x00\x1f\x00\x00\x01\x05\x01\x01\x01\x01\x01\x01\x00\x00\x00\x00\x00\x00\x00\x00\x01\x02\x03\x04\x05\x06\x07\x08\t\n\x0b\xff\xda\x00\x08\x01\x01\x00\x00?\x00\xfb\xda\xff\xd9').decode()
    return base64.b64decode(placeholder_jpeg)

def generate_frames():
    """Generate MJPEG frames from camera for web streaming (send immediately, no artificial delay)"""
    global camera_frame
    
    last_frame_id = None  # Track last frame sent to skip duplicates
    
    while True:
        try:
            with camera_lock:
                if camera_frame is None:
                    # Use placeholder when no camera
                    frame_bytes = generate_placeholder_frame()
                else:
                    # Camera_frame is already JPEG-encoded by GStreamer
                    frame_bytes = camera_frame
            
            # Skip if it's the same frame object (no new data from GStreamer)
            frame_id = id(frame_bytes)
            if frame_id == last_frame_id:
                time.sleep(0.001)  # 1ms sleep to prevent busy-spinning, let other threads run
                continue
            
            last_frame_id = frame_id
            
            # Send frame immediately - no artificial delay
            yield (b'--frame\r\n'
                   b'Content-Type: image/jpeg\r\n'
                   b'Content-Length: ' + str(len(frame_bytes)).encode() + b'\r\n\r\n'
                   + frame_bytes + b'\r\n')
            
        except Exception as e:
            print(f"[CAMERA] Frame generation error: {e}")
            time.sleep(0.01)  # Minimal sleep on error only

@app.route('/video_feed')
def video_feed():
    """Stream camera feed as MJPEG"""
    return __import__('flask').Response(generate_frames(),
                                        mimetype='multipart/x-mixed-replace; boundary=frame')

def camera_reader_thread():
    """Background thread that captures CSI camera and encodes to JPEG using GStreamer"""
    global camera, camera_frame
    
    if not GSTREAMER_AVAILABLE:
        print("[CAMERA] GStreamer not available - camera disabled")
        print("         Install with: sudo apt install python3-gi python3-gi-cairo gir1.2-gstreamer-1.0")
        return
    
    try:
        print("[CAMERA] Initializing GStreamer for CSI camera (CAM1)...")
        Gst.init(None)
        
        # GStreamer pipeline for Jetson Nano CSI camera with JPEG encoding
        # nvarguscamerasrc: CSI camera source → GPU memory (NV12)
        # nvvidconv: GPU YUV→BGR conversion, stays in GPU memory
        # videoconvert: Convert from GPU memory (NVMM) back to system memory
        # jpegenc: Software JPEG encoding (GPU->CPU transfer is the bottleneck)
        # appsink: Capture encoded JPEG bytes for web streaming
        
        pipeline_str = (
            'nvarguscamerasrc ! '
            f'video/x-raw(memory:NVMM), width={CAMERA_WIDTH}, height={CAMERA_HEIGHT}, framerate={CAMERA_FPS}/1 ! '
            'nvvidconv ! '
            'video/x-raw, format=BGRx ! '
            'videoconvert ! '
            'video/x-raw, format=I420 ! '
            f'jpegenc quality={JPEG_QUALITY} ! '
            'appsink emit-signals=true max-buffers=1 drop=true sync=false name=sink'
        )
        
        print(f"[CAMERA] Pipeline: {CAMERA_WIDTH}x{CAMERA_HEIGHT} @ {CAMERA_FPS}fps, JPEG quality={JPEG_QUALITY}")
        
        camera = Gst.parse_launch(pipeline_str)
        if not camera:
            print("[CAMERA] Failed to create GStreamer pipeline")
            return
        
        appsink = camera.get_by_name('sink')
        if not appsink:
            print("[CAMERA] Could not find appsink in pipeline")
            return
        
        def on_new_sample(sink):
            """Callback when new JPEG frame is available from GStreamer"""
            global camera_frame
            sample = sink.emit('pull-sample')
            if sample:
                buf = sample.get_buffer()
                
                # Extract JPEG data directly
                result, mapinfo = buf.map(Gst.MapFlags.READ)
                if result:
                    jpeg_data = bytes(mapinfo.data)
                    
                    with camera_lock:
                        camera_frame = jpeg_data
                    
                    buf.unmap(mapinfo)
            
            return Gst.FlowReturn.OK
        
        appsink.connect('new-sample', on_new_sample)
        
        print("[CAMERA] Camera initialized (640x480 @ 30 FPS, GPU-encoded JPEG)")
        print("[CAMERA] GStreamer pipeline: nvarguscamerasrc → nvvidconv → nvjpegenc")
        
        # Start the pipeline
        ret = camera.set_state(Gst.State.PLAYING)
        if ret == Gst.StateChangeReturn.FAILURE:
            print("[CAMERA] Failed to start GStreamer pipeline")
            return
        
        print("[CAMERA] Pipeline started successfully")
        
        # Keep thread alive
        while True:
            time.sleep(1)
            
    except (OSError, SystemError) as e:
        # Catch "Illegal Instruction" and other hardware-level errors
        print(f"[CAMERA] HARDWARE ERROR: {type(e).__name__} - {e}")
        print("[CAMERA] Camera initialization failed (likely NEON/SIMD incompatibility)")
        print("[CAMERA] Fallback: Running without camera")
        print("[CAMERA] Try: bash run_boat.sh (disables NEON optimizations)")
    except Exception as e:
        print(f"[CAMERA] Error: {e}")
        import traceback
        traceback.print_exc()
    finally:
        if camera:
            camera.set_state(Gst.State.NULL)
            print("[CAMERA] Camera pipeline stopped")

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
