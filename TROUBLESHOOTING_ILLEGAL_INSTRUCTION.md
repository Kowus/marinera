# Troubleshooting: "Illegal Instruction" on Jetson Nano 2GB

## Problem

The boat_service crashes with `Illegal Instruction (core dumped)` when starting GStreamer on Jetson Nano 2GB.

## Root Cause

Jetson Nano 2GB has ARM processors that don't fully support ARMv8 NEON (SIMD) optimizations used in some pre-compiled system libraries. When GStreamer tries to use these instructions, the kernel crashes with "Illegal Instruction" instead of a catchable exception.

## Solutions (in order of recommendation)

### ✅ RECOMMENDED: Use launch script with NEON disabled

```bash
bash run_boat.sh
```

This sets environment variables to disable ARM NEON optimizations before running Python.

### Try: Reinstall GStreamer system libraries

If the above doesn't work, reinstall with CPU-only support:

```bash
sudo apt-get update
sudo apt-get install --reinstall libgstreamer1.0-0
sudo apt-get install --reinstall gstreamer1.0-plugins-base
sudo apt-get install --reinstall gstreamer1.0-plugins-good
```

### Try: Disable GStreamer GPU plugins

Force CPU-only codec paths:

```bash
export GST_PLUGIN_SKIP_LOADING=1
export GST_PLUGIN_PATH=/usr/lib/aarch64-linux-gnu/gstreamer-1.0
python3 boat_service.py
```

### Fallback: Use USB camera instead of CSI

If CSI camera still fails, use a standard USB webcam:

1. Comment out CSI pipeline in boat_service.py
2. Replace with USB OpenCV camera (will need to reinstall opencv-python carefully)
3. Or use Motion or mjpg-streamer for USB camera

### Last Resort: Compile GStreamer from source

```bash
# Clone GStreamer repo
git clone https://github.com/GStreamer/gstreamer.git
cd gstreamer

# Build without NEON
meson builddir -Doption:arm_neon=disabled
ninja -C builddir
sudo ninja -C builddir install
```

## Diagnostic Tools

### Check CPU capabilities

```bash
cat /proc/cpuinfo | grep -i flags
```

Look for "neon" in flags - if not present, NEON isn't supported.

### Run diagnostics

```bash
bash diagnose_jetson.sh
```

### Test GStreamer manually

```bash
# Simple test (should not crash)
gst-launch-1.0 videotestsrc ! fakesink

# Try nvarguscamerasrc (may crash if incompatible)
gst-launch-1.0 nvarguscamerasrc ! fakesink
```

## System Configuration Check

```bash
# What Jetson model?
cat /etc/nv_tegra_release

# What's installed?
dpkg -l | grep gstreamer

# GStreamer version
gst-launch-1.0 --version
```

## Workaround While Debugging

The system has a built-in fallback:

- If GStreamer fails, camera will be disabled but **everything else works**
- Web dashboard will show a placeholder image
- Motor controls, GPS, IMU all function normally
- You can still test and use autonomous features without the camera

## Next Steps

1. Try `bash run_boat.sh` first
2. If that fails, run `bash diagnose_jetson.sh` and share output
3. If still broken, use USB camera as fallback
4. Consider filing issue with Jetson community if it's a known hardware limitation

## Files Added

- `run_boat.sh` - Launcher with NEON disabled
- `diagnose_jetson.sh` - Diagnostic script
