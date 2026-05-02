#!/bin/bash
# Launch boat_service.py with NEON optimizations disabled
# This fixes "Illegal Instruction" crashes on Jetson Nano 2GB

echo "[LAUNCHER] Starting boat_service with NEON disabled..."
echo "[LAUNCHER] If you still get 'Illegal Instruction', try:"
echo "           sudo apt remove libgstreamer1.0-0 && sudo apt install libgstreamer1.0-0"
echo ""

# Disable ARM NEON optimizations that cause Illegal Instruction on some Jetson models
export CFLAGS="-march=armv8-a -mfpu=neon"
export CXXFLAGS="-march=armv8-a -mfpu=neon"
export OPENBLAS_CORETYPE=CORTEX
export ARMV7_NEON_DISABLE=1
export OMP_NUM_THREADS=4

# Force CPU-only GStreamer plugins if available
export GST_PLUGIN_SKIP_LOADING=1
export GST_PLUGIN_PATH=/usr/lib/aarch64-linux-gnu/gstreamer-1.0

# Run the service
python3 boat_service.py "$@"
