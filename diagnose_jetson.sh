#!/bin/bash
# Diagnostic script for Jetson Nano SIMD/Illegal Instruction issues

echo "=== Jetson Nano Diagnostic Report ==="
echo ""
echo "1. CPU Info:"
cat /proc/cpuinfo | grep -i "processor\|model\|flags" | head -10
echo ""

echo "2. Jetson Info:"
if [ -f /etc/nv_tegra_release ]; then
    cat /etc/nv_tegra_release
else
    echo "Not a Jetson device"
fi
echo ""

echo "3. GStreamer Version:"
gst-launch-1.0 --version 2>/dev/null || echo "GStreamer not found"
echo ""

echo "4. Available GStreamer Plugins (video):"
gst-inspect-1.0 nvarguscamerasrc 2>/dev/null || echo "nvarguscamerasrc not available"
gst-inspect-1.0 jpegenc 2>/dev/null | head -5
gst-inspect-1.0 nvjpegenc 2>/dev/null || echo "nvjpegenc not available"
echo ""

echo "5. Test Simple GStreamer Pipeline (CPU-only):"
echo "   Running: gst-launch-1.0 videotestsrc ! jpegenc ! fakesink"
timeout 3 gst-launch-1.0 videotestsrc ! jpegenc ! fakesink 2>&1 | tail -5 || echo "   ^ Check if this fails with Illegal Instruction"
echo ""

echo "6. Attempting Python import test (with NEON disabled):"
CFLAGS="-mfpu=neon" python3 -c "import gi; gi.require_version('Gst', '1.0'); from gi.repository import Gst; print('GStreamer import: OK')" 2>&1 || echo "Failed"
echo ""

echo "7. Environment Variables to Try:"
echo "   export DISABLE_NEON=1"
echo "   export GST_PLUGIN_PATH=/usr/lib/aarch64-linux-gnu/gstreamer-1.0"
echo "   export GStreamer_SKIP_NEON=1"
