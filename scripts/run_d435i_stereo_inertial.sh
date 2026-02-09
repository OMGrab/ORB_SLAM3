#!/bin/bash
# Run D435i stereo-inertial SLAM (no viewer)

set -e

cd /Users/hendrik/ORB_SLAM3

VOCAB_FILE="./Vocabulary/ORBvoc.txt"
SETTINGS_FILE="./Examples/Stereo-Inertial/RealSense_D435i.yaml"
OUTPUT_NAME="${1:-d435i_test}"

echo "============================================================"
echo "D435i Stereo-Inertial SLAM (No Viewer)"
echo "============================================================"
echo "Vocabulary: $VOCAB_FILE"
echo "Settings:   $SETTINGS_FILE"
echo "Output:     $OUTPUT_NAME"
echo ""
echo "Press Ctrl+C to stop recording..."
echo "============================================================"
echo ""

# Run with sudo for camera access
sudo ./Examples/Stereo-Inertial/stereo_inertial_realsense_D435i \
    "$VOCAB_FILE" \
    "$SETTINGS_FILE" \
    "$OUTPUT_NAME"

echo ""
echo "============================================================"
echo "SLAM Complete!"
echo "============================================================"
echo ""
echo "Output files:"
ls -lh CameraTrajectory.txt KeyFrameTrajectory.txt 2>/dev/null || echo "  No trajectory files found"
echo ""

