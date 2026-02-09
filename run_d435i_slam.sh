#!/bin/bash
# Complete D435i SLAM Pipeline: Record → Process → Visualize
# Usage: ./run_d435i_slam.sh <recording_name> [duration]

set -e

if [ $# -lt 1 ]; then
    echo "Usage: ./run_d435i_slam.sh <recording_name> [duration_seconds]"
    echo ""
    echo "Example:"
    echo "  ./run_d435i_slam.sh my_room 30"
    echo ""
    echo "This will:"
    echo "  1. Record 30 seconds from D435i"
    echo "  2. Process with ORB_SLAM3"
    echo "  3. Generate trajectory visualizations"
    echo "  4. Create video with keypoints"
    exit 1
fi

RECORDING_NAME=$1
DURATION=${2:-30}
RECORDINGS_DIR="$HOME/d435i_recordings"
RECORDING_PATH="$RECORDINGS_DIR/$RECORDING_NAME"
ORBSLAM_DIR="/Users/hendrik/ORB_SLAM3"

cd "$ORBSLAM_DIR"

echo "============================================================"
echo "D435i SLAM Pipeline"
echo "============================================================"
echo "Recording: $RECORDING_NAME"
echo "Duration: $DURATION seconds"
echo "Output: $RECORDING_PATH"
echo "============================================================"
echo ""

# Step 1: Record from D435i
echo "📹 Step 1/4: Recording from D435i..."
echo ""
python3 record_d435i_tum.py "$RECORDING_PATH" "$DURATION"

if [ ! -f "$RECORDING_PATH/associations.txt" ]; then
    echo "❌ Error: Recording failed - no association file found"
    exit 1
fi

echo ""
echo "✅ Recording complete!"
echo ""

# Step 2: Run ORB_SLAM3
echo "============================================================"
echo "🚀 Step 2/4: Running ORB_SLAM3..."
echo "============================================================"
echo ""

./Examples/RGB-D/rgbd_tum \
    Vocabulary/ORBvoc.txt \
    "$RECORDING_PATH/camera_config.yaml" \
    "$RECORDING_PATH" \
    "$RECORDING_PATH/associations.txt"

if [ ! -f "CameraTrajectory.txt" ]; then
    echo "❌ Error: ORB_SLAM3 failed - no trajectory file generated"
    exit 1
fi

# Move trajectory files to recording directory
mv CameraTrajectory.txt "$RECORDING_PATH/"
mv KeyFrameTrajectory.txt "$RECORDING_PATH/"

echo ""
echo "✅ SLAM processing complete!"
echo ""

# Step 3: Visualize trajectory in 3D
echo "============================================================"
echo "📊 Step 3/4: Creating 3D trajectory visualization..."
echo "============================================================"
echo ""

if [ -f "visualize_trajectory.py" ]; then
    python3 visualize_trajectory.py \
        "$RECORDING_PATH/CameraTrajectory.txt" \
        "$RECORDING_PATH/KeyFrameTrajectory.txt" &
    echo "✅ 3D visualization window opened"
else
    echo "⚠️  visualize_trajectory.py not found, skipping 3D plot"
fi

echo ""

# Step 4: Create video with keypoints
echo "============================================================"
echo "🎥 Step 4/4: Creating video with keypoints..."
echo "============================================================"
echo ""

if [ -f "create_video_with_trajectory.py" ]; then
    python3 create_video_with_trajectory.py \
        "$RECORDING_PATH" \
        "$RECORDING_PATH/associations.txt" \
        "$RECORDING_PATH/CameraTrajectory.txt" \
        "$RECORDING_PATH/${RECORDING_NAME}_slam.mp4"
    
    echo ""
    echo "✅ Video created: $RECORDING_PATH/${RECORDING_NAME}_slam.mp4"
else
    echo "⚠️  create_video_with_trajectory.py not found, skipping video"
fi

echo ""
echo "============================================================"
echo "✅ SLAM Pipeline Complete!"
echo "============================================================"
echo ""
echo "Output files in: $RECORDING_PATH"
echo ""
echo "  📸 Images:"
echo "     - rgb/*.png (RGB frames)"
echo "     - depth/*.png (Depth frames)"
echo ""
echo "  📊 Trajectories:"
echo "     - CameraTrajectory.txt"
echo "     - KeyFrameTrajectory.txt"
echo ""
echo "  🎥 Video:"
echo "     - ${RECORDING_NAME}_slam.mp4"
echo ""
echo "  ⚙️  Config:"
echo "     - camera_config.yaml"
echo "     - associations.txt"
echo ""
echo "To view the video:"
echo "  open \"$RECORDING_PATH/${RECORDING_NAME}_slam.mp4\""
echo ""
echo "============================================================"

