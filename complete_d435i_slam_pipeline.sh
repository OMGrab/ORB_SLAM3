#!/bin/bash
# Complete D435i SLAM Pipeline with Interactive Recording
# Usage: ./complete_d435i_slam_pipeline.sh <recording_name>

set -e

if [ $# -lt 1 ]; then
    echo "Usage: ./complete_d435i_slam_pipeline.sh <recording_name>"
    echo ""
    echo "Example:"
    echo "  ./complete_d435i_slam_pipeline.sh my_room"
    echo ""
    echo "This will:"
    echo "  1. Interactive recording from D435i (press SPACE to start, Q to stop)"
    echo "  2. Convert .bag to TUM format"
    echo "  3. Process with ORB_SLAM3"
    echo "  4. Generate 3D trajectory visualization"
    echo "  5. Create video with detected keypoints"
    exit 1
fi

RECORDING_NAME=$1
RECORDINGS_DIR="$HOME/d435i_recordings"
RECORDING_PATH="$RECORDINGS_DIR/$RECORDING_NAME"
ORBSLAM_DIR="/Users/hendrik/ORB_SLAM3"

cd "$ORBSLAM_DIR"

echo "============================================================"
echo "🎥 Complete D435i SLAM Pipeline"
echo "============================================================"
echo "Recording: $RECORDING_NAME"
echo "Output directory: $RECORDING_PATH"
echo "============================================================"
echo ""

# Create output directory
mkdir -p "$RECORDING_PATH"

# Step 1: Interactive Recording
echo "============================================================"
echo "📹 Step 1/5: Recording from D435i (Interactive)"
echo "============================================================"
echo ""
echo "🎯 Instructions:"
echo "  1. Camera preview will open"
echo "  2. Press SPACEBAR when ready to start recording"
echo "  3. Press Q or ESC to stop recording"
echo ""

python3 record_realsense_interactive.py "$RECORDING_PATH"

# Find the bag file (it will have a timestamp)
BAG_FILE=$(ls -t "$RECORDING_PATH"/*.bag 2>/dev/null | head -1)

if [ -z "$BAG_FILE" ]; then
    echo "❌ Error: No .bag file found. Recording may have failed."
    exit 1
fi

echo ""
echo "✅ Recording saved: $BAG_FILE"
echo ""

# Step 2: Convert to TUM format
echo "============================================================"
echo "🔄 Step 2/5: Converting .bag to TUM format"
echo "============================================================"
echo ""

python3 bag_to_tum.py "$BAG_FILE"

if [ ! -f "$RECORDING_PATH/associations.txt" ]; then
    echo "❌ Error: Conversion failed - no association file found"
    exit 1
fi

echo ""
echo "✅ Conversion complete!"
echo ""

# Step 3: Run ORB_SLAM3
echo "============================================================"
echo "🚀 Step 3/5: Running ORB_SLAM3"
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

# Step 4: 3D Trajectory Visualization
echo "============================================================"
echo "📊 Step 4/5: Creating 3D trajectory visualization"
echo "============================================================"
echo ""

if [ -f "visualize_trajectory.py" ]; then
    python3 visualize_trajectory.py \
        "$RECORDING_PATH/CameraTrajectory.txt" \
        "$RECORDING_PATH/KeyFrameTrajectory.txt" &
    echo "✅ 3D visualization window opened (background)"
else
    echo "⚠️  visualize_trajectory.py not found, skipping"
fi

echo ""

# Step 5: Create Video with Keypoints
echo "============================================================"
echo "🎥 Step 5/5: Creating video with ORB keypoints"
echo "============================================================"
echo ""

if [ -f "create_video_with_trajectory.py" ]; then
    python3 create_video_with_trajectory.py \
        "$RECORDING_PATH" \
        "$RECORDING_PATH/associations.txt" \
        "$RECORDING_PATH/CameraTrajectory.txt" \
        "$RECORDING_PATH/${RECORDING_NAME}_slam_keypoints.mp4"
    
    echo ""
    echo "✅ Video created!"
else
    echo "⚠️  create_video_with_trajectory.py not found, skipping"
fi

echo ""
echo "============================================================"
echo "✅ Complete Pipeline Finished!"
echo "============================================================"
echo ""
echo "📂 Output directory: $RECORDING_PATH"
echo ""
echo "📁 Files created:"
echo "  🎬 Original:"
echo "     - $(basename "$BAG_FILE") (RealSense bag file)"
echo ""
echo "  📸 Images:"
echo "     - rgb/*.png (RGB frames)"
echo "     - depth/*.png (Depth frames)"
echo ""
echo "  📊 SLAM Results:"
echo "     - CameraTrajectory.txt"
echo "     - KeyFrameTrajectory.txt"
echo ""
echo "  🎥 Video:"
echo "     - ${RECORDING_NAME}_slam_keypoints.mp4"
echo ""
echo "  ⚙️  Config:"
echo "     - camera_config.yaml"
echo "     - associations.txt"
echo ""
echo "🎬 To view the results:"
echo "  open \"$RECORDING_PATH/${RECORDING_NAME}_slam_keypoints.mp4\""
echo ""
echo "============================================================"

