#!/bin/bash
# Script to download and process TUM RGB-D datasets with ORB_SLAM3
# Usage: ./process_tum_dataset.sh <dataset_name>
# Example: ./process_tum_dataset.sh freiburg1_xyz

if [ $# -eq 0 ]; then
    echo "Usage: $0 <dataset_name>"
    echo ""
    echo "Available datasets:"
    echo "  freiburg1_xyz       - Simple XYZ motion (427MB) - already downloaded"
    echo "  freiburg1_desk      - Desk sequence (328MB) - already downloaded"
    echo "  freiburg1_desk2     - Desk sequence 2 (647MB)"
    echo "  freiburg1_room      - Room sequence (1.3GB)"
    echo "  freiburg2_xyz       - XYZ with different camera (491MB)"
    echo "  freiburg2_desk      - Desk with different camera (1.2GB)"
    echo "  freiburg3_long_office - Long office trajectory (2.4GB)"
    echo ""
    exit 1
fi

DATASET=$1
DOWNLOAD_DIR=~/Downloads
ORBSLAM_DIR=/Users/hendrik/ORB_SLAM3

# Dataset URLs and associations
case $DATASET in
    freiburg1_xyz)
        URL="https://cvg.cit.tum.de/rgbd/dataset/freiburg1/rgbd_dataset_freiburg1_xyz.tgz"
        ASSOC="fr1_xyz.txt"
        ;;
    freiburg1_desk)
        URL="https://cvg.cit.tum.de/rgbd/dataset/freiburg1/rgbd_dataset_freiburg1_desk.tgz"
        ASSOC="fr1_desk.txt"
        ;;
    freiburg1_desk2)
        URL="https://cvg.cit.tum.de/rgbd/dataset/freiburg1/rgbd_dataset_freiburg1_desk2.tgz"
        ASSOC="fr1_desk2.txt"
        ;;
    freiburg1_room)
        URL="https://cvg.cit.tum.de/rgbd/dataset/freiburg1/rgbd_dataset_freiburg1_room.tgz"
        ASSOC="fr1_room.txt"
        ;;
    freiburg2_xyz)
        URL="https://cvg.cit.tum.de/rgbd/dataset/freiburg2/rgbd_dataset_freiburg2_xyz.tgz"
        ASSOC="fr2_xyz.txt"
        ;;
    freiburg2_desk)
        URL="https://cvg.cit.tum.de/rgbd/dataset/freiburg2/rgbd_dataset_freiburg2_desk.tgz"
        ASSOC="fr2_desk.txt"
        ;;
    freiburg3_long_office)
        URL="https://cvg.cit.tum.de/rgbd/dataset/freiburg3/rgbd_dataset_freiburg3_long_office_household.tgz"
        ASSOC="fr3_long_office.txt"
        ;;
    *)
        echo "Unknown dataset: $DATASET"
        exit 1
        ;;
esac

DATASET_DIR="${DOWNLOAD_DIR}/rgbd_dataset_${DATASET}"

# Check if dataset already exists
if [ -d "$DATASET_DIR" ]; then
    echo "Dataset already exists at $DATASET_DIR"
else
    echo "Downloading ${DATASET}..."
    cd $DOWNLOAD_DIR
    curl -L -o "rgbd_dataset_${DATASET}.tgz" "$URL"
    
    echo "Extracting..."
    tar -xzf "rgbd_dataset_${DATASET}.tgz"
fi

# Run ORB-SLAM3
echo ""
echo "Running ORB-SLAM3..."
cd $ORBSLAM_DIR

# Choose the correct settings file based on freiburg version
if [[ $DATASET == freiburg1* ]]; then
    SETTINGS="Examples/RGB-D/TUM1.yaml"
elif [[ $DATASET == freiburg2* ]]; then
    SETTINGS="Examples/RGB-D/TUM2.yaml"
else
    SETTINGS="Examples/RGB-D/TUM3.yaml"
fi

./Examples/RGB-D/rgbd_tum \
    Vocabulary/ORBvoc.txt \
    $SETTINGS \
    $DATASET_DIR \
    Examples/RGB-D/associations/$ASSOC

# Create visualizations
echo ""
echo "Creating visualizations..."

# Copy trajectory files to dataset-specific names
cp CameraTrajectory.txt "CameraTrajectory_${DATASET}.txt"
cp KeyFrameTrajectory.txt "KeyFrameTrajectory_${DATASET}.txt"

# Create video
python3 create_video_with_trajectory.py \
    $DATASET_DIR \
    Examples/RGB-D/associations/$ASSOC \
    CameraTrajectory.txt \
    "${DATASET}_output.mp4"

# Create 3D plot
python3 visualize_trajectory.py \
    CameraTrajectory.txt \
    KeyFrameTrajectory.txt

echo ""
echo "================================================"
echo "Processing complete!"
echo "================================================"
echo "Trajectory files:"
echo "  - CameraTrajectory_${DATASET}.txt"
echo "  - KeyFrameTrajectory_${DATASET}.txt"
echo "Video:"
echo "  - ${DATASET}_output.mp4"
echo ""
echo "To view the video:"
echo "  open ${ORBSLAM_DIR}/${DATASET}_output.mp4"

