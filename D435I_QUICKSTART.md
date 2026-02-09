# Intel RealSense D435i + ORB_SLAM3 on macOS M2
## Quick Start Guide

✅ **Your working D435i SLAM pipeline is ready!**

---

## 🚀 Quick Start (Complete Pipeline)

Record from your D435i and process with ORB_SLAM3 in one command:

```bash
cd /Users/hendrik/ORB_SLAM3
./complete_d435i_slam_pipeline.sh my_room_scan
```

**What it does:**
1. 📹 Opens camera preview (press SPACEBAR to start recording)
2. 🔄 Converts to TUM format
3. 🚀 Runs ORB_SLAM3
4. 📊 Creates 3D visualization
5. 🎥 Generates video with keypoints

**Output:** `~/d435i_recordings/my_room_scan/`
- `recording_*.bag` - Original RealSense recording
- `CameraTrajectory.txt` - Full camera path
- `KeyFrameTrajectory.txt` - Selected keyframes
- `my_room_scan_slam_keypoints.mp4` - Video with ORB features
- `rgb/` & `depth/` - Extracted images

---

## 📹 Step-by-Step Workflow

### 1. Record from D435i (Interactive)

```bash
python3 record_realsense_interactive.py ~/d435i_recordings/test1
```

**Controls:**
- **Preview:** Shows camera feed
- **SPACEBAR:** Start recording
- **Q or ESC:** Stop recording

### 2. Convert to TUM Format

```bash
python3 bag_to_tum.py ~/d435i_recordings/test1/recording_*.bag
```

Creates: `rgb/`, `depth/`, `associations.txt`, `camera_config.yaml`

### 3. Run ORB_SLAM3

```bash
cd /Users/hendrik/ORB_SLAM3
./Examples/RGB-D/rgbd_tum \
    Vocabulary/ORBvoc.txt \
    ~/d435i_recordings/test1/camera_config.yaml \
    ~/d435i_recordings/test1 \
    ~/d435i_recordings/test1/associations.txt
```

### 4. Visualize Results

**3D Trajectory:**
```bash
python3 visualize_trajectory.py \
    ~/d435i_recordings/test1/CameraTrajectory.txt \
    ~/d435i_recordings/test1/KeyFrameTrajectory.txt
```

**Video with Keypoints:**
```bash
python3 create_video_with_trajectory.py \
    ~/d435i_recordings/test1 \
    ~/d435i_recordings/test1/associations.txt \
    ~/d435i_recordings/test1/CameraTrajectory.txt \
    ~/d435i_recordings/test1/output.mp4
```

---

## 🎯 Camera Settings

Current D435i configuration:
- **Color:** 1920×1080 @ 15fps (BGR8)
- **Depth:** 1280×720 @ 15fps (Z16)
- **Display:** 50% scaled preview

To change settings, edit `record_realsense_interactive.py`:
```python
config.enable_stream(rs.stream.depth, WIDTH, HEIGHT, rs.format.z16, FPS)
config.enable_stream(rs.stream.color, WIDTH, HEIGHT, rs.format.bgr8, FPS)
```

---

## 📊 Processing TUM Datasets

Download and process standard SLAM benchmarks:

```bash
cd /Users/hendrik/ORB_SLAM3

# Process TUM RGB-D datasets
./process_tum_dataset.sh freiburg1_desk
./process_tum_dataset.sh freiburg1_room
./process_tum_dataset.sh freiburg2_desk
```

**Available datasets:** See TUM RGB-D website
- freiburg1_* - 640×480 camera
- freiburg2_* - Different camera  
- freiburg3_* - Long sequences

---

## 🔧 Troubleshooting

### Camera Not Detected

```bash
# Check if D435i is connected
python3 -c "import pyrealsense2 as rs; print(f'Devices: {len(rs.context().query_devices())}')"
```

**Solutions:**
1. Unplug/replug USB cable (use USB 3.0 port)
2. Close other apps using camera
3. Restart computer if persistent

### ORB_SLAM3 Crashes

**Cause:** Pangolin viewer threading issue on macOS

**Fix:** Already applied! Viewer disabled in all examples.

### Low Frame Rate

- Check USB connection (use USB 3.0, not USB 2.0)
- Reduce resolution in recording script
- Close other apps

---

## 📁 File Structure

```
~/d435i_recordings/
└── my_recording/
    ├── recording_20241105_123456.bag    # Original
    ├── rgb/                              # RGB images
    │   └── *.png
    ├── depth/                            # Depth maps
    │   └── *.png
    ├── associations.txt                  # RGB-Depth pairs
    ├── camera_config.yaml                # ORB_SLAM3 config
    ├── CameraTrajectory.txt              # All poses
    ├── KeyFrameTrajectory.txt            # Keyframes only
    └── my_recording_slam_keypoints.mp4   # Video result
```

---

## 🎥 Video Output

The generated video shows:
- ✅ Original RGB frames
- ✅ Detected ORB keypoints (colored by pyramid level)
- ✅ Feature orientation lines
- ✅ Trajectory mini-map (top-right)
- ✅ Frame info & distance traveled
- ✅ Current 3D position

**Color Legend:**
- 🟢 Green = Finest detail (Level 0)
- 🟡 Yellow = Level 1
- 🟠 Orange = Level 2  
- 🔴 Red = Coarse levels (3+)

---

## ✅ macOS M2 Compatibility

**What's Working:**
- ✅ D435i recording via Python (interactive script)
- ✅ ORB_SLAM3 processing (headless mode)
- ✅ TUM dataset processing
- ✅ Trajectory visualization
- ✅ Video generation with keypoints

**Known Issues (Fixed):**
- ❌ C++ librealsense crashes → ✅ Using Python
- ❌ Pangolin viewer crashes → ✅ Disabled viewer
- ❌ chrono timing issues → ✅ Fixed to steady_clock

---

## 📚 Additional Resources

**Scripts:**
- `record_realsense_interactive.py` - Interactive D435i recorder
- `bag_to_tum.py` - Convert .bag to TUM format
- `complete_d435i_slam_pipeline.sh` - One-command pipeline
- `visualize_trajectory.py` - 3D trajectory plotter
- `create_video_with_trajectory.py` - Video with keypoints
- `process_tum_dataset.sh` - Download & process TUM datasets

**Documentation:**
- TUM RGB-D Dataset: https://vision.in.tum.de/data/datasets/rgbd-dataset
- ORB_SLAM3 Paper: https://arxiv.org/abs/2007.11898
- RealSense SDK: https://github.com/IntelRealSense/librealsense

---

## 🎯 Next Steps

1. **Test the pipeline:**
   ```bash
   ./complete_d435i_slam_pipeline.sh test_recording
   ```

2. **Record your environment:**
   - Move camera smoothly
   - Include textured surfaces
   - Avoid motion blur
   - Good lighting helps

3. **Analyze results:**
   - Check trajectory smoothness
   - Verify loop closures
   - Compare with ground truth (if available)

4. **Experiment:**
   - Try different ORB_SLAM3 parameters
   - Adjust camera settings
   - Process multiple recordings

---

## 💡 Tips for Best Results

**Recording:**
- ✅ Smooth, slow camera movements
- ✅ Textured environment (books, posters, patterns)
- ✅ Good lighting (avoid backlighting)
- ✅ Overlap in camera views
- ❌ Avoid plain walls
- ❌ Avoid rapid movements
- ❌ Avoid reflective surfaces

**Processing:**
- More features = better tracking (increase ORBextractor.nFeatures)
- Lower threshold = more features (decrease minThFAST)
- Depth factor must match camera (1000.0 for D435i)

---

**🎉 You're all set! Happy SLAM-ing!**

