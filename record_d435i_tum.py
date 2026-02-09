#!/usr/bin/env python3
"""
Record RGB-D data from Intel RealSense D435i in TUM format for ORB_SLAM3
Usage: python3 record_d435i_tum.py output_directory [duration_seconds]
"""

import sys
import os
import time
from pathlib import Path
import numpy as np
import cv2
import pyrealsense2 as rs

def record_d435i(output_dir, duration=30):
    """Record RGB-D data from D435i in TUM format"""
    
    output_path = Path(output_dir)
    output_path.mkdir(parents=True, exist_ok=True)
    
    # Create subdirectories
    rgb_dir = output_path / "rgb"
    depth_dir = output_path / "depth"
    rgb_dir.mkdir(exist_ok=True)
    depth_dir.mkdir(exist_ok=True)
    
    print("="*60)
    print("D435i RGB-D Recorder (TUM Format)")
    print("="*60)
    print(f"Output directory: {output_path}")
    print(f"Duration: {duration} seconds")
    print(f"Press Ctrl+C to stop early")
    print("="*60)
    
    # Configure RealSense pipeline
    pipeline = rs.pipeline()
    config = rs.config()
    
    # Configure streams (640x480 @ 30fps)
    config.enable_stream(rs.stream.color, 640, 480, rs.format.rgb8, 30)
    config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
    
    # Start pipeline with retry logic for macOS USB issues
    print("\nStarting camera...")
    max_retries = 3
    for attempt in range(max_retries):
        try:
            profile = pipeline.start(config)
            break
        except RuntimeError as e:
            if attempt < max_retries - 1:
                print(f"  Retry {attempt + 1}/{max_retries}: {e}")
                time.sleep(2)
                # Try creating a new pipeline
                pipeline = rs.pipeline()
                pipeline.start(config)
            else:
                print(f"\n❌ Failed to start camera after {max_retries} attempts")
                print(f"Error: {e}")
                print("\nTroubleshooting:")
                print("  1. Unplug and replug the D435i USB cable")
                print("  2. Make sure no other app is using the camera")
                print("  3. Try: sudo killall VDCAssistant")
                print("  4. Restart your Mac if the issue persists")
                raise
    
    # Get camera intrinsics
    color_stream = profile.get_stream(rs.stream.color)
    depth_stream = profile.get_stream(rs.stream.depth)
    color_intrinsics = color_stream.as_video_stream_profile().get_intrinsics()
    depth_intrinsics = depth_stream.as_video_stream_profile().get_intrinsics()
    
    print(f"\nCamera intrinsics:")
    print(f"  Resolution: {color_intrinsics.width}x{color_intrinsics.height}")
    print(f"  fx: {color_intrinsics.fx:.2f}")
    print(f"  fy: {color_intrinsics.fy:.2f}")
    print(f"  cx: {color_intrinsics.ppx:.2f}")
    print(f"  cy: {color_intrinsics.ppy:.2f}")
    
    # Save calibration
    with open(output_path / "camera_intrinsics.txt", "w") as f:
        f.write(f"# Camera intrinsics (OpenCV format)\n")
        f.write(f"# fx fy cx cy k1 k2 p1 p2 k3\n")
        f.write(f"{color_intrinsics.fx} {color_intrinsics.fy} ")
        f.write(f"{color_intrinsics.ppx} {color_intrinsics.ppy} ")
        # Distortion coefficients
        coeffs = color_intrinsics.coeffs
        f.write(f"{coeffs[0]} {coeffs[1]} {coeffs[2]} {coeffs[3]} {coeffs[4]}\n")
    
    # Align depth to color
    align = rs.align(rs.stream.color)
    
    # Recording variables
    rgb_timestamps = []
    depth_timestamps = []
    associations = []
    frame_count = 0
    start_time = time.time()
    
    print(f"\nRecording started! ({duration}s)")
    print("Frame: ", end="", flush=True)
    
    try:
        while (time.time() - start_time) < duration:
            # Wait for frames
            frames = pipeline.wait_for_frames()
            
            # Align depth to color
            aligned_frames = align.process(frames)
            
            # Get aligned frames
            color_frame = aligned_frames.get_color_frame()
            depth_frame = aligned_frames.get_depth_frame()
            
            if not color_frame or not depth_frame:
                continue
            
            # Get timestamps (in seconds)
            timestamp = time.time() - start_time
            
            # Convert to numpy arrays
            color_image = np.asanyarray(color_frame.get_data())
            depth_image = np.asanyarray(depth_frame.get_data())
            
            # Convert RGB to BGR for OpenCV
            color_image_bgr = cv2.cvtColor(color_image, cv2.COLOR_RGB2BGR)
            
            # Generate filenames with timestamp
            rgb_filename = f"{timestamp:.6f}.png"
            depth_filename = f"{timestamp:.6f}.png"
            
            # Save images
            cv2.imwrite(str(rgb_dir / rgb_filename), color_image_bgr)
            cv2.imwrite(str(depth_dir / depth_filename), depth_image)
            
            # Record timestamps
            rgb_timestamps.append((timestamp, f"rgb/{rgb_filename}"))
            depth_timestamps.append((timestamp, f"depth/{depth_filename}"))
            associations.append((timestamp, f"rgb/{rgb_filename}", 
                               timestamp, f"depth/{depth_filename}"))
            
            frame_count += 1
            if frame_count % 10 == 0:
                print(f"{frame_count}...", end="", flush=True)
            
            # Small delay to maintain ~30 fps
            time.sleep(0.001)
            
    except KeyboardInterrupt:
        print("\n\nRecording stopped by user")
    
    finally:
        pipeline.stop()
        
    elapsed = time.time() - start_time
    fps = frame_count / elapsed if elapsed > 0 else 0
    
    print(f"\n\n{'='*60}")
    print(f"Recording complete!")
    print(f"{'='*60}")
    print(f"Frames captured: {frame_count}")
    print(f"Duration: {elapsed:.2f} seconds")
    print(f"Average FPS: {fps:.2f}")
    print(f"{'='*60}")
    
    # Save timestamp files
    print("\nSaving timestamp files...")
    with open(output_path / "rgb.txt", "w") as f:
        f.write("# timestamp filename\n")
        for ts, filename in rgb_timestamps:
            f.write(f"{ts:.6f} {filename}\n")
    
    with open(output_path / "depth.txt", "w") as f:
        f.write("# timestamp filename\n")
        for ts, filename in depth_timestamps:
            f.write(f"{ts:.6f} {filename}\n")
    
    # Save association file
    print("Saving association file...")
    with open(output_path / "associations.txt", "w") as f:
        f.write("# rgb_timestamp rgb_filename depth_timestamp depth_filename\n")
        for rgb_ts, rgb_file, depth_ts, depth_file in associations:
            f.write(f"{rgb_ts:.6f} {rgb_file} {depth_ts:.6f} {depth_file}\n")
    
    # Save groundtruth placeholder
    with open(output_path / "groundtruth.txt", "w") as f:
        f.write("# No ground truth available for this recording\n")
        f.write("# timestamp tx ty tz qx qy qz qw\n")
    
    # Create ORB_SLAM3 config file
    print("Creating ORB_SLAM3 config file...")
    config_yaml = output_path / "camera_config.yaml"
    with open(config_yaml, "w") as f:
        f.write("%YAML:1.0\n\n")
        f.write("#--------------------------------------------------------------------------------------------\n")
        f.write("# Camera Parameters\n")
        f.write("#--------------------------------------------------------------------------------------------\n")
        f.write('File.version: "1.0"\n\n')
        f.write('Camera.type: "PinHole"\n\n')
        f.write("# Camera calibration parameters (OpenCV)\n")
        f.write(f"Camera1.fx: {color_intrinsics.fx}\n")
        f.write(f"Camera1.fy: {color_intrinsics.fy}\n")
        f.write(f"Camera1.cx: {color_intrinsics.ppx}\n")
        f.write(f"Camera1.cy: {color_intrinsics.ppy}\n\n")
        f.write("# Distortion parameters\n")
        f.write(f"Camera1.k1: {color_intrinsics.coeffs[0]}\n")
        f.write(f"Camera1.k2: {color_intrinsics.coeffs[1]}\n")
        f.write(f"Camera1.p1: {color_intrinsics.coeffs[2]}\n")
        f.write(f"Camera1.p2: {color_intrinsics.coeffs[3]}\n")
        f.write(f"Camera1.k3: {color_intrinsics.coeffs[4]}\n\n")
        f.write("# Camera resolution\n")
        f.write(f"Camera.width: {color_intrinsics.width}\n")
        f.write(f"Camera.height: {color_intrinsics.height}\n\n")
        f.write("# Camera frames per second\n")
        f.write("Camera.fps: 30\n\n")
        f.write("# Color order (0: BGR, 1: RGB)\n")
        f.write("Camera.RGB: 1\n\n")
        f.write("# Depth map factor (converts to meters)\n")
        f.write("RGBD.DepthMapFactor: 1000.0\n\n")
        f.write("# Close/Far threshold\n")
        f.write("Stereo.ThDepth: 40.0\n\n")
        f.write("#--------------------------------------------------------------------------------------------\n")
        f.write("# ORB Parameters\n")
        f.write("#--------------------------------------------------------------------------------------------\n")
        f.write("ORBextractor.nFeatures: 1000\n")
        f.write("ORBextractor.scaleFactor: 1.2\n")
        f.write("ORBextractor.nLevels: 8\n")
        f.write("ORBextractor.iniThFAST: 20\n")
        f.write("ORBextractor.minThFAST: 7\n\n")
        f.write("#--------------------------------------------------------------------------------------------\n")
        f.write("# Viewer Parameters\n")
        f.write("#--------------------------------------------------------------------------------------------\n")
        f.write("Viewer.KeyFrameSize: 0.05\n")
        f.write("Viewer.KeyFrameLineWidth: 1.0\n")
        f.write("Viewer.GraphLineWidth: 0.9\n")
        f.write("Viewer.PointSize: 2.0\n")
        f.write("Viewer.CameraSize: 0.08\n")
        f.write("Viewer.CameraLineWidth: 3.0\n")
        f.write("Viewer.ViewpointX: 0.0\n")
        f.write("Viewer.ViewpointY: -0.7\n")
        f.write("Viewer.ViewpointZ: -1.8\n")
        f.write("Viewer.ViewpointF: 500.0\n")
    
    print(f"\n{'='*60}")
    print("Files created:")
    print(f"  - {frame_count} RGB images in rgb/")
    print(f"  - {frame_count} depth images in depth/")
    print(f"  - rgb.txt (RGB timestamps)")
    print(f"  - depth.txt (Depth timestamps)")
    print(f"  - associations.txt (RGB-Depth associations)")
    print(f"  - camera_config.yaml (ORB_SLAM3 config)")
    print(f"  - camera_intrinsics.txt (Calibration data)")
    print(f"{'='*60}")
    
    print(f"\n✅ Dataset ready for ORB_SLAM3!")
    print(f"\nTo process with ORB_SLAM3, run:")
    print(f"  cd /Users/hendrik/ORB_SLAM3")
    print(f"  ./Examples/RGB-D/rgbd_tum \\")
    print(f"      Vocabulary/ORBvoc.txt \\")
    print(f"      {output_path}/camera_config.yaml \\")
    print(f"      {output_path} \\")
    print(f"      {output_path}/associations.txt")
    print()

if __name__ == "__main__":
    if len(sys.argv) < 2:
        print("Usage: python3 record_d435i_tum.py <output_directory> [duration_seconds]")
        print("\nExample:")
        print("  python3 record_d435i_tum.py ~/d435i_recordings/test1 30")
        print("\nThis will record 30 seconds of RGB-D data from your D435i")
        sys.exit(1)
    
    output_dir = sys.argv[1]
    duration = int(sys.argv[2]) if len(sys.argv) > 2 else 30
    
    try:
        record_d435i(output_dir, duration)
    except Exception as e:
        print(f"\n❌ Error: {e}")
        import traceback
        traceback.print_exc()
        sys.exit(1)

