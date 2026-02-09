#!/usr/bin/env python3
"""
Convert RealSense .bag file to TUM RGB-D dataset format
Usage: python3 bag_to_tum.py <recording.bag>
"""

import sys
import os
from pathlib import Path
import numpy as np
import cv2
import pyrealsense2 as rs

def bag_to_tum(bag_file, max_frames=None):
    """Convert RealSense .bag to TUM format"""
    
    bag_path = Path(bag_file)
    if not bag_path.exists():
        print(f"❌ Error: {bag_file} not found")
        return False
    
    output_dir = bag_path.parent
    rgb_dir = output_dir / "rgb"
    depth_dir = output_dir / "depth"
    rgb_dir.mkdir(exist_ok=True)
    depth_dir.mkdir(exist_ok=True)
    
    print("="*60)
    print("Converting .bag to TUM format")
    print("="*60)
    print(f"Input: {bag_file}")
    print(f"Output: {output_dir}")
    if max_frames:
        print(f"Frame limit: {max_frames}")
    print("="*60)
    
    # Create pipeline for playback
    pipeline = rs.pipeline()
    config = rs.config()
    
    # Enable playback from bag file
    rs.config.enable_device_from_file(config, str(bag_file))
    
    # Start pipeline
    try:
        profile = pipeline.start(config)
    except RuntimeError as e:
        print(f"❌ Failed to read bag file: {e}")
        return False
    
    # Get device and disable real-time playback for faster processing
    device = profile.get_device()
    playback = device.as_playback()
    playback.set_real_time(False)
    
    # Get camera intrinsics from the first frame
    frames = pipeline.wait_for_frames()
    color_frame = frames.get_color_frame()
    depth_frame = frames.get_depth_frame()
    
    if not color_frame:
        print("❌ No color frames found in bag file")
        pipeline.stop()
        return False
    
    # Get intrinsics
    color_profile = color_frame.get_profile().as_video_stream_profile()
    intrinsics = color_profile.get_intrinsics()
    
    print(f"\nCamera intrinsics:")
    print(f"  Resolution: {intrinsics.width}x{intrinsics.height}")
    print(f"  fx: {intrinsics.fx:.2f}")
    print(f"  fy: {intrinsics.fy:.2f}")
    print(f"  cx: {intrinsics.ppx:.2f}")
    print(f"  cy: {intrinsics.ppy:.2f}")
    
    # Restart pipeline to process all frames
    pipeline.stop()
    pipeline = rs.pipeline()
    config = rs.config()
    rs.config.enable_device_from_file(config, str(bag_file))
    profile = pipeline.start(config)
    device = profile.get_device()
    playback = device.as_playback()
    playback.set_real_time(False)
    
    # Align depth to color
    align = rs.align(rs.stream.color)
    
    rgb_timestamps = []
    depth_timestamps = []
    associations = []
    frame_count = 0
    
    print(f"\nProcessing frames...")
    print("Frame: ", end="", flush=True)
    
    try:
        while True:
            frames = pipeline.wait_for_frames()
            
            # Align depth to color
            aligned_frames = align.process(frames)
            
            color_frame = aligned_frames.get_color_frame()
            depth_frame = aligned_frames.get_depth_frame()
            
            if not color_frame or not depth_frame:
                continue
            
            # Get timestamp (in seconds from start)
            timestamp = frames.get_timestamp() / 1000.0  # Convert ms to seconds
            
            # Convert to numpy
            color_image = np.asanyarray(color_frame.get_data())
            depth_image = np.asanyarray(depth_frame.get_data())
            
            # Convert RGB to BGR for OpenCV
            color_image_bgr = cv2.cvtColor(color_image, cv2.COLOR_RGB2BGR)
            
            # Generate filenames
            rgb_filename = f"{timestamp:.6f}.png"
            depth_filename = f"{timestamp:.6f}.png"
            
            # Save images
            cv2.imwrite(str(rgb_dir / rgb_filename), color_image_bgr)
            cv2.imwrite(str(depth_dir / depth_filename), depth_image)
            
            # Record associations
            rgb_timestamps.append((timestamp, f"rgb/{rgb_filename}"))
            depth_timestamps.append((timestamp, f"depth/{depth_filename}"))
            associations.append((timestamp, f"rgb/{rgb_filename}",
                               timestamp, f"depth/{depth_filename}"))
            
            frame_count += 1
            if frame_count % 30 == 0:
                print(f"{frame_count}...", end="", flush=True)
            
            # Check frame limit
            if max_frames and frame_count >= max_frames:
                print(f"\n✅ Reached frame limit ({max_frames})")
                break
            
    except RuntimeError:
        # End of file reached
        pass
    finally:
        pipeline.stop()
    
    print(f"\n\n{'='*60}")
    print(f"✅ Conversion complete!")
    print(f"{'='*60}")
    print(f"Frames extracted: {frame_count}")
    print(f"{'='*60}")
    
    # Save timestamp files
    print("\nSaving TUM format files...")
    
    with open(output_dir / "rgb.txt", "w") as f:
        f.write("# timestamp filename\n")
        for ts, filename in rgb_timestamps:
            f.write(f"{ts:.6f} {filename}\n")
    
    with open(output_dir / "depth.txt", "w") as f:
        f.write("# timestamp filename\n")
        for ts, filename in depth_timestamps:
            f.write(f"{ts:.6f} {filename}\n")
    
    with open(output_dir / "associations.txt", "w") as f:
        f.write("# rgb_timestamp rgb_filename depth_timestamp depth_filename\n")
        for rgb_ts, rgb_file, depth_ts, depth_file in associations:
            f.write(f"{rgb_ts:.6f} {rgb_file} {depth_ts:.6f} {depth_file}\n")
    
    # Save camera config for ORB_SLAM3
    config_file = output_dir / "camera_config.yaml"
    with open(config_file, "w") as f:
        f.write("%YAML:1.0\n\n")
        f.write("File.version: \"1.0\"\n\n")
        f.write("Camera.type: \"PinHole\"\n\n")
        f.write(f"Camera1.fx: {intrinsics.fx}\n")
        f.write(f"Camera1.fy: {intrinsics.fy}\n")
        f.write(f"Camera1.cx: {intrinsics.ppx}\n")
        f.write(f"Camera1.cy: {intrinsics.ppy}\n\n")
        f.write(f"Camera1.k1: {intrinsics.coeffs[0]}\n")
        f.write(f"Camera1.k2: {intrinsics.coeffs[1]}\n")
        f.write(f"Camera1.p1: {intrinsics.coeffs[2]}\n")
        f.write(f"Camera1.p2: {intrinsics.coeffs[3]}\n")
        f.write(f"Camera1.k3: {intrinsics.coeffs[4]}\n\n")
        f.write(f"Camera.width: {intrinsics.width}\n")
        f.write(f"Camera.height: {intrinsics.height}\n")
        f.write("Camera.fps: 30\n")
        f.write("Camera.RGB: 1\n\n")
        f.write("RGBD.DepthMapFactor: 1000.0\n\n")
        f.write("ORBextractor.nFeatures: 1000\n")
        f.write("ORBextractor.scaleFactor: 1.2\n")
        f.write("ORBextractor.nLevels: 8\n")
        f.write("ORBextractor.iniThFAST: 20\n")
        f.write("ORBextractor.minThFAST: 7\n\n")
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
    print(f"  - {frame_count} RGB images (rgb/)")
    print(f"  - {frame_count} depth images (depth/)")
    print("  - rgb.txt")
    print("  - depth.txt")
    print("  - associations.txt")
    print("  - camera_config.yaml")
    print(f"{'='*60}")
    
    print(f"\n✅ Ready for ORB_SLAM3!")
    print(f"\nRun:")
    print(f"  cd /Users/hendrik/ORB_SLAM3")
    print(f"  ./Examples/RGB-D/rgbd_tum \\")
    print(f"      Vocabulary/ORBvoc.txt \\")
    print(f"      {output_dir}/camera_config.yaml \\")
    print(f"      {output_dir} \\")
    print(f"      {output_dir}/associations.txt\n")
    
    return True

if __name__ == "__main__":
    if len(sys.argv) < 2 or len(sys.argv) > 3:
        print("Usage: python3 bag_to_tum.py <recording.bag> [max_frames]")
        print("\nExamples:")
        print("  python3 bag_to_tum.py ~/d435i_recordings/my_room/recording.bag")
        print("  python3 bag_to_tum.py ~/d435i_recordings/my_room/recording.bag 30")
        sys.exit(1)
    
    bag_file = sys.argv[1]
    max_frames = int(sys.argv[2]) if len(sys.argv) == 3 else None
    success = bag_to_tum(bag_file, max_frames)
    sys.exit(0 if success else 1)

