#!/usr/bin/env python3
"""
Record RGB-D data from Intel RealSense D435i to .bag file
Then convert to TUM format for ORB_SLAM3
Usage: python3 record_d435i_bag.py <output_name> [duration_seconds]
"""

import sys
import time
import pyrealsense2 as rs

def record_realsense_bag(bag_filename, duration=30):
    """
    Records RGB, depth, and IMU data to a .bag file
    """
    # Create pipeline and config
    pipeline = rs.pipeline()
    config = rs.config()
    
    print("="*60)
    print("D435i Recorder (RealSense .bag format)")
    print("="*60)
    print(f"Output: {bag_filename}")
    print(f"Duration: {duration} seconds")
    print("="*60)
    
    # Enable RGB stream
    config.enable_stream(rs.stream.color, 640, 480, rs.format.rgb8, 30)
    
    # Enable depth stream
    config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
    
    # Enable IMU streams (D435i has IMU)
    try:
        config.enable_stream(rs.stream.accel, rs.format.motion_xyz32f, 250)
        config.enable_stream(rs.stream.gyro, rs.format.motion_xyz32f, 400)
        print("✅ IMU streams enabled")
    except Exception as e:
        print(f"⚠️  IMU not available: {e}")
    
    # Enable recording to bag file
    config.enable_record_to_file(bag_filename)
    
    print(f"\n📹 Starting recording...")
    
    # Start streaming
    try:
        pipeline.start(config)
    except RuntimeError as e:
        print(f"\n❌ Failed to start camera: {e}")
        print("\nTroubleshooting:")
        print("  1. Make sure D435i is connected via USB 3.0")
        print("  2. Try unplugging and replugging the camera")
        print("  3. Close any other apps using the camera")
        return False
    
    start_time = time.time()
    frame_count = 0
    
    print(f"Recording... Press Ctrl+C to stop")
    print("Frame: ", end="", flush=True)
    
    try:
        while time.time() - start_time < duration:
            # Wait for frames (this includes all enabled streams)
            frames = pipeline.wait_for_frames()
            frame_count += 1
            
            if frame_count % 30 == 0:
                elapsed = time.time() - start_time
                print(f"{frame_count} ({elapsed:.1f}s)...", end="", flush=True)
                
    except KeyboardInterrupt:
        print("\n\n⏹️  Recording stopped by user")
    except RuntimeError as e:
        print(f"\n❌ Runtime error: {e}")
        return False
    except Exception as e:
        print(f"\n❌ Unexpected error: {e}")
        return False
    finally:
        pipeline.stop()
    
    elapsed = time.time() - start_time
    fps = frame_count / elapsed if elapsed > 0 else 0
    
    print(f"\n\n{'='*60}")
    print("✅ Recording complete!")
    print(f"{'='*60}")
    print(f"Frames captured: {frame_count}")
    print(f"Duration: {elapsed:.2f} seconds")
    print(f"Average FPS: {fps:.2f}")
    print(f"File: {bag_filename}")
    print(f"{'='*60}\n")
    
    return True

if __name__ == "__main__":
    if len(sys.argv) < 2:
        print("Usage: python3 record_d435i_bag.py <output_name> [duration_seconds]")
        print("\nExample:")
        print("  python3 record_d435i_bag.py my_room 30")
        print("\nThis will create: ~/d435i_recordings/my_room/recording.bag")
        sys.exit(1)
    
    recording_name = sys.argv[1]
    duration = int(sys.argv[2]) if len(sys.argv) > 2 else 30
    
    # Create output directory
    import os
    from pathlib import Path
    
    base_dir = Path.home() / "d435i_recordings" / recording_name
    base_dir.mkdir(parents=True, exist_ok=True)
    
    bag_file = str(base_dir / "recording.bag")
    
    success = record_realsense_bag(bag_file, duration)
    
    if success:
        print(f"\n{'='*60}")
        print("Next steps:")
        print(f"{'='*60}")
        print("Convert to TUM format:")
        print(f"  python3 bag_to_tum.py {bag_file}")
        print("\nOr run complete pipeline:")
        print(f"  ./process_d435i_bag.sh {recording_name}")
        print(f"{'='*60}\n")
    else:
        sys.exit(1)

