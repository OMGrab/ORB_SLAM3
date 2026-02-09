#!/usr/bin/env python3
"""
Interactive RealSense D435i recorder with preview
Press SPACEBAR to start, Q to stop
"""

import pyrealsense2 as rs
import cv2
import numpy as np
import os
import time
from datetime import datetime

def wait_for_start():
    """Wait for user to press spacebar to start recording"""
    print("\n🎥 RealSense Recording Setup")
    print("=" * 40)
    print("📹 Camera preview is starting...")
    print("🎯 Controls:")
    print("   SPACEBAR = Start recording")
    print("   Q = Quit without recording")
    print("\n⏳ Press SPACEBAR when ready to start recording...")

def display_preview(pipeline):
    """Display camera preview until user decides to start recording"""
    recording_started = False
    
    try:
        while not recording_started:
            # Wait for frames
            frames = pipeline.wait_for_frames(timeout_ms=1000)
            color_frame = frames.get_color_frame()
            
            if not color_frame:
                continue
                
            # Convert to numpy array
            color_image = np.asanyarray(color_frame.get_data())
            
            # Resize to smaller display size (50% of original)
            height, width = color_image.shape[:2]
            display_width = int(width * 0.5)
            display_height = int(height * 0.5)
            color_image_resized = cv2.resize(color_image, (display_width, display_height))
            
            # Add text overlay (scaled for smaller image)
            overlay_text = "Press SPACEBAR to start recording | Press Q to quit"
            cv2.putText(color_image_resized, overlay_text, (10, 25), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
            
            # Display the frame
            cv2.imshow('RealSense Camera', color_image_resized)
            
            # Check for key presses
            key = cv2.waitKey(1) & 0xFF
            if key == ord(' '):  # Spacebar to start
                recording_started = True
                print("🚀 Starting recording...")
            elif key == ord('q'):  # Q to quit
                print("❌ Recording cancelled by user")
                return False
                
    except Exception as e:
        print(f"❌ Error in preview: {e}")
        return False
    
    return True

def record_with_display(pipeline, bag_filename):
    """Record bag file while displaying live video with controls"""
    frame_count = 0
    start_time = time.time()
    recording = True
    
    print(f"✅ Recording started to: {bag_filename}")
    print("🎯 Controls during recording:")
    print("   Q = Stop recording")
    print("   ESC = Stop recording")
    
    try:
        while recording:
            # Wait for frames
            frames = pipeline.wait_for_frames(timeout_ms=1000)
            depth_frame = frames.get_depth_frame()
            color_frame = frames.get_color_frame()
            
            if not depth_frame or not color_frame:
                print("⚠️  Incomplete frame set, skipping...")
                continue
                
            frame_count += 1
            
            # Convert color frame for display
            color_image = np.asanyarray(color_frame.get_data())
            
            # Resize to smaller display size (50% of original)
            height, width = color_image.shape[:2]
            display_width = int(width * 0.5)
            display_height = int(height * 0.5)
            color_image_resized = cv2.resize(color_image, (display_width, display_height))
            
            # Add recording overlay
            elapsed_time = time.time() - start_time
            fps = frame_count / elapsed_time if elapsed_time > 0 else 0
            
            # Create status text
            status_text = f"RECORDING | Time: {elapsed_time:.1f}s | Frames: {frame_count} | FPS: {fps:.1f}"
            controls_text = "Press Q or ESC to stop recording"
            
            # Add red recording indicator (scaled for smaller image)
            cv2.circle(color_image_resized, (15, 30), 5, (0, 0, 255), -1)  # Red dot
            cv2.putText(color_image_resized, "REC", (25, 35), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 0, 255), 1)
            
            # Add status text (scaled for smaller image)
            cv2.putText(color_image_resized, status_text, (5, 15), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.3, (0, 255, 0), 1)
            cv2.putText(color_image_resized, controls_text, (5, display_height - 10), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.3, (255, 255, 255), 1)
            
            # Display the frame
            cv2.imshow('RealSense Camera', color_image_resized)
            
            # Check for key presses
            key = cv2.waitKey(1) & 0xFF
            if key == ord('q') or key == 27:  # Q or ESC to stop
                recording = False
                print("\n🛑 Stopping recording...")
                break
            
            # Print status every 30 frames (roughly every 2 seconds at 15fps)
            if frame_count % 30 == 0:
                print(f"📹 Recorded {frame_count} frame sets | {fps:.1f} fps | {elapsed_time:.1f}s elapsed")
                
    except KeyboardInterrupt:
        print("\n🛑 Recording interrupted by Ctrl+C...")
        recording = False
    except Exception as e:
        print(f"\n❌ Error during recording: {e}")
        recording = False
    
    return frame_count, time.time() - start_time

def main():
    import sys
    
    # Get output directory from command line if provided
    if len(sys.argv) > 1:
        output_dir = sys.argv[1]
        os.makedirs(output_dir, exist_ok=True)
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        bag_filename = os.path.join(output_dir, f"recording_{timestamp}.bag")
    else:
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        bag_filename = f"realsense_recording_{timestamp}.bag"
    
    # Create a pipeline
    pipeline = rs.pipeline()
    
    # Create a config and configure the pipeline to stream
    config = rs.config()
    config.enable_stream(rs.stream.depth, 1280, 720, rs.format.z16, 15)
    config.enable_stream(rs.stream.color, 1920, 1080, rs.format.bgr8, 15)
    
    try:
        # Start streaming for preview (without recording)
        print("🔧 Initializing RealSense camera...")
        pipeline.start(config)
        print("✅ Camera initialized successfully!")
        
        # Wait for user input to start recording
        wait_for_start()
        
        # Display preview and wait for start signal
        if not display_preview(pipeline):
            return
        
        # Stop the preview pipeline
        pipeline.stop()
        
        # Reconfigure for recording
        config.enable_record_to_file(bag_filename)
        
        print(f"\n📁 Recording to: {bag_filename}")
        print(f"📊 Streams: Depth (1280x720@15fps) + Color (1920x1080@15fps)")
        
        # Start streaming with recording
        pipeline.start(config)
        
        # Record with live display
        frame_count, elapsed_time = record_with_display(pipeline, bag_filename)
        
    except Exception as e:
        print(f"❌ Error: {e}")
        
    finally:
        # Cleanup
        try:
            pipeline.stop()
            cv2.destroyAllWindows()
        except:
            pass
    
    # Report results
    if os.path.exists(bag_filename):
        file_size = os.path.getsize(bag_filename)
        file_size_mb = file_size / (1024 * 1024)
        
        print(f"\n✅ Recording completed successfully!")
        print("=" * 50)
        print(f"📁 File: {bag_filename}")
        print(f"📏 Size: {file_size_mb:.2f} MB ({file_size:,} bytes)")
        print(f"🎬 Duration: {elapsed_time:.1f} seconds")
        print(f"🖼️  Total frames: {frame_count}")
        print(f"📊 Average FPS: {frame_count / elapsed_time:.1f}")
        print(f"💾 Full path: {os.path.abspath(bag_filename)}")
        print("\n🎯 Next steps:")
        print(f"   python3 bag_to_tum.py {bag_filename}")
        print("   To convert to TUM format and process with ORB_SLAM3")
    else:
        print(f"❌ Error: Bag file {bag_filename} was not created!")

if __name__ == "__main__":
    main()

