#!/usr/bin/env python3
"""
Create a video showing RGB frames with trajectory overlay and detected ORB keypoints
Usage: python3 create_video_with_trajectory.py dataset_path associations.txt CameraTrajectory.txt output.mp4
"""

import sys
import os
import cv2
import numpy as np
from pathlib import Path

def load_trajectory(filename):
    """Load trajectory from TUM format file"""
    data = np.loadtxt(filename)
    timestamps = data[:, 0]
    positions = data[:, 1:4]  # x, y, z
    return timestamps, positions

def load_associations(filename):
    """Load image associations"""
    associations = []
    with open(filename, 'r') as f:
        for line in f:
            if line.strip():
                parts = line.strip().split()
                associations.append({
                    'timestamp': float(parts[0]),
                    'rgb': parts[1],
                    'depth_timestamp': float(parts[2]),
                    'depth': parts[3]
                })
    return associations

def draw_trajectory_minimap(frame, positions, current_idx, map_size=200):
    """Draw a mini-map of the trajectory on the frame"""
    h, w = frame.shape[:2]
    
    # Create mini-map background
    map_x = w - map_size - 20
    map_y = 20
    cv2.rectangle(frame, (map_x, map_y), (map_x + map_size, map_y + map_size), 
                  (0, 0, 0), -1)
    cv2.rectangle(frame, (map_x, map_y), (map_x + map_size, map_y + map_size), 
                  (255, 255, 255), 2)
    
    # Get XY positions and normalize to map
    if len(positions) < 2:
        return frame
    
    xy = positions[:, [0, 1]]  # Use X and Y
    
    # Normalize to map size with padding
    padding = 20
    x_min, x_max = xy[:, 0].min(), xy[:, 0].max()
    y_min, y_max = xy[:, 1].min(), xy[:, 1].max()
    
    x_range = x_max - x_min
    y_range = y_max - y_min
    max_range = max(x_range, y_range)
    
    if max_range == 0:
        return frame
    
    scale = (map_size - 2 * padding) / max_range
    
    # Convert positions to map coordinates
    map_points = []
    for i in range(len(xy)):
        px = int((xy[i, 0] - x_min) * scale + padding + map_x)
        py = int((xy[i, 1] - y_min) * scale + padding + map_y)
        map_points.append((px, py))
    
    # Draw trajectory line
    for i in range(len(map_points) - 1):
        if i < current_idx:
            cv2.line(frame, map_points[i], map_points[i+1], (0, 255, 0), 2)
        else:
            cv2.line(frame, map_points[i], map_points[i+1], (100, 100, 100), 1)
    
    # Draw current position
    if current_idx < len(map_points):
        cv2.circle(frame, map_points[current_idx], 5, (0, 0, 255), -1)
    
    # Draw start position
    cv2.circle(frame, map_points[0], 4, (255, 255, 0), -1)
    
    return frame

def create_video(dataset_path, associations_file, trajectory_file, output_file):
    """Create video with trajectory overlay and ORB keypoints"""
    
    print("Loading trajectory...")
    timestamps, positions = load_trajectory(trajectory_file)
    print(f"Loaded {len(positions)} trajectory poses")
    
    print("Loading associations...")
    associations = load_associations(associations_file)
    print(f"Loaded {len(associations)} frames")
    
    # Initialize ORB detector with ORB-SLAM3-like parameters
    print("Initializing ORB detector...")
    orb = cv2.ORB_create(
        nfeatures=1000,          # Number of features (matches ORB-SLAM3 default)
        scaleFactor=1.2,         # Scale factor between levels
        nlevels=8,               # Number of pyramid levels
        edgeThreshold=19,        # Size of border where features are not detected
        firstLevel=0,            # Level of pyramid to put source image
        WTA_K=2,                 # Number of points to produce oriented BRIEF descriptor
        scoreType=cv2.ORB_HARRIS_SCORE,  # Harris or FAST score
        patchSize=31,            # Size of patch used by oriented BRIEF
        fastThreshold=20         # FAST threshold
    )
    
    # Get first frame to determine video size
    first_img_path = os.path.join(dataset_path, associations[0]['rgb'])
    first_frame = cv2.imread(first_img_path)
    if first_frame is None:
        print(f"Error: Could not load first frame from {first_img_path}")
        return
    
    h, w = first_frame.shape[:2]
    
    # Initialize video writer
    fourcc = cv2.VideoWriter_fourcc(*'mp4v')
    fps = 30.0
    video_writer = cv2.VideoWriter(output_file, fourcc, fps, (w, h))
    
    print(f"Creating video: {output_file} ({w}x{h} @ {fps} fps)")
    
    # Process frames
    for i, assoc in enumerate(associations):
        img_path = os.path.join(dataset_path, assoc['rgb'])
        frame = cv2.imread(img_path)
        
        if frame is None:
            print(f"Warning: Could not load frame {img_path}")
            continue
        
        # Detect ORB keypoints
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        keypoints = orb.detect(gray, None)
        
        # Draw keypoints on the frame
        # Use different colors for different octaves (pyramid levels)
        for kp in keypoints:
            x, y = int(kp.pt[0]), int(kp.pt[1])
            octave = kp.octave & 255  # Get pyramid level
            
            # Color coding by pyramid level
            if octave == 0:
                color = (0, 255, 0)    # Green - finest level
            elif octave == 1:
                color = (0, 255, 255)  # Yellow
            elif octave == 2:
                color = (0, 165, 255)  # Orange
            else:
                color = (0, 0, 255)    # Red - coarser levels
            
            # Draw circle for keypoint
            cv2.circle(frame, (x, y), 3, color, 1)
            
            # Draw orientation line
            angle = kp.angle * np.pi / 180.0
            end_x = int(x + 10 * np.cos(angle))
            end_y = int(y + 10 * np.sin(angle))
            cv2.line(frame, (x, y), (end_x, end_y), color, 1)
        
        # Draw trajectory mini-map
        frame = draw_trajectory_minimap(frame, positions, i)
        
        # Add info panel background
        overlay = frame.copy()
        cv2.rectangle(overlay, (5, 5), (400, 125), (0, 0, 0), -1)
        cv2.addWeighted(overlay, 0.6, frame, 0.4, 0, frame)
        
        # Add frame info
        cv2.putText(frame, f"Frame: {i+1}/{len(associations)}", 
                   (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
        
        # Add number of detected features
        cv2.putText(frame, f"Features: {len(keypoints)}", 
                   (10, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
        
        # Add distance traveled
        if i > 0:
            dist = np.linalg.norm(positions[i] - positions[0])
            cv2.putText(frame, f"Distance: {dist:.2f}m", 
                       (10, 75), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
        
        # Add current position
        pos_text = f"Pos: [{positions[i, 0]:.2f}, {positions[i, 1]:.2f}, {positions[i, 2]:.2f}]"
        cv2.putText(frame, pos_text, 
                   (10, 100), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
        
        # Add legend for keypoint colors
        cv2.putText(frame, "Level 0", (10, h-65), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 255, 0), 1)
        cv2.putText(frame, "Level 1", (10, h-50), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 255, 255), 1)
        cv2.putText(frame, "Level 2", (10, h-35), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 165, 255), 1)
        cv2.putText(frame, "Level 3+", (10, h-20), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 0, 255), 1)
        
        video_writer.write(frame)
        
        if (i + 1) % 50 == 0:
            print(f"Processed {i+1}/{len(associations)} frames")
    
    video_writer.release()
    print(f"\nVideo saved to: {output_file}")
    print(f"Total frames: {len(associations)}")
    print(f"Duration: {len(associations)/fps:.1f} seconds")

if __name__ == '__main__':
    if len(sys.argv) != 5:
        print("Usage: python3 create_video_with_trajectory.py dataset_path associations.txt CameraTrajectory.txt output.mp4")
        print("Example: python3 create_video_with_trajectory.py ~/Downloads/rgbd_dataset_freiburg1_xyz Examples/RGB-D/associations/fr1_xyz.txt CameraTrajectory.txt output.mp4")
        sys.exit(1)
    
    dataset_path = sys.argv[1]
    associations_file = sys.argv[2]
    trajectory_file = sys.argv[3]
    output_file = sys.argv[4]
    
    if not os.path.exists(dataset_path):
        print(f"Error: Dataset path not found: {dataset_path}")
        sys.exit(1)
    
    if not os.path.exists(associations_file):
        print(f"Error: Associations file not found: {associations_file}")
        sys.exit(1)
    
    if not os.path.exists(trajectory_file):
        print(f"Error: Trajectory file not found: {trajectory_file}")
        sys.exit(1)
    
    create_video(dataset_path, associations_file, trajectory_file, output_file)

