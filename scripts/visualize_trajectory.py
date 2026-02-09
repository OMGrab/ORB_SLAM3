#!/usr/bin/env python3
"""
Visualize ORB_SLAM3 trajectory in 3D
Usage: python3 visualize_trajectory.py CameraTrajectory.txt
"""

import sys
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

def load_trajectory(filename):
    """Load trajectory from TUM format file"""
    data = np.loadtxt(filename)
    timestamps = data[:, 0]
    positions = data[:, 1:4]  # x, y, z
    quaternions = data[:, 4:8]  # qx, qy, qz, qw
    return timestamps, positions, quaternions

def plot_trajectory_3d(positions, keyframe_positions=None):
    """Plot 3D trajectory"""
    fig = plt.figure(figsize=(12, 8))
    ax = fig.add_subplot(111, projection='3d')
    
    # Plot full trajectory
    ax.plot(positions[:, 0], positions[:, 1], positions[:, 2], 
            'b-', linewidth=1, alpha=0.6, label='Camera Trajectory')
    
    # Plot start point
    ax.scatter(positions[0, 0], positions[0, 1], positions[0, 2], 
               c='g', marker='o', s=100, label='Start')
    
    # Plot end point
    ax.scatter(positions[-1, 0], positions[-1, 1], positions[-1, 2], 
               c='r', marker='o', s=100, label='End')
    
    # Plot keyframes if provided
    if keyframe_positions is not None:
        ax.scatter(keyframe_positions[:, 0], keyframe_positions[:, 1], 
                   keyframe_positions[:, 2], c='orange', marker='^', 
                   s=50, alpha=0.7, label='Keyframes')
    
    ax.set_xlabel('X (m)')
    ax.set_ylabel('Y (m)')
    ax.set_zlabel('Z (m)')
    ax.set_title('ORB-SLAM3 Camera Trajectory')
    ax.legend()
    
    # Equal aspect ratio
    max_range = np.array([positions[:, 0].max()-positions[:, 0].min(),
                          positions[:, 1].max()-positions[:, 1].min(),
                          positions[:, 2].max()-positions[:, 2].min()]).max() / 2.0
    
    mid_x = (positions[:, 0].max()+positions[:, 0].min()) * 0.5
    mid_y = (positions[:, 1].max()+positions[:, 1].min()) * 0.5
    mid_z = (positions[:, 2].max()+positions[:, 2].min()) * 0.5
    ax.set_xlim(mid_x - max_range, mid_x + max_range)
    ax.set_ylim(mid_y - max_range, mid_y + max_range)
    ax.set_zlim(mid_z - max_range, mid_z + max_range)
    
    plt.tight_layout()
    return fig

def plot_trajectory_2d(positions):
    """Plot 2D trajectory views"""
    fig, axes = plt.subplots(2, 2, figsize=(12, 10))
    
    # XY view (top-down)
    axes[0, 0].plot(positions[:, 0], positions[:, 1], 'b-', linewidth=1)
    axes[0, 0].scatter(positions[0, 0], positions[0, 1], c='g', s=100, label='Start')
    axes[0, 0].scatter(positions[-1, 0], positions[-1, 1], c='r', s=100, label='End')
    axes[0, 0].set_xlabel('X (m)')
    axes[0, 0].set_ylabel('Y (m)')
    axes[0, 0].set_title('Top View (XY)')
    axes[0, 0].legend()
    axes[0, 0].grid(True)
    axes[0, 0].axis('equal')
    
    # XZ view (side)
    axes[0, 1].plot(positions[:, 0], positions[:, 2], 'b-', linewidth=1)
    axes[0, 1].scatter(positions[0, 0], positions[0, 2], c='g', s=100, label='Start')
    axes[0, 1].scatter(positions[-1, 0], positions[-1, 2], c='r', s=100, label='End')
    axes[0, 1].set_xlabel('X (m)')
    axes[0, 1].set_ylabel('Z (m)')
    axes[0, 1].set_title('Side View (XZ)')
    axes[0, 1].legend()
    axes[0, 1].grid(True)
    axes[0, 1].axis('equal')
    
    # YZ view (front)
    axes[1, 0].plot(positions[:, 1], positions[:, 2], 'b-', linewidth=1)
    axes[1, 0].scatter(positions[0, 1], positions[0, 2], c='g', s=100, label='Start')
    axes[1, 0].scatter(positions[-1, 1], positions[-1, 2], c='r', s=100, label='End')
    axes[1, 0].set_xlabel('Y (m)')
    axes[1, 0].set_ylabel('Z (m)')
    axes[1, 0].set_title('Front View (YZ)')
    axes[1, 0].legend()
    axes[1, 0].grid(True)
    axes[1, 0].axis('equal')
    
    # Distance traveled over time
    distances = np.sqrt(np.sum(np.diff(positions, axis=0)**2, axis=1))
    cumulative_distance = np.concatenate([[0], np.cumsum(distances)])
    axes[1, 1].plot(cumulative_distance, 'b-', linewidth=2)
    axes[1, 1].set_xlabel('Frame')
    axes[1, 1].set_ylabel('Distance Traveled (m)')
    axes[1, 1].set_title('Cumulative Distance')
    axes[1, 1].grid(True)
    
    plt.tight_layout()
    return fig

if __name__ == '__main__':
    if len(sys.argv) < 2:
        print("Usage: python3 visualize_trajectory.py CameraTrajectory.txt [KeyFrameTrajectory.txt]")
        sys.exit(1)
    
    # Load camera trajectory
    print(f"Loading trajectory from {sys.argv[1]}...")
    timestamps, positions, quaternions = load_trajectory(sys.argv[1])
    print(f"Loaded {len(positions)} poses")
    
    # Load keyframe trajectory if provided
    keyframe_positions = None
    if len(sys.argv) >= 3:
        print(f"Loading keyframes from {sys.argv[2]}...")
        _, keyframe_positions, _ = load_trajectory(sys.argv[2])
        print(f"Loaded {len(keyframe_positions)} keyframes")
    
    # Plot 3D trajectory
    print("Plotting 3D trajectory...")
    fig3d = plot_trajectory_3d(positions, keyframe_positions)
    
    # Plot 2D views
    print("Plotting 2D views...")
    fig2d = plot_trajectory_2d(positions)
    
    # Print statistics
    total_distance = np.sum(np.sqrt(np.sum(np.diff(positions, axis=0)**2, axis=1)))
    print(f"\nTrajectory Statistics:")
    print(f"  Total frames: {len(positions)}")
    print(f"  Total distance: {total_distance:.2f} m")
    print(f"  Start position: [{positions[0, 0]:.3f}, {positions[0, 1]:.3f}, {positions[0, 2]:.3f}]")
    print(f"  End position: [{positions[-1, 0]:.3f}, {positions[-1, 1]:.3f}, {positions[-1, 2]:.3f}]")
    if keyframe_positions is not None:
        print(f"  Keyframes: {len(keyframe_positions)}")
    
    plt.show()

