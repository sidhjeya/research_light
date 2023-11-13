#!/usr/bin/env python

import rospy
import cv2
import numpy as np
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2
import pyrealsense2 as rs

def get_pointcloud(depth_frame, color_frame, depth_intrinsics, color_intrinsics, depth_to_color_extrinsics):
    # Create a point cloud object
    pc = rs.pointcloud()

    # Map color to depth to get corresponding pixels
    depth_frame = depth_frame.as_depth_frame()
    points = pc.calculate(depth_frame)

    # Convert RealSense points to numpy array
    vtx = np.asanyarray(points.get_vertices())
    tex = np.asanyarray(points.get_texture_coordinates())

    # Filter out invalid points
    valid_idx = np.where(vtx[:, 2] > 0)
    vtx = vtx[valid_idx]
    tex = tex[valid_idx]

    # Create a PointCloud2 message
    pc_msg = pc2.create_cloud_xyz32(header=rospy.Header(), 
                                    cloud_points=np.column_stack([vtx[:, 0], vtx[:, 1], vtx[:, 2]]))

    return pc_msg

def main():
    rospy.init_node('realsense_pointcloud_publisher')

    # Configure RealSense pipeline
    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
    config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

    # Start RealSense pipeline
    pipeline.start(config)

    # Create ROS publisher for PointCloud2
    pointcloud_pub = rospy.Publisher('/camera/depth/color/points', PointCloud2, queue_size=1)

    rate = rospy.Rate(30)  # Hz

    try:
        while not rospy.is_shutdown():
            # Wait for a coherent pair of frames: depth and color
            frames = pipeline.wait_for_frames()
            depth_frame = frames.get_depth_frame()
            color_frame = frames.get_color_frame()

            if not depth_frame or not color_frame:
                continue

            # Get camera parameters
            depth_intrinsics = depth_frame.profile.as_video_stream_profile().intrinsics
            color_intrinsics = color_frame.profile.as_video_stream_profile().intrinsics
            depth_to_color_extrinsics = depth_frame.profile.get_extrinsics_to(color_frame.profile)

            # Get PointCloud2 message
            pc_msg = get_pointcloud(depth_frame, color_frame, depth_intrinsics, color_intrinsics, depth_to_color_extrinsics)

            # Publish PointCloud2 message
            pointcloud_pub.publish(pc_msg)

            rate.sleep()

    finally:
        pipeline.stop()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
