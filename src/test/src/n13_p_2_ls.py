#!/usr/bin/env python

import rospy
from sensor_msgs.msg import PointCloud2, LaserScan
import sensor_msgs.point_cloud2 as pc2
import numpy as np
import cv2
import pyrealsense2 as rs

def pointcloud_to_laserscan(msg):
    # Extract point cloud data
    pc_data = pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)

    # Convert point cloud to numpy array
    pc_np = np.array(list(pc_data))

    # Assuming a simple projection to 2D for simplicity
    # You might need to adjust this based on your camera's configuration
    distances = np.sqrt(pc_np[:, 0]**2 + pc_np[:, 1]**2)
    angles = np.arctan2(pc_np[:, 1], pc_np[:, 0])

    # Create LaserScan message
    laser_scan_msg = LaserScan()
    laser_scan_msg.header = msg.header
    laser_scan_msg.angle_min = np.min(angles)
    laser_scan_msg.angle_max = np.max(angles)
    laser_scan_msg.angle_increment = (laser_scan_msg.angle_max - laser_scan_msg.angle_min) / len(distances)
    laser_scan_msg.time_increment = 0.0
    laser_scan_msg.scan_time = 0.1
    laser_scan_msg.range_min = np.min(distances)
    laser_scan_msg.range_max = np.max(distances)
    laser_scan_msg.ranges = distances.tolist()

    # Publish LaserScan message
    laser_scan_pub.publish(laser_scan_msg)

if __name__ == '__main__':
    rospy.init_node('pointcloud_to_laserscan_node')

    # # Create a RealSense pipeline
    # pipeline = rs.pipeline()
    # config = rs.config()
    # config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)

    # # Start the RealSense pipeline
    # pipeline.start(config)

    # Create a publisher for LaserScan messages
    laser_scan_pub = rospy.Publisher('/laser_scan', LaserScan, queue_size=1)

    # Create a subscriber for PointCloud2 messages
    rospy.Subscriber('/camera/depth/points', PointCloud2, pointcloud_to_laserscan)

    # Spin ROS
    rospy.spin()

    # Stop the RealSense pipeline when the node is terminated
    pipeline.stop()
