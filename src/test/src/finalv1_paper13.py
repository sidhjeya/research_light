#!/usr/bin/env python3
import pyrealsense2 as rs

# Create a pipeline object
pipeline = rs.pipeline()

# Configure the pipeline to stream both color and depth frames
config = rs.config()
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)

# Start streaming
pipeline.start(config)

# Get frameset of color and depth
frames = pipeline.wait_for_frames()
color_frame = frames.get_color_frame()
depth_frame = frames.get_depth_frame()

# Get intrinsics of the color stream
color_profile = color_frame.get_profile().as_video_stream_profile()
color_intrinsics = color_profile.get_intrinsics()

# Get pixel coordinates of a point of interest
px = 310 # x-coordinate of the pixel
py = 223 # y-coordinate of the pixel

# Get depth value of the pixel
depth = depth_frame.get_distance(px, py)

# Deproject pixel to point
point = rs.rs2_deproject_pixel_to_point(color_intrinsics, [px, py], depth)

# Print the point coordinates
print(f"The 3D coordinates of the point are: {point}")
