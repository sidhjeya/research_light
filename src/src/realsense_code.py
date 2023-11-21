#!/usr/bin/env python3
import cv2                                # state of the art computer vision algorithms library
import numpy as np                        # fundamental package for scientific computing
import matplotlib.pyplot as plt           # 2D plotting library producing publication quality figures
import pyrealsense2 as rs                 # Intel RealSense cross-platform open-source API
print("Loading Intel Realsense Camera")
pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
pipeline.start(config)
while True:
        try:
           
            frames = pipeline.wait_for_frames()
            depth_frame = frames.get_color_frame()
            color_frame = frames.get_depth_frame()
            
            
            cv2.imshow("color_frame Detection", color_frame)
            cv2.imshow("depth_frame depth_frame", depth_frame)

            # cv2.imshow("YOLO Detectin", depth)
            if cv2.waitKey(1) == ord('q'):
                 rs.release()
                 cv2.destroyAllWindowstalker
                 break
            
        except:
            print("out of range")
            pass
# Cleanup:
pipeline.stop()
print("Frames Captured")