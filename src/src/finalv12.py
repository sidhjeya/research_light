#!/usr/bin/env python3
import cv2
import torch
import rospy
from std_msgs.msg import String
import pyrealsense2 as rs2
import statistics as st

from test.realsense_depth import DepthCamera
dc=DepthCamera()
ret,depth_frame,color_frame =dc.get_frame()
point=(400,300)
cv2.circle(color_frame,point,4,(0,0,255))
distance=depth_frame[point[1],point[0]]
print(distance)
cv2.imshow("depth+frame",depth_frame)
