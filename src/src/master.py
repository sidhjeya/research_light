#!/usr/bin/env python3
import cv2
import torch
import rospy
from std_msgs.msg import String

import pyrealsense2 as rs2
import statistics as st
from rospy.numpy_msg import numpy_msg
from rospy_tutorials.msg import Floats
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import numpy as np
from test.realsense_camera import RealsenseCamera
bridge = CvBridge()
rs=RealsenseCamera()
point = [0,0]
xA=0
xB=0
yA=0
yB=0
distance =0
# model = torch.hub.load('ultralytics/yolov5', 'custom', path='/home/light/re_ws/src/test/src/yolov5-best.pt', force_reload=True)
def callback(data):
    rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)
    


def processing():
    global distance
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    
    while not rospy.is_shutdown():
        try:
            ret,color,depth=rs.get_frame_stream()
            if ret == False:
                 print("no frame recevied")
            print("Sending IMages>>>>>>.")
            # Convert the OpenCV image to a ROS Image message
            cv2.imshow("color",color)
            ros_image = bridge.cv2_to_imgmsg(color, encoding="bgr8")
            ros_image1 = bridge.cv2_to_imgmsg(depth, encoding="passthrough")
            ros_image1.encoding = "16UC1"

            pub.publish(ros_image)
            pub1.publish(ros_image1)
            
            # cv2.imshow("YOLO Detection", color)
            # cv2.imshow("YOLO", depth)
            if cv2.waitKey(1) == ord('q'):
                 rs.release()
                 cv2.destroyAllWindows
                 break
            
        except:
            print("out of range")
            pass    
        
    
if __name__=='__main__':
    #    talker()
    #    listener()
    pub = rospy.Publisher('/camera/color/image', Image, queue_size=10)
    pub1 = rospy.Publisher('/camera/depth/image', Image, queue_size=10)

    processing()

  
