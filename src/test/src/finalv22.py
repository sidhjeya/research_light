#!/usr/bin/env python3

import rospy
import torch
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from geometry_msgs.msg import Point
import cv2
from test.msg import mylist
point = [0,0]
xA=0
xB=0
yA=0
yB=0
distance =0
x=0
y=0
xA=0
xB=0
yA=0
yB=0
def point_call_back(msg):
    global x,y,xA,xB,yA,yB
    list=msg.data
    x=list[4]
    y=list[5]
    xB=list[0]
    xA=list[1]
    yB=list[2]
    yA=list[3]


def depth_frame_callback(data):
    try:
        # Convert ROS Image message to OpenCV image
        bridge = CvBridge()
        depth_image = bridge.imgmsg_to_cv2(data, desired_encoding="16UC1")

                               

        distance =depth_image[x,y]
        print("distance",distance)
    
   
        # cv2.imshow("dpeth Frame", depth_image)
        cv2.waitKey(1)

    except Exception as e:
        rospy.logerr("Error processing color frame: %s", str(e))

def main():
    rospy.init_node('realsense_depth_frame_subscriber', anonymous=True)

    # Subscribe to the color frame topic
    rospy.Subscriber('obj_pnt', mylist, point_call_back)
    rospy.Subscriber('/camera/depth/image', Image, depth_frame_callback)
    


    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
