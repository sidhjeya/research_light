#!/usr/bin/env python3

import rospy
import torch
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from geometry_msgs.msg import Point
from test.msg import mylist
import cv2
mes=mylist()
point = [0,0]
xA=0
xB=0
yA=0
yB=0
distance =0
x=0
y=0

def point_call_back(msg):
    global x,y,xA,xB,yA,yB
    print("poits received")
  
    print(msg)
    
def color_frame_callback(data):
    try:
        # print("xa",xA)
        # print("xb",xB)
        bridge = CvBridge()
        color_image = bridge.imgmsg_to_cv2(data, desired_encoding="bgr8")
        # cv2.putText(color_image, "{}cm".format(distance/10), ( x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
        cv2.circle(color_image ,point,4,(0,0,255)) 
        cv2.rectangle(color_image, (xA, yA), (xB, yB), (0, 255, 0), 2)
         
        cv2.imshow("YOLO Detection", color_image)    
                               

        
    
   
        # cv2.imshow("dpeth Frame", depth_image)
        cv2.waitKey(1)

    except Exception as e:
        rospy.logerr("Error processing color frame: %s", str(e))

def main():
    rospy.init_node('realsense_depth_frame_subscriber', anonymous=True)

    # Subscribe to the color frame topic
    rospy.Subscriber('obj_pnt', mylist, point_call_back)
    rospy.Subscriber('/camera/color/image', Image, color_frame_callback)
    


    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
