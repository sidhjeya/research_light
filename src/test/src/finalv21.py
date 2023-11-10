#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import torch
import time
from geometry_msgs.msg import Point
from test.msg import mylist

device = torch.device("cpu")
model = torch.hub.load('ultralytics/yolov5', 'custom', path='/home/htetro/best_wire1.pt', force_reload=True)
point = [0,0,0]
msg=mylist()

def color_frame_callback(data):
    try:
        # Convert ROS Image message to OpenCV image
        bridge = CvBridge()
        color_image = bridge.imgmsg_to_cv2(data, desired_encoding="bgr8")
        results = model(color_image)
        for box in results.xyxy[0]:     
                            xB = int(box[2])
                            xA = int(box[0])
                            yB = int(box[3])
                            yA = int(box[1])
                            center_x= (xA+xB)/2
                            center_y=(yA+yB)/2
                            print(center_x)
                            print(center_y)
                            point[0]=int(center_x)
                            point[1]=int(center_y)
                            msg.data[0]=xB
                            msg.data[1]=xA
                            msg.data[2]=yB
                            msg.data[3]=yA
                            msg.data[4]=center_x
                            msg.data[5]=center_y
                            
                            
        print(xA)
        print(xB)
        # Display the color frame
        print(results.pandas().xyxy[0])
        # cv2.imshow("Color Frame", color_image)
        pub.publish(point)
        cv2.waitKey(1)

    except Exception as e:
        rospy.logerr("Error processing color frame: %s", str(e))

def main():
    rospy.init_node('realsense_color_frame_subscriber', anonymous=True)

    # Subscribe to the color frame topic
    rospy.Subscriber('/camera/color/image', Image, color_frame_callback)

    rospy.spin()

if __name__ == '__main__':
    try:
        pub = rospy.Publisher('obj_pnt', mylist, queue_size=10)
        main()
    except rospy.ROSInterruptException:
        pass
