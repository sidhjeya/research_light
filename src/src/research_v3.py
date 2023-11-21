#!/usr/bin/env python3
import torch
# Import the rospy module

import rospy
# Import the String message type from std_msgs
from std_msgs.msg import String
import cv2
import numpy as np
from test.realsense_depth import DepthCamera

dc = DepthCamera()
# Load YOLOv3 model
model = torch.hub.load('ultralytics/yolov5', 'custom', path='/home/light/re_ws/src/test/src/yolov5-best.pt', force_reload=True)
#cap = cv2.VideoCapture(6)
#net = cv2.dnn.readNet("/home/light/Documents/research/darknet/cfg/yolov3.cfg", "/home/light/Documents/research/darknet/yolov3.weights")
x=0
y=0
width=0
height=0
label={}
point = [0,0]
distance=0
# cap = cv2.VideoCapture(6)

# Load COCO class names
# with open("/home/light/re_ws/src/test/src/coco.names", "r") as f:
#     classes = f.read().strip().split("\n")


# Define a callback function for the listener node
def callback(msg):
    # Print the received message
    rospy.loginfo("I heard %s", msg.data)

# Define the main function for the talker node
def talker():
    # Initialize the node with a name
    rospy.init_node("talker")
    # Create a publisher object for the chatter topic
    pub = rospy.Publisher("chatter", String, queue_size=10)
    # Create a rate object with 1 Hz frequency
    rate = rospy.Rate(1)
    # Loop until the node is shut down
    while not rospy.is_shutdown():
        # Publish a message with the current time
        pub.publish("The time is %s" % rospy.get_time())
        # Sleep for 1 second
        rate.sleep()

# Define the main function for the listener node
def listener():
    # Initialize the node with a name
    rospy.init_node("listener")
    # Create a subscriber object for the chatter topic
    sub = rospy.Subscriber("chatter", String, callback)
    # Keep the node running until it is shut down
    rospy.spin()



while True:
            try :    
                ret, depth_frame, color_frame = dc.get_frame()

                if not ret:
                    print("Can't receive frame. Exiting ...")
                    pass
                
                results = model(color_frame)
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
                    # distance=depth_frame[center_x,center_y]
                    # print('distance : ',distance)
                blob = cv2.dnn.blobFromImage(color_frame, scalefactor=1/255.0, size=(416, 416), swapRB=True, crop=False)
                cv2.putText(color_frame, str(distance), (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
                cv2.circle(color_frame,point,4,(0,0,255)) 
                cv2.rectangle(color_frame, (xA, yA), (xB, yB), (0, 255, 0), 2)
                cv2.imshow("YOLO Detection", color_frame)
                key = cv2.waitKey(1)
                if key == 27:
                    break
            except: 
               print("problem")
               break          
