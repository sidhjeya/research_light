#!/usr/bin/env python3
import pyrealsense2 as rs
import numpy as np
import torch
import rospy
from std_msgs.msg import String
import cv2

x=0
y=0
width=0
height=0
label={}
point = [0,0]
distance=0

model = torch.hub.load('ultralytics/yolov5', 'custom', path='yolov5-best.pt', force_reload=True)
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

def processing():
    while True :
        # try :    
                frames = pipeline.wait_for_frames()
                depth_frame = frames.get_depth_frame()
                color_frame = frames.get_color_frame()
                depth_frame = frames.as_depth_frame()

                # if not depth_frame or not color_frame:
                #     break

                # results = model(color_frame)
                # for box in results.xyxy[0]:     
                #     xB = int(box[2])
                #     xA = int(box[0])
                #     yB = int(box[3])
                #     yA = int(box[1])
                #     center_x= (xA+xB)/2
                #     center_y=(yA+yB)/2
                #     print(center_x)
                #     print(center_y)
                #     point[0]=int(center_x)
                #     point[1]=int(center_y)
                #     # distance=depth_frame[center_x,center_y]
                #     # print('distance : ',distance)
                # # blob = cv2.dnn.blobFromImage(color_frame, scalefactor=1/255.0, size=(416, 416), swapRB=True, crop=False)
                # # cv2.putText(color_frame, str(distance), (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
                # cv2.circle(color_frame,point,4,(0,0,255)) 
                # cv2.rectangle(color_frame, (xA, yA), (xB, yB), (0, 255, 0), 2)
                cv2.imshow("YOLO Detection", depth_frame)
                key = cv2.waitKey(1)
                if key == 27:
                    break
        # except: 
        #        print("problem")
        #        break              

if __name__ == "__main__":

    pipeline = rs.pipeline()
    config = rs.config()
    # Get device product line for setting a supporting resolution
    pipeline_wrapper = rs.pipeline_wrapper(pipeline)
    pipeline_profile = config.resolve(pipeline_wrapper)
    device = pipeline_profile.get_device()
    device_product_line = str(device.get_info(rs.camera_info.product_line))
    config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
    config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
    pipeline.start(config)
    processing()
    
