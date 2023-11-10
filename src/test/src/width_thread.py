#!/usr/bin/env python3
import cv2
import torch
import rospy
from std_msgs.msg import String
import statistics as st
import math
import pyrealsense2 as rs
import numpy as np
import threading
# from models import Darknet
# from models.experimental import attempt_load
point1 = [0,0]
point_tl=[0,0]
point_tr=[0,0]
point_br=[0,0]
point_bl=[0,0]
p_bl=[0,0]
p_br=[0,0]
xA=0
xB=0
yA=0
yB=0
distance =0     
dis=0
theta=90
alpha1=62
alpha2=62
alpha_hori=75
a=480
b=640
x_s=0
h=0.11
x_real=0
z_real=0
width=0
depth_bl=0
depth_br=0

model = torch.hub.load('ultralytics/yolov5','custom', path='/home/htetro/ree_ws/src/test/src/best_wire1.pt', force_reload=True)
 
if __name__ == "__main__":
    print("Loading Intel Realsense Camera")
    pipeline = rs.pipeline()

    config = rs.config()    
    config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
    config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)

    # Start streaming
    pipeline.start(config)
    align_to = rs.stream.color
    align = rs.align(align_to)
    # talker()

    pub = rospy.Publisher('chatter', String, queue_size=10)
    pub_width = rospy.Publisher('chatter_width', String, queue_size=10)    
    rospy.init_node('light', anonymous=True)
    rate = rospy.Rate(10)

    while True:
            # Wait for a coherent pair of frames: depth and color
            frames = pipeline.wait_for_frames()
            aligned_frames = align.process(frames)
            depth_frame = aligned_frames.get_depth_frame()
            color_frame = aligned_frames.get_color_frame()
            
            if not depth_frame or not color_frame:
                # # If there is no frame, probably camera not connected, return False
                # print("Error, impossible to get the frame, make sure that the Intel Realsense camera is correctly connected")
                # return False, None, None
                continue
            
        
            spatial = rs.spatial_filter()
            spatial.set_option(rs.option.holes_fill, 3)
            filtered_depth = spatial.process(depth_frame)

            hole_filling = rs.hole_filling_filter()
            filled_depth = hole_filling.process(filtered_depth)

            colorizer = rs.colorizer()
            depth_colormap = np.asanyarray(colorizer.colorize(filled_depth).get_data())
            depth_image = np.asanyarray(filled_depth.get_data())
            color_image = np.asanyarray(color_frame.get_data())
            color=color_image
            depth=depth_image
            color_profile = color_frame.get_profile().as_video_stream_profile()
            color_intrinsics = color_profile.get_intrinsics()

            results = model(color)
            col=color
            # cv2.imshow("RGB", col)
            for box in results.xyxy[0]:     
                            xB = int(box[2])
                            xA = int(box[0])
                            yB = int(box[3])
                            yA = int(box[1])
                            point_bl[0]=int(xA)
                            point_bl[1]=int(yB)
                            point_tr[0]=int(xB)
                            point_tr[1]=int(yA)  
                            point_br[0]=int(xB)
                            point_br[1]=int(yB)                          
                            classs= int(box[5])
                            center_x= (xA+xB)/2
                            center_y=(yA+yB)/2
                            point1[0]=int(center_x)
                            point1[1]=int(center_y)
                            # Get depth value of the pixel
                            print("center x",int(center_x), " centeer y --->",center_y)
                            depth = depth_frame.get_distance(int(center_x),int(center_y))
                            try : 
                                depth_bl=depth_frame.get_distance(int(point_bl[0]),int(point_bl[1]))
                                depth_br=depth_frame.get_distance(int(point_br[0]),int(point_br[1]))
                            except : 
                                pass
                            p_bl = rs.rs2_deproject_pixel_to_point(color_intrinsics, [int(point_bl[0]), int(point_bl[1])], depth_bl)
                            p_br = rs.rs2_deproject_pixel_to_point(color_intrinsics, [int(point_br[0]), int(point_br[1])], depth_br)
                            # Deproject pixel to center point
                            point = rs.rs2_deproject_pixel_to_point(color_intrinsics, [int(center_x), int(center_y)], depth)
                            width = abs(p_bl[0])+abs(p_br[0])
                            print("width of the objec   t-------------------------->",width)
                            # Print the point coordinates

                            z_real=point[2]
                            if classs == 0 :
                                    object = 'CARPET'
                            if classs == 1 :
                                    object = 'GLASS'
                            if classs == 2 :
                                    object = 'RAMP' 
                            if classs == 3 :
                                    object = 'WIRES' 
                            print(f"The 3D coordinates of the point are: {z_real}")
                            cv2.putText(color, "{}m".format(point[2]), ( point1[0],  point1[1] - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
                            cv2.putText(color, "{}m".format(width), ( point1[0],  point1[1] - 30), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
                            cv2.putText(color, object, ( point1[0]-50,  point1[1] - 50), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
                            cv2.circle(color ,(xA,yA),4,(0,0,255)) 
                            cv2.circle(color ,(xB,yB),4,(0,0,255)) 
                            cv2.circle(color ,point_tr,4,(0,0,255)) 
                            cv2.circle(color ,point_bl,4,(0,0,255))
                            cv2.rectangle(color, (xA, yA), (xB, yB), (0, 255, 0), 2)

            cv2.imshow("YOLO color", color) 
            hello_str = z_real % rospy.get_time()
            hello_str1 = width % rospy.get_time()
            rospy.loginfo(hello_str)
            pub.publish(str(hello_str))
            pub_width.publish(str(hello_str1))



            key = cv2.waitKey(1)
            # Press esc or 'q' to close the image window
            if key & 0xFF == ord('q') or key == 27:
                print("stooooooooooooooooooooping")
                cv2.destroyAllWindows()
                break
