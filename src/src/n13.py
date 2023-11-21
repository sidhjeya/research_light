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
from sensor_msgs.msg import Image
from sensor_msgs.msg import CameraInfo
from cv_bridge import CvBridge
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
bounding_box = [0, 0,0,0]
mod = np.random.rand(480, 640) * 10
model = torch.hub.load('ultralytics/yolov5', 'custom', path='/home/light/re_ws/src/test/src/best_wire1.pt', force_reload=True)
def dept_frame():
    frame_id = "depth_optical_frame"




def modify_depth_image(depth_image, bounding_box):
   
    x1, y1, x2, y2 = bounding_box

 
    
    modified_depth_image = np.copy(depth_image)
    print(modified_depth_image.shape)

    modified_depth_image[:y1, :] = 100
    modified_depth_image[y2:, :] = 100
    modified_depth_image[:, :x1] = 100
    modified_depth_image[:, x2:] = 100

    return modified_depth_image
if __name__ == "__main__":
    print("Loading Intel Realsense Camera")
    pipeline = rs.pipeline()
    config = rs.config()    
    config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
    config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
    pipeline.start(config)
    align_to = rs.stream.color
    align = rs.align(align_to)
    pub = rospy.Publisher('chatter', String, queue_size=10)
    pub_width = rospy.Publisher('/t_chatter_width', String, queue_size=10) 
    depth_image_publisher = rospy.Publisher('/t_depth_image', Image, queue_size=10)   
    camera_info_pub = rospy.Publisher('/bef_camera_info', CameraInfo, queue_size=10)
    bridge = CvBridge()
    camera_info = CameraInfo()
    rospy.init_node('light', anonymous=True)
    rate = rospy.Rate(10)
# fx,fy=381.243 ,381.243    cx,cy=321.739 , 241.478
    camera_info = CameraInfo()
    camera_info.header.frame_id="mod_depth_frame"
    camera_info.width = 640  
    camera_info.height = 480 
    camera_info.distortion_model = 'plumb_bob'
    # camera_info.K = [400.0, 0.0, 320.0, 0.0, 400.0, 240.0, 0.0, 0.0, 1.0]
    # camera_info.P = [400.0, 0.0, 320.0, 0.0, 0.0, 400.0, 240.0, 0.0, 0.0, 0.0, 1.0, 0.0]

    camera_info.K = [381.243, 0.0, 321.739, 0.0, 381.243, 321.739, 0.0, 0.0, 1.0]
    camera_info.P = [381.243, 0.0, 321.739, 0.0, 0.0, 381.243, 241.478, 0.0, 0.0, 0.0, 1.0, 0.0]

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
            depth_fr=depth_image


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
                            bounding_box[0]=xA
                            bounding_box[1]=yA
                            bounding_box[2]=xB
                            bounding_box[3]=yB
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
                            mod=modify_depth_image(depth_fr,bounding_box)
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
            cv2.imshow("YOLO depth_fr", depth_image)
            cv2.imshow("MOdified depth", mod) 

            #adding frame to the modified depth image
            depth_image_message = bridge.cv2_to_imgmsg(mod, encoding="passthrough")
            depth_image_message.header.frame_id = "mod_depth_frame"
            depth_image_message.header.stamp=rospy.Time.now()
            camera_info.header.stamp = depth_image_message.header.stamp
            hello_str = z_real % rospy.get_time()
            hello_str1 = width % rospy.get_time()
            rospy.loginfo(hello_str)
            pub.publish(str(hello_str))
            pub_width.publish(str(hello_str1))
            depth_image_publisher.publish(depth_image_message)
            camera_info_pub.publish(camera_info)
        

            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
                cap.release()
                cv2.destroyAllWindows()
