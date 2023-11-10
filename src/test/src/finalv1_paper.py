#!/usr/bin/env python3
import cv2
import torch
import rospy
from std_msgs.msg import String
import pyrealsense2 as rs2
import statistics as st
import math

from test.realsense_camera import RealsenseCamera
rs=RealsenseCamera()
point = [0,0]
point_tl=[0,0]
point_tr=[0,0]
point_br=[0,0]
point_bl=[0,0]
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
model = torch.hub.load('ultralytics/yolov5', 'custom', path='/home/light/re_ws/src/test/src/best_wire1.pt', force_reload=True)
def callback(data):
    rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)


def talker():
    pub = rospy.Publisher('chatter', String, queue_size=10)
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        dis=processing()
       
        
        hello_str = dis % rospy.get_time()
        rospy.loginfo(hello_str/1000)
        pub.publish(str(hello_str/1000))
        
        rate.sleep()
def processing():
    global distance,dis,classs,object,distance_bc,x_s,x_real,dis_tl,dis_br,dis_tr,dis_bl
    ret,color,depth=rs.get_frame_stream()
    while True:
        try:
            
            if ret == True:
                 print("no frame recevied")
            results = model(color)
            for box in results.xyxy[0]:     
                            xB = int(box[2])
                            xA = int(box[0])
                            yB = int(box[3])
                            yA = int(box[1])
                            classs= int(box[5])
                            center_x= (xA+xB)/2
                            center_y=(yA+yB)/2
                            # center_x=321
                            # center_y=241
                            # print(center_x)
                            # print(center_y)
                            point_bl[0]=int(xA)
                            point_bl[1]=int(yB)
                            point_tr[0]=int(xB)
                            point_tr[1]=int(yA)
                            point[0]=int(center_x)
                            point[1]=int(center_y)
                            distance =depth[point[1],point[0]]
                            dis_tl=depth[yA,xA]
                            dis_br=depth[yB,xB]
                            dis_tr=depth[point_tr]
                            #distance_bc=depth[yB,xB]
                            zx=center_x/a
                            x_real = h*math.tan(theta + math.atan(math.tan(alpha1)-(zx)*(math.tan(alpha1)+math.tan(alpha2))))
                            y_real = x_real *math.tan(alpha_hori)*(1-(2*center_y/b))
                            # x_real = h*math.tan(theta+ math.atan(math.tan(alpha1)-(center_x/a)*math.tan(alpha1)+math.tan(alpha2)))
                            print("center new  x ----------------------> ",x_real,"y real -->",y_real)
                            # print(results.pandas().xyxy[0])
            print("top left",dis_tl)
            print("btm_right",dis_br)
            
            if classs == 0 :
                  object = 'CARPET'
            if classs == 1 :
                  object = 'GLASS'
            if classs == 2 :
                  object = 'RAMP' 
            if classs == 3 :
                  object = 'WIRES'                                              
            # print("-------------------------------------------->distance  BC",distance_bc)
            cv2.putText(color, "{}cm".format(distance/10), ( point[0],  point[1] - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
            cv2.putText(color, object, ( point[0]-50,  point[1] - 50), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
            cv2.circle(color ,point,4,(0,0,255)) 
            cv2.circle(color ,(xA,yA),4,(0,0,255)) 
            cv2.circle(color ,(xB,yB),4,(0,0,255)) 
            cv2.circle(color ,point_tr,4,(0,0,255)) 
            cv2.circle(color ,point_bl,4,(0,0,255)) 
            cv2.rectangle(color, (xA, yA), (xB, yB), (0, 255, 0), 2)
            
            cv2.imshow("YOLO color", color)
          
           
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
            
        except:
            print("out of range")
            pass
        return distance
        
    
if __name__=='__main__':
       talker()
    #    listener()
    # processing()
  
