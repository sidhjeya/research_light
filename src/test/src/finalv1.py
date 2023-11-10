#!/usr/bin/env python3
import cv2
import torch
import rospy
from std_msgs.msg import String
import pyrealsense2 as rs2
import statistics as st

from test.realsense_camera import RealsenseCamera
rs=RealsenseCamera()
point = [0,0]
xA=0
xB=0
yA=0
yB=0
distance =0
dis=0
model = torch.hub.load('ultralytics/yolov5', 'custom', path='/home/light/re_ws/src/test/src/yolov5-best.pt', force_reload=True)
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
    global distance,dis,classs,object,distance_bc
    while True:
        try:
            ret,color,depth=rs.get_frame_stream()
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

                            # print(center_x)
                            # print(center_y)
                            point[0]=int(center_x)
                            point[1]=int(center_y)
                            distance =depth[point[1],point[0]]
                            distance_bc=depth[yB,xB]
  
            if classs == 0 :
                  object = 'CARPET'
            if classs == 1 :
                  object = 'GLASS'
            if classs == 2 :
                  object = 'RAMP' 
            if classs == 3 :
                  object = 'WIRES'                                              
            print("-------------------------------------------->distance  BC",distance_bc)
            cv2.putText(color, "{}cm".format(distance/10), ( point[0],  point[1] - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
            cv2.putText(color, object, ( point[0]-50,  point[1] - 50), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
            cv2.circle(color ,point,4,(0,0,255)) 
            cv2.rectangle(color, (xA, yA), (xB, yB), (0, 255, 0), 2)
            
            cv2.imshow("YOLO Detection", color)
           
            if cv2.waitKey(1) == ord('q'):
                 rs.release()
                 cv2.destroyAllWindowstalker
                 break
            
        except:
            print("out of range")
            pass
        return distance
        
    
if __name__=='__main__':
       talker()
    #    listener()
    # processing()
  
