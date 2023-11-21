#!/usr/bin/env python3
######### 638,477
import torch
import cv2
center_x=0
center_y=0
point = [0,0]
model = torch.hub.load('ultralytics/yolov5', 'custom', path='/home/light/re_ws/src/test/src/best_wire1.pt', force_reload=True)


cap = cv2.VideoCapture(6)
while True:
   
    ret, frame = cap.read()

    if not ret:
        print("Can't receive frame. Exiting ...")
        break

  
    results = model(frame)
    print(results.pandas().xyxy[0])
    for box in results.xyxy[0]:     
                            xB = int(box[2])
                            xA = int(box[0])
                            yB = int(box[3])
                            yA = int(box[1])
                            center_x= (xA+xB)/2
                            center_y=(yA+yB)/2
                            point[0]=int(center_x)
                            point[1]=int(center_y)
    #results.save()
    cv2.circle(frame ,point,4,(0,0,255)) 
    cv2.imshow('frame', frame)

    if cv2.waitKey(1) == ord('q'):
        break
cap.release()
cv2.destroyAllWindows()

##########################################################1277,718
# import cv2
# import torch
# import rospy
# from std_msgs.msg import String
# point = [0,0]
# xA=0
# xB=0
# yA=0
# yB=0
# distance =0
# model = torch.hub.load('ultralytics/yolov5', 'custom', path='/home/light/re_ws/src/test/src/yolov5-best.pt', force_reload=True)
# from test.realsense_camera import RealsenseCamera
# rs=RealsenseCamera()
# while True:
#         try:
#             ret,color,depth=rs.get_frame_stream()
#             if ret == True:
#                  print("no frame recevied")
#             results = model(color)
#             # for box in results.xyxy[0]:     
#             #                 xB = int(box[2])
#             #                 xA = int(box[0])
#             #                 yB = int(box[3])
#             #                 yA = int(box[1])
#             #                 center_x= (xA+xB)/2
#             #                 center_y=(yA+yB)/2
#             #                 print(center_x)
#             #                 print(center_y)
#             #                 point[0]=int(center_x)
#             #                 point[1]=int(center_y)
                            

#             # distance =depth[point[0],point[1]]
#             # print(distance)
#             # print(results.pandas().xyxy[0])
         
#             # cv2.putText(color, "{}cm".format(distance/10), ( point[0],  point[1] - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
#             # cv2.circle(color ,point,4,(0,0,255)) 
#             # cv2.rectangle(color, (xA, yA), (xB, yB), (0, 255, 0), 2)
#             print(results.pandas().xyxy[0])
#             cv2.imshow("YOLO Detection", color)
#         except:
#             print("out of range")
#             pass
#         key=cv2.waitKey(1)
#         if key==27:
#             break
# rs.release()
# cv2.destroyAllWindows()