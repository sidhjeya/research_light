import torch
# Import the rospy module
import rospy
# Import the String message type from std_msgs
from std_msgs.msg import String
import cv2
import numpy as np
import pyrealsense2
from realsense_depth import *
from test.realsense_depth import DepthCamera

dc = DepthCamera()
# Load YOLOv3 model
model = torch.hub.load('ultralytics/yolov5', 'custom', path='yolov5-best.pt', force_reload=True)
cap = cv2.VideoCapture(0)
net = cv2.dnn.readNet("/home/light/Documents/research/darknet/cfg/yolov3.cfg", "/home/light/Documents/research/darknet/yolov3.weights")
x=0
y=0
width=0
height=0
label={}
point = [0,0]
distance=0
# Load COCO class names
with open("/home/light/re_ws/src/test/src/coco.names", "r") as f:
    classes = f.read().strip().split("\n")


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
      global distance,x,y,width,height,label,point
      while True:
            try :    
                ret, depth_frame, color_frame = dc.get_frame()

                blob = cv2.dnn.blobFromImage(color_frame, scalefactor=1/255.0, size=(416, 416), swapRB=True, crop=False)
                net.setInput(blob)

                # Run YOLO inference
                layer_names = net.getUnconnectedOutLayersNames()
                outputs = net.forward(layer_names)

                # Process detections
                for output in outputs:
                    for detection in output:
                        scores = detection[5:]
                        class_id = np.argmax(scores)
                        confidence = scores[class_id]
                        if confidence > 0.5:  # Minimum confidence threshold
                            center_x = int(detection[0] * color_frame.shape[1])
                            center_y = int(detection[1] * color_frame.shape[0])
                            width = int(detection[2] * color_frame.shape[1])
                            height = int(detection[3] * color_frame.shape[0])
                            x = int(center_x - width / 2)
                            y = int(center_y - height / 2)

                            label = f"{classes[class_id]}: {confidence:.2f}"
                            distance=depth_frame[center_x,center_y]
                            point[0]=center_x
                            point[1]=center_y
                cv2.circle(color_frame,point,4,(0,0,255))    
                print(distance)            
                cv2.rectangle(color_frame, (x, y), (x + width, y + height), (0, 255, 0), 2)
                cv2.putText(color_frame, str(label), (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

                # Display image with detections
                cv2.imshow("YOLO Detection", color_frame)
                key = cv2.waitKey(1)
                if key == 27:
                    break
            except: 
                print('probelm')
                break
              
# Run the main function if this module is executed
if __name__ == "__main__":
    try:
        processing()
        while True :
            ret, frame = cap.read()

            if not ret:
                print("Can't receive frame. Exiting ...")
                break

        
            results = model(frame)
            print(results.pandas().xyxy[0])
            #results.save()
            cv2.imshow('frame', frame)
            if cv2.waitKey(1) == ord('q'):
                break

    except rospy.ROSInterruptException:
        pass
