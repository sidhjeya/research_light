#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image, CameraInfo
from message_filters import ApproximateTimeSynchronizer, Subscriber

def callback(image_msg, camera_info_msg):
    # Your synchronization logic goes here
    # image_msg is the synchronized image message
    # camera_info_msg is the synchronized camera info message

    # Example: Print timestamps of synchronized messages
    rospy.loginfo("Synchronized Image timestamp: %s", str(image_msg.header.stamp))
    rospy.loginfo("Synchronized CameraInfo timestamp: %s", str(camera_info_msg.header.stamp))

    # Perform your desired operations with synchronized messages here
    # e.g., publish the synchronized messages to a new topic

def main():
    rospy.init_node('synchronized_publisher_node')

    # Set the image topic you want to subscribe to
    image_topic = '/t_depth_image'

    # Set the camera info topic you want to subscribe to
    camera_info_topic = '/bef_camera_info'

    # Create subscribers for image and camera info topics
    image_sub = Subscriber(image_topic, Image)
    camera_info_sub = Subscriber(camera_info_topic, CameraInfo)

    # Use ApproximateTimeSynchronizer to synchronize the messages
    ts = ApproximateTimeSynchronizer([image_sub, camera_info_sub], queue_size=10, slop=0.0001)
    ts.registerCallback(callback)

    # Create publishers for synchronized image and camera info topics
    synchronized_image_pub = rospy.Publisher('/sync_depth', Image, queue_size=10)
    synchronized_camera_info_pub = rospy.Publisher('/camera_info', CameraInfo, queue_size=10)

    # Create a callback to publish the synchronized messages
    def publish_synchronized_messages(image_msg, camera_info_msg):
        synchronized_image_pub.publish(image_msg)
        synchronized_camera_info_pub.publish(camera_info_msg)

    # Use ApproximateTimeSynchronizer to synchronize the messages before publishing
    publish_ts = ApproximateTimeSynchronizer([image_sub, camera_info_sub], queue_size=10, slop=0.0001)
    publish_ts.registerCallback(publish_synchronized_messages)

    rospy.spin()

if __name__ == '__main__':
    main()
