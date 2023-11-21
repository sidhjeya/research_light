#!/usr/bin/env python3

import rospy
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Point, PoseStamped

pub_map = None

def map_callback(msg):
    rospy.loginfo("Received a new map message with width: %d, height: %d", msg.info.width, msg.info.height)

def main():
    global pub_map

    rospy.init_node("Occupancy_Grid")
    # nh = rospy.NodeHandle()
    
    pub_map = rospy.Publisher("/map_new", OccupancyGrid, queue_size=1)

    # Subscribe to the "/map" topic
    rospy.Subscriber("/map", OccupancyGrid, map_callback)

    c_x, c_y = 0, 0
    polygon = []
    pt = Point()
    y, x = 3.0, 3.0
    pose_out = PoseStamped()

    map_pub = OccupancyGrid()
    pose = PoseStamped()

    map_pub.header.frame_id = "odom"
    map_pub.header.stamp = rospy.Time(1, 1)
    map_pub.info.map_load_time = rospy.Time(1, 2)
    map_pub.info.resolution = 0.1
    map_pub.info.width = 300
    map_pub.info.height = 300
    map_pub.info.origin.position.x = 0
    map_pub.info.origin.position.y = 0
    map_pub.info.origin.position.z = 0
    map_pub.info.origin.orientation.x = 0
    map_pub.info.origin.orientation.y = 0
    map_pub.info.origin.orientation.z = 0
    map_pub.info.origin.orientation.w = 1

    pose.header.frame_id = "odom"
    pose.header.stamp = rospy.Time(2, 3)
    pose.pose.position.x = 8.5
    pose.pose.position.y = 5
    pose.pose.position.z = 0
    pose.pose.orientation.w = 0.923
    pose.pose.orientation.x = 0
    pose.pose.orientation.y = 0
    pose.pose.orientation.z = -0.865

    for i in range(map_pub.info.width * map_pub.info.height):
        map_pub.data.append(0)

    while y < 20:
        c_x = int((8 - map_pub.info.origin.position.x) / map_pub.info.resolution)
        c_y = int((map_pub.info.origin.position.y - y) / map_pub.info.resolution) + map_pub.info.height
        i = c_x + (map_pub.info.height - c_y - 1) * map_pub.info.width
        map_pub.data[i] = 127
        y += 0.01

    pt.x = 1.23
    pt.y = 0.43
    polygon.append(pt)
    pt.x = 1.23
    pt.y = -0.43
    polygon.append(pt)
    pt.x = -0.42
    pt.y = -0.43
    polygon.append(pt)
    pt.x = -0.42
    pt.y = 0.43
    polygon.append(pt)

    while not rospy.is_shutdown():
        pub_map.publish(map_pub)
        rospy.spin()

if __name__ == '__main__':
    main()
