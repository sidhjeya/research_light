#!/usr/bin/env python3

import rospy
from nav_msgs.msg import OccupancyGrid
import numpy as np

# Initialize ROS node
rospy.init_node('update_map_node')

# Load the existing occupancy grid map or create one
# Replace the following lines with code to load/create your map
map_width = 100  # Adjust according to your map dimensions
map_height = 100
map_data = np.zeros((map_width, map_height), dtype=np.int8)

# Update the map with the object's position
object_x = 6  # Replace with the object's position in grid cell coordinates
object_y = 40
object_width_cells = 5  # Width of the object in grid cells

# Update the occupancy grid cells
for i in range(object_x - object_width_cells // 2, object_x + object_width_cells // 2 + 1):
    map_data[i, object_y] = 100  # Set cells as occupied

# Create an OccupancyGrid message
map_msg = OccupancyGrid()
map_msg.header.stamp = rospy.Time.now()
map_msg.header.frame_id = "map"
map_msg.info.map_load_time = rospy.Time.now()
map_msg.info.resolution = 0.1  # Adjust based on your map resolution
map_msg.info.width = map_width
map_msg.info.height = map_height
map_msg.data = map_data.flatten().tolist()

# Publish the updated map
map_publisher = rospy.Publisher('map', OccupancyGrid, queue_size=1)
map_publisher.publish(map_msg)

rospy.spin()
