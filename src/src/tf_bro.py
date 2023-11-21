#!/usr/bin/env python3  

import rospy
import tf

if __name__ == '__main__':
    rospy.init_node('my_transform_broadcaster')

    # Create a transform broadcaster
    br = tf.TransformBroadcaster()

    rate = rospy.Rate(10.0)  # 10 Hz

    while not rospy.is_shutdown():
        # Broadcast a transform from frame 'base_link' to 'child_frame'
        br.sendTransform(
            (1.0, 2.0, 0.0),        # translation (x, y, z)
            (0.0, 0.0, 0.0, 1.0),   # rotation (quaternion)
            rospy.Time.now(),      # timestamp
            'camera_depth_frame',         # child frame
            'base_link'            # parent frame
        )

        rate.sleep()
