#!/usr/bin/env python3

import pyrealsense2 as rs
import numpy as np
import cv2
import torch
import rospy
from geometry_msgs.msg import PointStamped
from std_msgs.msg import Header
from geometry_msgs.msg import Point

#define model path
model_path = '/home/htetro/ree_ws/src/test/src/best_wire1.pt'
model = torch.hub.load('ultralytics/yolov5', 'custom', model_path)


def show_coordinate(org_img, boxs, depth_intrin):
    img = org_img.copy()
    dock_coordinate = [0,0,0]
    lu_coordinate = [0,0,0]
    ld_coordinate = [0,0,0]
    ru_coordinate = [0,0,0]
    rd_coordinate = [0,0,0]
    for box in boxs:
        cv2.rectangle(img, (int(box[0]), int(box[1])), (int(box[2]), int(box[3])), (0, 255, 0), 2)
        if box[0]==640:
            box[0]-=1
        if box[2]==640:
            box[2]-=1
        if box[1]==480:
            box[1]-=1
        if box[3]==480:
            box[3]-=1
        mid_pos = [int((box[0] + box[2])//2), int((box[1] + box[3])//2)]
        lu_pos = [int(box[0]), int(box[1])]
        ld_pos = [int(box[0]), int(box[3])]
        ru_pos = [int(box[2]), int(box[1])]
        rd_pos = [int(box[2]), int(box[3])]

        img = cv2.circle(img, [mid_pos[0], mid_pos[1]], radius=3, color=(0, 0, 255), thickness=-1)
        img = cv2.circle(img, [lu_pos[0], lu_pos[1]], radius=3, color=(255, 0, 0), thickness=-1)
        img = cv2.circle(img, [ld_pos[0], ld_pos[1]], radius=3, color=(255, 0, 0), thickness=-1)
        img = cv2.circle(img, [ru_pos[0], ru_pos[1]], radius=3, color=(255, 0, 0), thickness=-1)
        img = cv2.circle(img, [rd_pos[0], rd_pos[1]], radius=3, color=(255, 0, 0), thickness=-1)
        
        dist_c = aligned_depth_frame.get_distance(mid_pos[0], mid_pos[1])
        dist_lu = aligned_depth_frame.get_distance(lu_pos[0], lu_pos[1])
        dist_ld = aligned_depth_frame.get_distance(ld_pos[0], ld_pos[1])
        dist_ru = aligned_depth_frame.get_distance(ru_pos[0], ru_pos[1])
        dist_rd = aligned_depth_frame.get_distance(rd_pos[0], rd_pos[1])

        dock_coordinate = rs.rs2_deproject_pixel_to_point(depth_intrin, mid_pos, dist_c)
        lu_coordinate = rs.rs2_deproject_pixel_to_point(depth_intrin, lu_pos, dist_lu)
        ld_coordinate = rs.rs2_deproject_pixel_to_point(depth_intrin, ld_pos, dist_ld)
        ru_coordinate = rs.rs2_deproject_pixel_to_point(depth_intrin, ru_pos, dist_ru)
        rd_coordinate = rs.rs2_deproject_pixel_to_point(depth_intrin, rd_pos, dist_rd)

        #cv2.putText(img, box[-1]+str('%.2f' % box[-3]),
                    #(int(box[0]), int(box[1])), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
        cv2.putText(img, "("+str('%.2f' %dock_coordinate[0])+","+str('%.2f' %dock_coordinate[1])+","+str('%.2f' %dock_coordinate[2])+")", 
                    (mid_pos[0], mid_pos[1]), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
        cv2.putText(img, "("+str('%.2f' %lu_coordinate[0])+","+str('%.2f' %lu_coordinate[1])+","+str('%.2f' %lu_coordinate[2])+")", 
                    (lu_pos[0], lu_pos[1]), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
        cv2.putText(img, "("+str('%.2f' %ld_coordinate[0])+","+str('%.2f' %ld_coordinate[1])+","+str('%.2f' %ld_coordinate[2])+")", 
                    (ld_pos[0], ld_pos[1]), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
        cv2.putText(img, "("+str('%.2f' %ru_coordinate[0])+","+str('%.2f' %ru_coordinate[1])+","+str('%.2f' %ru_coordinate[2])+")", 
                    (ru_pos[0], ru_pos[1]), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
        cv2.putText(img, "("+str('%.2f' %rd_coordinate[0])+","+str('%.2f' %rd_coordinate[1])+","+str('%.2f' %rd_coordinate[2])+")", 
                    (rd_pos[0], rd_pos[1]), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
    cv2.imshow('dock_detection', img)

    return dock_coordinate, lu_coordinate, ld_coordinate, ru_coordinate, rd_coordinate


if __name__ == "__main__":
    # Configure depth and color streams
    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 60)
    config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 60)
    align_to = rs.stream.color
    align = rs.align(align_to)
    # Start streaming
    pipeline.start(config)

    #init ros node
    pub_center = rospy.Publisher('dock_center_point', PointStamped, queue_size=10)
    pub_lu = rospy.Publisher('dock_lf_point', PointStamped, queue_size=10)
    pub_ru = rospy.Publisher('dock_ru_point', PointStamped, queue_size=10)
    pub_ld = rospy.Publisher('dock_ld_point', PointStamped, queue_size=10)
    pub_rd = rospy.Publisher('dock_rd_point', PointStamped, queue_size=10)
    rospy.init_node('dock_locate_node', anonymous=True)
    rate = rospy.Rate(10)

    try:
        while True:
            # Wait for a coherent pair of frames: depth and color
            frames = pipeline.wait_for_frames()
            aligned_frames = align.process(frames)
            aligned_depth_frame = aligned_frames.get_depth_frame()
            color_frame = aligned_frames.get_color_frame()
            intr = color_frame.profile.as_video_stream_profile().intrinsics
            depth_intrin = aligned_depth_frame.profile.as_video_stream_profile().intrinsics
            if not aligned_depth_frame or not color_frame:
                continue
            # Convert images to numpy arrays

            depth_image = np.asanyarray(aligned_depth_frame.get_data())

            color_image = np.asanyarray(color_frame.get_data())

            results = model(color_image)
            boxs = results.pandas().xyxy[0].values
            dock_coordinate, lu_coordinate, ld_coordinate, ru_coordinate, rd_coordinate = show_coordinate(color_image, boxs, depth_intrin)
            #publish ros topic
            if dock_coordinate[2]>0:
                dock_header = Header(stamp=rospy.Time.now(), frame_id="camera_link")
                dock_point = Point(dock_coordinate[0], dock_coordinate[1], dock_coordinate[2])
                lu_point = Point(lu_coordinate[0], lu_coordinate[1], lu_coordinate[2])
                ld_point = Point(ld_coordinate[0], ld_coordinate[1], ld_coordinate[2])
                ru_point = Point(ru_coordinate[0], ru_coordinate[1], ru_coordinate[2])
                rd_point = Point(rd_coordinate[0], rd_coordinate[1], rd_coordinate[2])

                dock_point_stamped = PointStamped(header = dock_header, point = dock_point)
                lu_point_stamped = PointStamped(header = dock_header, point = lu_point)
                ld_point_stamped = PointStamped(header = dock_header, point = ld_point)
                ru_point_stamped = PointStamped(header = dock_header, point = ru_point)
                rd_point_stamped = PointStamped(header = dock_header, point = rd_point)

                pub_center.publish(dock_point_stamped)
                pub_lu.publish(lu_point_stamped)
                pub_ld.publish(ld_point_stamped)
                pub_ru.publish(ru_point_stamped)
                pub_rd.publish(rd_point_stamped)

            # depth image to color map (image must be converted to 8-bit per pixel first)
            depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.03), cv2.COLORMAP_JET)
            # Stack both images horizontally
            images = np.hstack((color_image, depth_colormap))
            # Show images
            cv2.namedWindow('Docking System Detection', cv2.WINDOW_AUTOSIZE)
            cv2.imshow('Docking System Detection', images)
            key = cv2.waitKey(1)
            # Press esc or 'q' to close the image window
            if key & 0xFF == ord('q') or key == 27:
                cv2.destroyAllWindows()
                break
    finally:
        # Stop streaming
        pipeline.stop()
