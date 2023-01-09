## License: Apache 2.0. See LICENSE file in root directory.
## Copyright(c) 2015-2017 Intel Corporation. All Rights Reserved.

###############################################
##      Open CV and Numpy integration        ##
###############################################
# First import the library
import random
import time

import pyrealsense2 as rs
# Import Numpy for easy array manipulation
import numpy as np
# Import OpenCV for easy image rendering
import cv2

from networktables import NetworkTables

# Initialize Network Tables
# As a client to connect to a robot
NetworkTables.initialize(server='127.0.0.1')

red_pos_entry = NetworkTables.getEntry('/realsense/pos_red')
blue_pos_entry = NetworkTables.getEntry('/realsense/pos_blue')
latency_entry = NetworkTables.getEntry('/realsense/latency')

# Create a pipeline
pipeline = rs.pipeline()

# Create a config and configure the pipeline to stream
#  different resolutions of color and depth streams
config = rs.config()

# Get device product line for setting a supporting resolution
pipeline_wrapper = rs.pipeline_wrapper(pipeline)
pipeline_profile = config.resolve(pipeline_wrapper)
device = pipeline_profile.get_device()
device_product_line = str(device.get_info(rs.camera_info.product_line))

found_rgb = False
for s in device.sensors:
    if s.get_info(rs.camera_info.name) == 'RGB Camera':
        found_rgb = True
        break
if not found_rgb:
    print("The demo requires Depth camera with Color sensor")
    exit(0)

config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)

if device_product_line == 'L500':
    config.enable_stream(rs.stream.color, 960, 540, rs.format.bgr8, 30)
else:
    config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

# Start streaming
profile = pipeline.start(config)
color_sensor = profile.get_device().query_sensors()[1]
color_sensor.set_option(rs.option.enable_auto_exposure, False)
color_sensor.set_option(rs.option.enable_auto_white_balance, False)
color_sensor.set_option(rs.option.exposure, 400)
color_sensor.set_option(rs.option.white_balance, 3500)

# Getting the depth sensor's depth scale (see rs-align example for explanation)
depth_sensor = profile.get_device().first_depth_sensor()
depth_scale = depth_sensor.get_depth_scale()
print("Depth Scale is: ", depth_scale)
depth_sensor.set_option(rs.option.laser_power, 360)  # Max Laser power

# We will be removing the background of objects more than
#  clipping_distance_in_meters meters away
clipping_distance_in_meters = 6  # 6 meters
clipping_distance = clipping_distance_in_meters / depth_scale

# Create an align object
# rs.align allows us to perform alignment of depth frames to others frames
# The "align_to" is the stream type to which we plan to align depth frames.
align_to = rs.stream.color
align = rs.align(align_to)

hole_filling = rs.hole_filling_filter(1)



try:
    while True:
        # Get frameset of color and depth
        frames = pipeline.wait_for_frames()
        # frames.get_depth_frame() is a 640x360 depth image

        # Align the depth frame to color frame
        aligned_frames = align.process(frames)

        # Get aligned frames
        depth_frame = aligned_frames.get_depth_frame()  # aligned_depth_frame is a 640x480 depth image
        color_frame = aligned_frames.get_color_frame()

        # Validate that both frames are valid
        if not depth_frame or not color_frame:
            continue

        filled_depth = hole_filling.process(depth_frame)

        aligned_depth_intrinsics = rs.video_stream_profile(depth_frame.profile).get_intrinsics()

        depth_image = np.asanyarray(depth_frame.get_data())
        filled_depth_image = np.asanyarray(filled_depth.get_data())
        color_image = np.asanyarray(color_frame.get_data())

        # Remove background - Set pixels further than clipping_distance to grey
        back_color = 0
        depth_image_3d = np.dstack((depth_image, depth_image, depth_image))  # depth image is 1 channel, color is 3 channels
        bg_removed = np.where((depth_image_3d > clipping_distance) | (depth_image_3d <= 0), back_color, color_image)

        # Render images:
        #   depth align to color on left
        #   depth on right
        depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.03), cv2.COLORMAP_HSV)

        depth_gray = cv2.convertScaleAbs(filled_depth_image, alpha=0.05)
        edge = cv2.Canny(depth_gray, 10, 200, apertureSize=5, L2gradient=False)  # Use the depth to determine the edges between objects
        cv2.imshow('depth edges', cv2.bitwise_or(depth_gray, edge))
        edge = cv2.dilate(edge, None, iterations=1)
        edge = cv2.bitwise_not(edge)  # Invert the edges so that we can use it as a mask

        color_image = cv2.imread("C:/Users/varun/Pictures/Camera Roll/WIN_20230108_15_07_21_Pro.jpg")

        blurred = cv2.GaussianBlur(color_image, (11, 11), 0)
        # blurred = cv2.bitwise_and(blurred, blurred, mask=edge)  # ensure that there is at least a 1 pixel gap between objects of different depths

        cv2.imshow("Blured Masked", blurred)

        img_hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)
        h_min_1 = 25 * 255 / 360
        h_max_1 = 45 * 255 / 360
        s_min_1 = 50 * 255 / 100
        s_max_1 = 100 * 255 / 100
        v_min_1 = 20 * 255 / 100
        v_max_1 = 100 * 255 / 100

        min_1 = np.array([h_min_1, s_min_1, v_min_1], np.uint8)
        max_1 = np.array([h_max_1, s_max_1, v_max_1], np.uint8)


        h_min_2 = 30 * 255 / 360
        h_max_2 = 75 * 255 / 360
        s_min_2 = 0 * 255 / 100
        s_max_2 = 50 * 255 / 100
        v_min_2 = 90 * 255 / 100
        v_max_2 = 100 * 255 / 100

        min_2 = np.array([h_min_2, s_min_2, v_min_2], np.uint8)
        max_2 = np.array([h_max_2, s_max_2, v_max_2], np.uint8)

        img_threshold_yellow_1 = cv2.inRange(img_hsv, min_1, max_1)
        img_threshold_yellow_2 = cv2.inRange(img_hsv, min_2, max_2)

        # Merge the mask and crop the red regions
        img_threshold_yellow = cv2.bitwise_or(img_threshold_yellow_1, img_threshold_yellow_2)
        img_threshold_yellow = cv2.erode(img_threshold_yellow, None, iterations=2)
        img_threshold_yellow = cv2.dilate(img_threshold_yellow, None, iterations=4)
        img_threshold_yellow = cv2.erode(img_threshold_yellow, None, iterations=2)


        contours, hierarchy = cv2.findContours(img_threshold_yellow, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        cv2.drawContours(img_threshold_yellow, contours, -1, (0,255,0), 3)
        cv2.drawContours(color_image, contours, -1, (0,255,0), 3)


        latency = time.time() * 1000 - frames.get_timestamp()
        latency_entry.setValue([latency])
        #print("latency: {}".format(latency))

        #NetworkTables.flush()

        for cnt in contours:
            area = cv2.contourArea(cnt)
            if area > 5000:
                cnt = cv2.convexHull(cnt)
                M = cv2.moments(cnt)
                cx = int(M['m10'] / M['m00'])
                cy = int(M['m01'] / M['m00'])
                cv2.circle(color_image, (cx, cy), 7, (255, 255, 255), -1)
                cv2.putText(color_image, "center", (cx - 20, cy - 20),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
                
                # Draw the semi-major and semi-minor axes
                ellipse = cv2.fitEllipse(cnt)
                cv2.ellipse(color_image, ellipse, (0, 255, 0), 2)
                

                # Draw the minimum area rectangle
                rect = cv2.minAreaRect(cnt)
                box = cv2.boxPoints(rect)
                box = np.int0(box)
                cv2.drawContours(color_image, [box], 0, (0, 0, 255), 2)


                        
                




        cv2.imshow('Contours', color_image)
        cv2.imshow('depth', depth_colormap)
        cv2.imshow('img_threshold_yellow', img_threshold_yellow)
        # cv2.namedWindow('RealSense', cv2.WINDOW_AUTOSIZE)
        # cv2.imshow('RealSense', images)
        cv2.waitKey(1)



finally:

    # Stop streaming
    # pipeline.stop()
    pass