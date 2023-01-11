## License: Apache 2.0. See LICENSE file in root directory.
## Copyright(c) 2015-2017 Intel Corporation. All Rights Reserved.

# Import OpenCV for easy image rendering
import math
import cv2 
# Import Numpy for easy array manipulation
import numpy as np
###############################################
##      Open CV and Numpy integration        ##
###############################################
# First import the library
from math import atan, cos, pi, sin, sqrt, tan
from networktables import NetworkTables
import pyrealsense2 as rs

# Initialize Network Tables
# As a client to connect to a robot
NetworkTables.initialize(server='127.0.0.1')


# # Create a pipeline
# pipeline = rs.pipeline()

# # Create a config and configure the pipeline to stream
# #  different resolutions of color and depth streams
# config = rs.config()

# # Get device product line for setting a supporting resolution
# pipeline_wrapper = rs.pipeline_wrapper(pipeline)
# pipeline_profile = config.resolve(pipeline_wrapper)
# device = pipeline_profile.get_device()
# device_product_line = str(device.get_info(rs.camera_info.product_line))

# found_rgb = False
# for s in device.sensors:
#     if s.get_info(rs.camera_info.name) == 'RGB Camera':
#         found_rgb = True
#         break
# if not found_rgb:
#     print("The demo requires Depth camera with Color sensor")
#     exit(0)

# config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)

# if device_product_line == 'L500':
#     config.enable_stream(rs.stream.color, 960, 540, rs.format.bgr8, 30)
# else:
#     config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

# # Start streaming
# profile = pipeline.start(config)
# color_sensor = profile.get_device().query_sensors()[1]
# color_sensor.set_option(rs.option.enable_auto_exposure, True)
# color_sensor.set_option(rs.option.enable_auto_white_balance, True)
# # color_sensor.set_option(rs.option.exposure, 400)
# color_sensor.set_option(rs.option.white_balance, 3500)

# # Getting the depth sensor's depth scale (see rs-align example for explanation)
# depth_sensor = profile.get_device().first_depth_sensor()
# depth_scale = depth_sensor.get_depth_scale()
# print("Depth Scale is: ", depth_scale)
# depth_sensor.set_option(rs.option.laser_power, 360)  # Max Laser power

# # We will be removing the background of objects more than
# #  clipping_distance_in_meters meters away
# clipping_distance_in_meters = 6  # 6 meters
# clipping_distance = clipping_distance_in_meters / depth_scale

# # Create an align object
# # rs.align allows us to perform alignment of depth frames to others frames
# # The "align_to" is the stream type to which we plan to align depth frames.
# align_to = rs.stream.color
# align = rs.align(align_to)

# hole_filling = rs.hole_filling_filter(1)

cap = cv2.VideoCapture("C:/Users/varun/Pictures/Camera Roll/WIN_20230108_20_16_07_Pro.mp4")


def getSemiMajorAxis(moment):
    E = Ellipse()

    # --- Get the Moments
    E.m00 = moment['m00']
    E.m10 = moment['m10']
    E.m01 = moment['m01']
    E.m11 = moment['m11']
    E.m02 = moment['m02']
    E.m20 = moment['m20']

    # --- Ellipse properties

    # Barycenter
    E.x = E.m10 / E.m00;
    E.y = E.m01 / E.m00;

    # Central moments (intermediary step)
    a = E.m20 / E.m00 - E.x * E.x;
    b = 2 * (E.m11 / E.m00 - E.x * E.y);
    c = E.m02 / E.m00 - E.y * E.y;

    # Orientation (radians)
    E.theta = 1 / 2 * atan(b / (a - c)) + (a < c) * pi / 2;

    # Minor and major axis
    E.w = sqrt(8 * (a + c - sqrt((b * b) + (a - c) * (a - c)))) / 2;
    E.l = sqrt(8 * (a + c + sqrt((b * b) + (a - c) * (a - c)))) / 2;

    # Ellipse focal points
    d = sqrt((E.l * E.l) - (E.w * E.w));
    E.x1 = E.x + d * cos(E.theta);
    E.y1 = E.y + d * sin(E.theta);
    E.x2 = E.x - d * cos(E.theta);
    E.y2 = E.y - d * sin(E.theta);

    # # Ellipse direction
    # if direct:
    #     tmp = [i-mean(i) j-mean(j)]*[cos(E.theta) -sin(E.theta) ; sin(E.theta) cos(E.theta)];
    #     if skewness(tmp(:,1))>0

    #         # Fix direction
    #         E.theta = mod(E.theta + pi, 2*pi);
    #         tmp = [E.x1 E.y1];

    #         # Swap F1 and F2
    #         E.x1 = E.x2;
    #         E.y1 = E.y2;
    #         E.x2 = tmp(1);
    #         E.y2 = tmp(2);
    #     end
    # end

    return E


class Ellipse:
    x = 0
    y = 0
    m00 = 0
    m10 = 0
    m01 = 0
    m11 = 0
    m02 = 0
    m20 = 0
    theta = 0
    w = 0
    l = 0
    x1 = 0
    y1 = 0
    x2 = 0
    y2 = 0
    
    
def getLength(x1, y1, x2, y2):
    return sqrt((x1 - x2) ** 2 + (y1 - y2) ** 2)

try:
    while True:
# # Get frameset of color and depth
#         frames = pipeline.wait_for_frames()
#         # frames.get_depth_frame() is a 640x360 depth image

#         # Align the depth frame to color frame
#         aligned_frames = align.process(frames)

#         # Get aligned frames
#         depth_frame = aligned_frames.get_depth_frame()  # aligned_depth_frame is a 640x480 depth image
#         color_frame = aligned_frames.get_color_frame()

#         # Validate that both frames are valid
#         if not depth_frame or not color_frame:
#             continue

#         filled_depth = hole_filling.process(depth_frame)

#         aligned_depth_intrinsics = rs.video_stream_profile(depth_frame.profile).get_intrinsics()

#         depth_image = np.asanyarray(depth_frame.get_data())
#         filled_depth_image = np.asanyarray(filled_depth.get_data())
#         color_image = np.asanyarray(color_frame.get_data())

#         # Remove background - Set pixels further than clipping_distance to grey
#         back_color = 0
#         depth_image_3d = np.dstack((depth_image, depth_image, depth_image))  # depth image is 1 channel, color is 3 channels
#         bg_removed = np.where((depth_image_3d > clipping_distance) | (depth_image_3d <= 0), back_color, color_image)

#         # Render images:
#         #   depth align to color on left
#         #   depth on right
#         depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.03), cv2.COLORMAP_HSV)

#         depth_gray = cv2.convertScaleAbs(filled_depth_image, alpha=0.05)
#         edge = cv2.Canny(depth_gray, 10, 200, apertureSize=5, L2gradient=False)  # Use the depth to determine the edges between objects
#         cv2.imshow('depth edges', cv2.bitwise_or(depth_gray, edge))
#         edge = cv2.dilate(edge, None, iterations=1)
#         edge = cv2.bitwise_not(edge)  # Invert the edges so that we can use it as a mask

        # color_image = None

        ret, color_image = cap.read()

        # blurred = cv2.GaussianBlur(color_image, (11, 11), 0)
        # blurred = cv2.bitwise_and(blurred, blurred, mask=edge)  # ensure that there is at least a 1 pixel gap between objects of different depths

        cv2.imshow("Blured Masked", color_image)
        time = cv2.getTickCount()
        img_hsv = cv2.cvtColor(color_image, cv2.COLOR_BGR2HSV)
        h_min_1 = 25 * 255 / 360
        h_max_1 = 45 * 255 / 360
        s_min_1 = 50 * 255 / 100
        s_max_1 = 100 * 255 / 100
        v_min_1 = 20 * 255 / 100
        v_max_1 = 100 * 255 / 100

        min_1 = np.array([h_min_1, s_min_1, v_min_1], np.uint8)
        max_1 = np.array([h_max_1, s_max_1, v_max_1], np.uint8)

        h_min_2 = 20 * 255 / 360
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
        #cv2.drawContours(img_threshold_yellow, contours, -1, (0, 255, 0), 3)
        #cv2.drawContours(color_image, contours, -1, (0, 255, 0), -1)

        # print("latency: {}".format(latency))

        # NetworkTables.flush()

        for cnt in contours:
            area = cv2.contourArea(cnt)
            if area > 1000:
                tmp = np.zeros(color_image.shape[:2], dtype="uint8")


                cv2.drawContours(tmp, [cnt], -1, 255, -1)
                cv2.drawContours(color_image, [cnt], -1, (0, 255,0 ), -1)
                M = cv2.moments(tmp)
                cx = int(M['m10'] / M['m00'])
                cy = int(M['m01'] / M['m00'])
                cv2.circle(color_image, (cx, cy), 7, (255, 255, 255), -1)
                cv2.putText(color_image, "center", (cx - 20, cy - 20),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)

                data = getSemiMajorAxis(M)
                shape = tmp.shape

                invalid = True
                reverse = False
            
               
                
                length = 1
                cos_slope = math.cos(data.theta)
                sin_slope = math.sin(data.theta)

                while (invalid):
                    x1 = data.x1 + length * cos_slope
                    y1 = data.y1 + length * sin_slope
                    x2 = data.x2 - length * cos_slope
                    y2 = data.y2 - length * sin_slope
                    
                    if (x1 <= tmp.shape[1] and y1 <= tmp.shape[0] and x1 >= 0 and y1 >= 0 and x2 <= tmp.shape[1] and y2 <= tmp.shape[0] and x2 >= 0 and y2 >= 0):
                        #print("x1: {}, y1: {}, x2: {}, y2: {}".format(x1, y1, x2, y2))
                        if (tmp[int(y1)][int(x1)] == 0):
                            reverse = True
                            invalid = False
                            #print("reversed")
                            break
                        elif (tmp[int(y2)][int(x2)] == 0):
                            reverse = False
                            invalid = False
                           # print("not reversed")
                            break
                    else:
                        #print("invalid")
                        break
                    length += 1

                #print (length)

                color = None
                if (invalid):
                    color = (0, 0, 255)
                else:
                    color = (255, 255, 255)
                # draw a point on the top side
                if reverse:      
                    cv2.circle(color_image, (int(data.x2), int(data.y2)), 7, color, -1)
                else:
                    cv2.circle(color_image, (int(data.x1), int(data.y1)), 7, color, -1)
                cv2.line(color_image, (int(data.x1), int(data.y1)), (int(data.x2), int(data.y2)), color, 2)




                # Draw the minimum area rectangle
                rect = cv2.minAreaRect(cnt)
                box = cv2.boxPoints(rect)
                box = np.int0(box)
                
                squareness = min(data.l, data.w) / max(data.l, data.w)  

                # print ("sqareness: {}".format(squareness))

                minAreaRectColor = None
                if ( squareness > 0.80):
                    minAreaRectColor = (0, 0, 255)
                else:
                    minAreaRectColor = (0, 255, 255)
                cv2.drawContours(color_image, [box], 0, minAreaRectColor, 2)

                


        cv2.imshow('Contours', color_image)
        cv2.imshow('img_threshold_yellow', img_threshold_yellow)
        # cv2.namedWindow('RealSense', cv2.WINDOW_AUTOSIZE)
        # cv2.imshow('RealSense', images)
        print("{} ms".format((cv2.getTickCount() - time)/cv2.getTickFrequency() * 1000))
        cv2.waitKey(30)





finally:

    # Stop streaming
    # pipeline.stop()
    pass
