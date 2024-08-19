#!/usr/bin/env python3

"""
    This sample demonstrates how to capture a live 3D point cloud
    with the ZED SDK and display the result in an OpenGL window.
"""
import rospy
from cv_bridge import CvBridge
import datetime
from sensor_msgs.msg import Image, PointCloud2
from geometry_msgs.msg import PoseStamped
import numpy as np
import ros_numpy
import sys
import ogl_viewer.viewer as gl
import pyzed.sl as sl
import argparse
import cv2
import apriltag



# Constants
CAM_TOPIC   = "/zedm/zed_node/rgb_raw/image_raw_color"
DEPTH_TOPIC = "/zedm/zed_node/point_cloud/cloud_registered"
POSE_TOPIC = "/ekf/estimated_pose"
MODEL_PATH = "last.pt"
CAPTURE_INTERVAL = datetime.timedelta(seconds=0.1)

class Perception:

    def __init__(self):
        
        self.xyz_array = None
        rospy.Subscriber(CAM_TOPIC, Image, self.cam_callback)
        rospy.Subscriber(DEPTH_TOPIC, PointCloud2, self.pc_data_callback)


        self.show_screen = True
        self.bridge = CvBridge()
        self.last_capture_time = None
        
        
        self.center_x,self.center_y=None, None


        self.detector = apriltag.Detector()



    def pc_data_callback(self, msg: PointCloud2):
        try:
        
            pc_data = ros_numpy.point_cloud2.pointcloud2_to_array(msg)
            self.xyz_array = np.nan_to_num(
                np.stack([pc_data["x"], pc_data["y"], pc_data["z"]], axis=-1), nan=-1.0
            )
            if self.xyz_array is not None:
                rospy.loginfo("PointCloud2 data successfully converted")
            else:
                rospy.logerr("Conversion of PointCloud2 data resulted in None")
        except Exception as e:
            rospy.logerr(f"Error converting PointCloud2 data: {e}")
            self.xyz_array = None
    
    def cam_callback(self, msg: Image):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            gray_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
            detections = self.detector.detect(gray_image)

            for detection in detections:
                center = detection.center
                self.center_x, self.center_y = int(center[0]), int(center[1])
                print(f"AprilTag detected at ({self.center_x}, {self.center_y})")
            
            current_time = datetime.datetime.now()
            if (self.last_capture_time is None
                or (current_time - self.last_capture_time) >= CAPTURE_INTERVAL):
                
                self.last_capture_time = current_time
                self.show_image(cv_image, self.center_x, self.center_y)

        except Exception as e:
            rospy.logerr(f"Error in cam_callback: {e}")
    
    def get_3d_position(self, center_x, center_y):
        print(center_x, center_y)
        if self.xyz_array is not None:
            height, width, _ = self.xyz_array.shape
            if 0 <= center_x < width and 0 <= center_y < height:
                zed_x, zed_y, zed_z = self.xyz_array[center_y, center_x]
                if zed_x == -1.0 or zed_y == -1.0 or zed_z == -1.0:
                    rospy.logwarn("3D position contains invalid value")
                return zed_x, zed_y, zed_z
            else:
                rospy.logerr(
                    f"Center coordinates out of bounds: center_x={center_x}, center_y={center_y}, width={width}, height={height}"
                )
                raise IndexError("Center coordinates out of bounds")
        else:
            rospy.logerr("XYZ array is None")
            raise ValueError("XYZ array is None")

    def show_image(self, cv_image, center_x, center_y):
        if self.show_screen:
            zed_x, zed_y, zed_z = self.get_3d_position(center_x, center_y)
            cv2.circle(cv_image, (center_x, center_y), 2, (0, 255, 0), 2)
            cv2.putText(cv_image, f"{zed_x:.2f}m, {zed_y:.2f}m, {zed_z:.2f}m", (center_x + 5, center_y+5), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
            cv2.imshow("Apriltag", cv_image)
            cv2.waitKey(1)

if __name__ == "__main__":
    rospy.init_node("zed_apriltag")
    perception = Perception()
    rospy.spin()
