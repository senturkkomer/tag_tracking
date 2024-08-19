#!/usr/bin/env python3
import rospy
import cv2
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
import apriltag

CAM_TOPIC = "/depth_camera/depth_camera_rgb/image_raw"
DEPTH_TOPIC = "/zedm/zed_node/point_cloud/cloud_registered"
CAPTURE_INTERVAL = datetime.timedelta(seconds=0.3)

class Perception:
    def _init_(self):


        rospy.Subscriber(CAM_TOPIC, Image, self.cam_callback)
        rospy.Subscriber(DEPTH_TOPIC, PointCloud2, self.pc_data_callback)


        self.show_screen = True
        self.bridge = CvBridge()
        self.last_capture_time = None

        self.detections = []
        self.xyz_array = None

    def cam_callback(self, msg: Image):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            current_time = datetime.datetime.now()
            if (
                self.last_capture_time is None
                or (current_time - self.last_capture_time) >= CAPTURE_INTERVAL
            ):
                detect_img = self.detect_image(cv_image)
                self.last_capture_time = current_time
                self.show_image(detect_img)
        except Exception as e:
            rospy.logerr(f"Error in cam_callback: {e}")

    # def car_pose_callback(self, msg: EstimatedPose):
    #     self.car_x = msg.x
    #     self.car_y = msg.y
    #     self.car_theta = msg.heading

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

    def detect_image(self, cv_image):
        results = self.model([cv_image], size=640)
        detected_image = cv_image.copy()
        self.detections = []

        for pred in results.pred[0]:
            x1, y1, x2, y2, conf, cls = list(map(int, pred[:4])) + [
                pred[4],
                int(pred[5]),
            ]
            center_x, center_y = (x1 + x2) // 2, (y1 + y2) // 2
            object_name = DETECTION_CLASSES[cls]

            cv2.rectangle(detected_image, (x1, y1), (x2, y2), (0, 0, 255), 2)
            label = f"{object_name}, {conf:.2f}"
            cv2.putText(
                detected_image,
                label,
                (x1, y1 - 10),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.5,
                (0, 0, 255),
                1,
            )

            detection = {
                "center": (center_x, center_y),
                "size_x": abs(x2 - x1),
                "size_y": abs(y2 - y1),
                "class": object_name,
                "probability": conf,
                "3d_position": None,
            }
            self.detections.append(detection)

        self.update_detections_with_3d_positions()
        return detected_image

    def update_detections_with_3d_positions(self):
        if self.xyz_array is not None:
            for detection in self.detections:
                center_x, center_y = detection["center"]
                zed_x, zed_y, zed_z = self.get_3d_position(center_x, center_y)
                detection["3d_position"] = (zed_x, zed_y, zed_z)

                if zed_x != -1.0 and zed_y != -1.0 and zed_z != -1.0:
                    pose_msg = PoseStamped()
                    pose_msg.header.stamp = rospy.Time.now()
                    pose_msg.header.frame_id = detection["class"]
                    pose_msg.pose.position.x = zed_z
                    pose_msg.pose.position.y = zed_x
                    pose_msg.pose.position.z = zed_y
                    self.object_position_publisher.publish(pose_msg)

    def get_3d_position(self, center_x, center_y):
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

    def show_image(self, cv_image):
        if self.show_screen:
            cv2.imshow("ylo", cv_image)
            cv2.waitKey(1)


rospy.init_node("YOLOv5_node")

init = sl.InitParameters(depth_mode=sl.DEPTH_MODE.ULTRA,
                        coordinate_units=sl.UNIT.METER,
                        coordinate_system=sl.COORDINATE_SYSTEM.RIGHT_HANDED_Y_UP)
zed = sl.Camera()
status = zed.open(init)
if status != sl.ERROR_CODE.SUCCESS:
    print(repr(status))
    exit()
runtime = sl.RuntimeParameters()
mat_image = sl.Mat()
mat_depth = sl.Mat()
detector = apriltag.Detector()

while True:
    if zed.grab(runtime) == sl.ERROR_CODE.SUCCESS:
        zed.retrieve_image(mat_image, sl.VIEW.LEFT)
        zed.retrieve_measure(mat_depth, sl.MEASURE.DEPTH)
        
        image_ocv = mat_image.get_data()
        gray_image = cv2.cvtColor(image_ocv, cv2.COLOR_BGR2GRAY)
        detections = detector.detect(gray_image)

        for detection in detections:
            center = detection.center
            center_x, center_y = int(center[0]), int(center[1])

perception = Perception()
rospy.spin()
