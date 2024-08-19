#!/usr/bin/env python3

import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import PointCloud2
import numpy as np
import ros_numpy
import cv2
import apriltag
import pyzed.sl as sl

# Constants
CAM_TOPIC = "/depth_camera/depth_camera_rgb/image_raw"
DEPTH_TOPIC = "/zedm/zed_node/point_cloud/cloud_registered"
POSE_TOPIC = "/ekf/estimated_pose"

class Perception:
    def __init__(self):
        self.initialize_zed_camera()
        self.xyz_array = None
        rospy.Subscriber(DEPTH_TOPIC, PointCloud2, self.pc_data_callback)
        self.bridge = CvBridge()

    def initialize_zed_camera(self):
        init = sl.InitParameters(depth_mode=sl.DEPTH_MODE.ULTRA,
                                 coordinate_units=sl.UNIT.METER,
                                 coordinate_system=sl.COORDINATE_SYSTEM.RIGHT_HANDED_Y_UP)
        
        self.zed = sl.Camera()
        status = self.zed.open(init)
        if status != sl.ERROR_CODE.SUCCESS:
            print(repr(status))
            exit()

        self.runtime = sl.RuntimeParameters()
        self.mat_image = sl.Mat()
        self.mat_depth = sl.Mat()
        self.detector = apriltag.Detector()

        rospy.Timer(rospy.Duration(0.1), self.process_zed_camera)

    def process_zed_camera(self, event):
        if self.zed.grab(self.runtime) == sl.ERROR_CODE.SUCCESS:
            self.zed.retrieve_image(self.mat_image, sl.VIEW.LEFT)
            self.zed.retrieve_measure(self.mat_depth, sl.MEASURE.DEPTH)
            
            image_ocv = self.mat_image.get_data()
            gray_image = cv2.cvtColor(image_ocv, cv2.COLOR_BGR2GRAY)
            detections = self.detector.detect(gray_image)
            
            for detection in detections:
                center = detection.center
                center_x, center_y = int(center[0]), int(center[1])
                if self.xyz_array is not None:
                    zed_x, zed_y, zed_z = self.get_3d_position(center_x, center_y)
                    print(f"AprilTag detected at ({zed_x}, {zed_y}, {zed_z})")

                    cv2.circle(image_ocv, (center_x, center_y), 2, (0, 255, 0), 2)
                    cv2.putText(image_ocv, f"{zed_z:.2f}m", (center_x + 5, center_y+5), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
                else:
                    rospy.logwarn("Skipping detection because XYZ array is None")

            cv2.imshow("Image", image_ocv)
            if cv2.waitKey(1) & 0xFF == 27:  # Press 'Esc' to exit
                rospy.signal_shutdown("User exit")

    def pc_data_callback(self, msg: PointCloud2):
        try:
            pc_data = ros_numpy.numpify(msg)
            print(pc_data['x'])
            print(pc_data['y'])
            print(pc_data['z'])
            self.xyz_array = np.nan_to_num(
                np.stack([pc_data['x'], pc_data['y'], pc_data['z']], axis=-1), nan=-1.0
            )
            rospy.loginfo("PointCloud2 data successfully converted")
        except Exception as e:
            rospy.logerr(f"Error converting PointCloud2 data: {e}")
            self.xyz_array = None

    def get_3d_position(self, center_x, center_y):
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

if __name__ == "__main__":
    rospy.init_node("deneme_v2")
    perception = Perception()
    rospy.spin()
