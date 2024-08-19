#!/usr/bin/env python3


import robot_move_server
from jointControl.srv import RobotMove
import threading
import math
import rospy
from cv_bridge import CvBridge
import datetime
from sensor_msgs.msg import Image, PointCloud2
import geometry_msgs.msg
import numpy as np
import ros_numpy
import sys
import cv2
import apriltag
import tf
import tf2_ros
import tf2_geometry_msgs
import moveit_commander
import moveit_msgs.msg
from vector_to_rpy import Transformation as trans
import torch
# Constants
CAM_TOPIC   = "/zedm/zed_node/rgb_raw/image_raw_color"
DEPTH_TOPIC = "/zedm/zed_node/point_cloud/cloud_registered"
MODEL_PATH = "last.pt"
CAPTURE_INTERVAL = datetime.timedelta(seconds=0.3)
DETECTION_CLASSES = [
    "person",
    "bicycle",
    "car",
    "motorcycle",
    "airplane",
    "bus",
    "train",
    "truck",
    "boat",
    "traffic light",
    "fire hydrant",
    "stop sign",
    "parking meter",
    "bench",
    "bird",
    "cat",
    "dog",
    "horse",
    "sheep",
    "cow",
    "elephant",
    "bear",
    "zebra",
    "giraffe",
    "backpack",
    "umbrella",
    "handbag",
    "tie",
    "suitcase",
    "frisbee",
    "skis",
    "snowboard",
    "sports ball",
    "kite",
    "baseball bat",
    "baseball glove",
    "skateboard",
    "surfboard",
    "tennis racket",
    "bottle",
    "wine glass",
    "cup",
    "fork",
    "knife",
    "spoon",
    "bowl",
    "banana",
    "apple",
    "sandwich",
    "orange",
    "broccoli",
    "carrot",
    "hot dog",
    "pizza",
    "donut",
    "cake",
    "chair",
    "couch",
    "potted plant",
    "bed",
    "dining table",
    "toilet",
    "tv",
    "laptop",
    "mouse",
    "remote",
    "keyboard",
    "cell phone",
    "microwave",
    "oven",
    "toaster",
    "sink",
    "refrigerator",
    "book",
    "clock",
    "vase",
    "scissors",
    "teddy bear",
    "hair drier",
    "toothbrush"
]
class Perception:
    # def __init__(self, model):
    def __init__(self):

        self.xyz_array, self.pose_target = None, None
        self.flag = False
        # self.last_capture_time = None
        self.bridge = CvBridge()
        self.detector = apriltag.Detector()
        # self.model = model

        # Initialize TF Buffer and Listener
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        self.broadcaster = tf2_ros.TransformBroadcaster()
        rate = rospy.Rate(10)  # 10 Hz

        rospy.Subscriber(DEPTH_TOPIC, PointCloud2, self.pc_data_callback)
        rospy.Subscriber(CAM_TOPIC, Image, self.cam_callback)

        self.move_group = moveit_commander.MoveGroupCommander("manipulator")

        thread_main = threading.Thread(target=self.thread_main_sub, name='thread_main')

        self.define_safety_zone()
        
        thread_main.start() 

    def thread_main_sub(self):
        
        while True:
            rospy.wait_for_service('robot_move')

            try:
                if (self.center_x is not None) and (self.center_y is not None):
                    center_x_zed, center_y_zed, center_z_zed, err0 = self.get_3d_position(self.center_x, self.center_y)
                    self.corners_in_3d = np.zeros((4,3))
                    self.corners_in_3d[0][0], self.corners_in_3d[0][1], self.corners_in_3d[0][2], err1 = self.get_3d_position(self.corners[0][0], self.corners[0][1])
                    self.corners_in_3d[1][0], self.corners_in_3d[1][1], self.corners_in_3d[1][2], err2 = self.get_3d_position(self.corners[0][0], self.corners[0][1])
                    self.corners_in_3d[2][0], self.corners_in_3d[2][1], self.corners_in_3d[2][2], err3 = self.get_3d_position(self.corners[0][0], self.corners[0][1])
                    self.corners_in_3d[3][0], self.corners_in_3d[3][1], self.corners_in_3d[3][2], err4 = self.get_3d_position(self.corners[0][0], self.corners[0][1])
                    err = err0 or err1 or err2 or err3 or err4
                    if not err:
                        self.broadcast_tag_frame(center_x_zed, center_y_zed, center_z_zed)
                        self.tag_pose_to_target_pose(center_x_zed, center_y_zed, center_z_zed)
                        if not self.flag:
                            self.flag = input("Apriltag e gitmek için y yaziniz:") =="y"
                        else:          
                            if self.robot_move_client(self.pose_target):
                                rospy.loginfo("Robot is at target position")
                            else:
                                rospy.logwarn("Failed to move the robot to the target pose.")
            except Exception as e:
                print(e)
    
    def robot_move_client(self, target_pose):
        try:
            if not self.robot_at_target_pos() :
                move_robot = rospy.ServiceProxy('robot_move', RobotMove)
                response = move_robot(target_pose)
                self.flag = False
                return response.success
            else:
                self.flag = False
                return True
        except rospy.ServiceException as e:
            rospy.logerr("Service call failed: %s", e)
            return False             
    
    def pc_data_callback(self, msg: PointCloud2):
        try:
            pc_data = ros_numpy.point_cloud2.pointcloud2_to_array(msg)
            self.xyz_array = np.nan_to_num(
                np.stack([pc_data["x"], pc_data["y"], pc_data["z"]], axis=-1), nan=-1.0
            )
            # if self.xyz_array is not None:
            #     rospy.loginfo("PointCloud2 data successfully converted")
            if self.xyz_array is None:
                rospy.logerr("Conversion of PointCloud2 data resulted in None")
            
        except Exception as e:
            rospy.logerr(f"Error converting PointCloud2 data: {e}")
            self.xyz_array = None
                    
    def cam_callback(self, msg: Image):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            gray_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
            # current_time = datetime.datetime.now()
            # if (
            #     self.last_capture_time is None
            #     or (current_time - self.last_capture_time) >= CAPTURE_INTERVAL
            # ):
            #     detect_img = self.detect_image(cv_image)
            #     self.last_capture_time = current_time
            
            if not self.detect_tag(cv_image,gray_image):
                cv2.circle(cv_image, (self.center_x, self.center_y), 2, (0, 255, 0), 2)
                for corner in self.corners:
                    cv2.circle(cv_image, (corner[0], corner[1]), 2, (0, 255, 0), 2)
                    
            cv2.imshow("Apriltag", cv_image)
            cv2.waitKey(1)
        except Exception as e:
            rospy.logerr(f"Error in cam_callback: {e}")  
            
    def detect_tag(self,cv_image,gray_image):
        try:
            self.center_x, self.center_y, self.corners = None, None, None
            
            detections = self.detector.detect(gray_image)
            if len(detections) is not 0:
                for detection in detections:
                    center = detection.center
                    corners = detection.corners
                self.corners = [(int(corner[0]), int(corner[1])) for corner in corners]
                self.center_x, self.center_y = int(center[0]), int(center[1])
                
            err = self.center_x is None or self.center_y is None or self.corners is None
            return err
        except:
            pass
            
    def get_3d_position(self, center_x, center_y):
        if self.xyz_array is not None :
            height, width, _ = self.xyz_array.shape
            
            if 0 <= center_x < width and 0 <= center_y < height:
                center_x_zed, center_y_zed, center_z_zed = self.xyz_array[center_y, center_x]
                
                if center_x_zed == -1.0 or center_y_zed == -1.0 or center_z_zed == -1.0:
                    rospy.logwarn("3D position contains invalid value")
                err =  center_x_zed == -1.0 or center_y_zed == -1.0 or center_z_zed == -1.0
                return center_x_zed, center_y_zed, center_z_zed, err
            else:
                rospy.logerr(
                    f"Center coordinates out of bounds: center_x={center_x}, center_y={center_y}, width={width}, height={height}"
                )
                raise IndexError("Center coordinates out of bounds")
        else:
            raise ValueError("XYZ array is None")
        
    def broadcast_tag_frame(self,center_x_zed, center_y_zed, center_z_zed):
        try:  
            
            t = geometry_msgs.msg.TransformStamped()
            t.header.stamp = rospy.Time.now()
            t.header.frame_id = "zedm_base_link"
            t.child_frame_id = "april_tag"
            t.transform.translation.x = center_x_zed
            t.transform.translation.y = -center_z_zed
            t.transform.translation.z = center_y_zed
            t.transform.rotation.x = 0.0
            t.transform.rotation.y = 0.0
            t.transform.rotation.z = 0.0
            t.transform.rotation.w = 1.0
            
            self.broadcaster.sendTransform(t)
            
            self.transform = self.tf_buffer.lookup_transform('base_link', 'april_tag', rospy.Time(0), rospy.Duration(2.0))
            
        except Exception as  e:
            rospy.logerr(f"Error broadcasting tag frame: {e}")

    def tag_pose_to_target_pose(self, center_x_zed, center_y_zed, center_z_zed):
        try:
            origin_april_tag = geometry_msgs.msg.PointStamped()
            origin_april_tag.point.x = 0
            origin_april_tag.point.y = 0
            origin_april_tag.point.z = 0

            origin_tag_in_base_link = tf2_geometry_msgs.do_transform_point(origin_april_tag, self.transform)
            # Set the target pose for MoveIt
            self.pose_target = geometry_msgs.msg.Pose()
            self.pose_target.position.x = origin_tag_in_base_link.point.x
            self.pose_target.position.y = origin_tag_in_base_link.point.y
            self.pose_target.position.z = origin_tag_in_base_link.point.z + 0.4
            
            quaternion = tf.transformations.quaternion_from_euler(3.14,0, 1.57)  # (Roll, Pitch, Yaw) = (0, 90°, 0)

            self.pose_target.orientation.x = quaternion[0]
            self.pose_target.orientation.y = quaternion[1]
            self.pose_target.orientation.z = quaternion[2]
            self.pose_target.orientation.w = quaternion[3]
        
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            rospy.logerr(f"Error transforming pose: {e}")
        
    def robot_at_target_pos(self):
        
        if self.pose_target is None:
            return False
        else:
            current_pose = self.move_group.get_current_pose().pose
            xdiff = abs(current_pose.position.x - self.pose_target.position.x)
            ydiff = abs(current_pose.position.y - self.pose_target.position.y)
            zdiff = abs(current_pose.position.z - self.pose_target.position.z)

        return not(math.sqrt(xdiff**2 + ydiff**2 + zdiff**2) > 0.02)
    
    def define_safety_zone(self):
        scene = moveit_commander.PlanningSceneInterface()
        # Güvenlik bölgesi için pose tanımlayın
        safety_zone_pose = geometry_msgs.msg.PoseStamped()
        safety_zone_pose.header.frame_id = "base_link"
        safety_zone_pose.pose.position.x = 0.0
        safety_zone_pose.pose.position.y = 0.0
        safety_zone_pose.pose.position.z = 0.6
        safety_zone_pose.pose.orientation.w = 1.0
        
        # Güvenlik bölgesi için pose tanımlayın
        safety_zone_pose2 = geometry_msgs.msg.PoseStamped()
        safety_zone_pose2.header.frame_id = "base_link"
        safety_zone_pose2.pose.position.x = 0.0
        safety_zone_pose2.pose.position.y = 0.0
        safety_zone_pose2.pose.position.z = 0
        safety_zone_pose2.pose.orientation.w = 1.0
        
        # Güvenlik bölgesi için pose tanımlayın
        safety_zone_posex = geometry_msgs.msg.PoseStamped()
        safety_zone_posex.header.frame_id = "base_link"
        safety_zone_posex.pose.position.x = 0.5
        safety_zone_posex.pose.position.y = 0.0
        safety_zone_posex.pose.position.z = 0
        safety_zone_posex.pose.orientation.w = 1.0
        
        # Güvenlik bölgesi için pose tanımlayın
        safety_zone_posey = geometry_msgs.msg.PoseStamped()
        safety_zone_posey.header.frame_id = "base_link"
        safety_zone_posey.pose.position.x = 0
        safety_zone_posey.pose.position.y = 0.5
        safety_zone_posey.pose.position.z = 0
        safety_zone_posey.pose.orientation.w = 1.0
        # Güvenlik bölgesini oluşturun
        scene.add_box("safety_zone", safety_zone_pose, size=(2, 2, 0.02))
        scene.add_box("safety_zone2", safety_zone_pose2, size=(2, 2, 0.02))
        scene.add_box("safety_zonex", safety_zone_posex, size=(0.02, 2, 2))
        scene.add_box("safety_zoney", safety_zone_posey, size=(2, 0.02, 2))

    # def detect_image(self, cv_image):
    #     results = self.model([cv_image], size=640)
    #     detected_image = cv_image.copy()
    #     self.detections = []

    #     for pred in results.pred[0]:
    #         x1, y1, x2, y2, conf, cls = list(map(int, pred[:4])) + [
    #             pred[4],
    #             int(pred[5]),
    #         ]
    #         center_x, center_y = (x1 + x2) // 2, (y1 + y2) // 2
    #         object_name = DETECTION_CLASSES[cls]

    #         cv2.rectangle(detected_image, (x1, y1), (x2, y2), (0, 0, 255), 2)
    #         label = f"{object_name}, {conf:.2f}"
    #         cv2.putText(
    #             detected_image,
    #             label,
    #             (x1, y1 - 10),
    #             cv2.FONT_HERSHEY_SIMPLEX,
    #             0.5,
    #             (0, 0, 255),
    #             1,
    #         )

    #         detection = {
    #             "center": (center_x, center_y),
    #             "size_x": abs(x2 - x1),
    #             "size_y": abs(y2 - y1),
    #             "class": object_name,
    #             "probability": conf,
    #             "3d_position": None,
    #         }
    #         self.detections.append(detection)
    #         self.show_image(detected_image)
    #     return detected_image        
    
    # def show_image(self, cv_image):
    #     cv2.imshow("yolo", cv_image)
    #     cv2.waitKey(1)
        


if __name__ == "__main__":
    rospy.init_node("zed_apriltag")
    # model = torch.hub.load("ultralytics/yolov5", "yolov5s")  # or yolov5n - yolov5x6, custom
    # perception = Perception(model=model)   
    perception = Perception()   
    rospy.spin()
    
