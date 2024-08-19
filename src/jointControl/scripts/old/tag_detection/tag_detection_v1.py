#!/usr/bin/env python3

from visualization_msgs.msg import Marker
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
ZEDM_BASE = "zedm_base_link"
TAG_BASE = "april_tag"
TARGET_POSE = "target_pose"
BASE_LINK = "base_link"

class Perception:
    # def __init__(self, model):
    def __init__(self):
        self.xyz_array, self.pose_target = None, None
        self.flag = False
        self.bridge = CvBridge()
        self.detector = apriltag.Detector()

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
                    self.center_x_zed, self.center_y_zed, self.center_z_zed, err0 = self.get_3d_position(self.center_x, self.center_y)
                    self.corners_in_zed = np.zeros((4,3))
                    errors = []

                    for i in range(3):
                        self.corners_in_zed[i][0], self.corners_in_zed[i][1], self.corners_in_zed[i][2], err = self.get_3d_position(self.corners[i][0], self.corners[i][1])
                        errors.append(err)
       
                    err = any(errors)
                    if not err and not self.check_tag_center(self.corners_in_zed):
                        
                        tag_x_axis, tag_y_axis, tag_z_axis= self.get_orientation(self.corners_in_zed)
                        R= trans.rotation_matrix_from_vectors(vec1 = np.array((0,0,1)),vec2=tag_z_axis)
                        roll, pitch, yaw = trans.r_to_rpy(R)
                        
                        self.broadcast_tag_frame(self.center_x_zed, self.center_y_zed, self.center_z_zed,roll, pitch, yaw,ZEDM_BASE, TAG_BASE)

                        for i in range(3):
                            self.broadcast_tag_frame(self.corners_in_zed[i][0], self.corners_in_zed[i][1], self.corners_in_zed[i][2],roll, pitch, yaw,ZEDM_BASE, "corner: "+str(i))
                        self.tag_pose_to_target_pose()
                        if not self.flag:
                            current_pose = self.move_group.get_current_pose().pose
                            print(current_pose)
                            print(self.target_pos_in_base_link.pose)
                            self.flag = input("Apriltag e gitmek için y yaziniz:") =="y"
                        else:          
                            if self.robot_move_client(self.target_pos_in_base_link.pose):
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
            if self.xyz_array is None:
                rospy.logerr("Conversion of PointCloud2 data resulted in None")
            
        except Exception as e:
            rospy.logerr(f"Error converting PointCloud2 data: {e}")
            self.xyz_array = None
                    
    def cam_callback(self, msg: Image):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            gray_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
            
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
            
    def get_3d_position(self, x, y):
        if self.xyz_array is not None:
            height, width, _ = self.xyz_array.shape

            if 0 <= x < width and 0 <= y < height:
                sum_x_zed = sum_y_zed = sum_z_zed = 0.0
                valid_neighbors = 0

                for i in range(-1, 2):
                    for j in range(-1, 2):
                        nx, ny = x + i, y + j
                        
                        if 0 <= nx < width and 0 <= ny < height:
                            x_zed, y_zed, z_zed = self.xyz_array[ny, nx]
                            if x_zed != -1.0 and y_zed != -1.0 and z_zed != -1.0:
                                sum_x_zed += x_zed
                                sum_y_zed += y_zed
                                sum_z_zed += z_zed
                                valid_neighbors += 1

                if valid_neighbors > 0:
                    avg_x = sum_x_zed / valid_neighbors
                    avg_y = sum_y_zed / valid_neighbors
                    avg_z = sum_z_zed / valid_neighbors
                    err = False
                else:
                    rospy.logwarn("No valid neighbors found, returning invalid position")
                    avg_x, avg_y, avg_z = -1.0, -1.0, -1.0
                    err = True

                return avg_x, avg_y, avg_z, err
            else:
                rospy.logerr(
                    f"Coordinates out of bounds: X={x}, Y={y}, width={width}, height={height}"
                )
                raise IndexError("Center coordinates out of bounds")
        else:
            raise ValueError("XYZ array is None")
    
    def get_orientation(self,corners):
        
        y_axis = corners[0]-corners[1]
        x_axis = corners[0]-corners[2]
        x_axis = x_axis / (np.linalg.norm(x_axis)*5)
        y_axis = y_axis / (np.linalg.norm(y_axis)*5)
        z_axis = np.cross(x_axis,y_axis)
        
        return x_axis, y_axis, z_axis
        
    def broadcast_tag_frame(self,x_zed, y_zed, z_zed, roll, pitch, yaw, src_frame, child_frame):
        try:  

            q=tf.transformations.quaternion_from_euler(roll,yaw,pitch)
            
            t = geometry_msgs.msg.TransformStamped()
            t.header.stamp = rospy.Time.now()
            t.header.frame_id = src_frame
            t.child_frame_id = child_frame
            t.transform.translation.x = x_zed
            t.transform.translation.y = -z_zed
            t.transform.translation.z = y_zed
            
            t.transform.rotation.x = q[0]
            t.transform.rotation.y = q[1]
            t.transform.rotation.z = q[2]
            t.transform.rotation.w = q[3]
            
            self.broadcaster.sendTransform(t)
            
            
        except Exception as  e:
            rospy.logerr(f"Error broadcasting tag frame: {e}")

    def tag_pose_to_target_pose(self):
        try:
            # PoseStamped mesajını oluşturun
            self.pose_target = geometry_msgs.msg.PoseStamped()

            # Header bilgilerini tanımlama
            self.pose_target.header.stamp = rospy.Time.now()  # Şu anki zamanı kullanın
            self.pose_target.header.frame_id = TAG_BASE    # Frame ID'yi tanımlayın

            # Pozisyon bilgilerini tanımlama
            self.pose_target.pose.position.x = 0
            self.pose_target.pose.position.y = 0
            self.pose_target.pose.position.z = 0.3
            
            # Oryantasyon bilgilerini tanımlama (quaternion formatında)
            roll, pitch, yaw = 0, 1.57, 1.57

            q = tf.transformations.quaternion_from_euler(roll, pitch, yaw)
            self.pose_target.pose.orientation.x = q[0]
            self.pose_target.pose.orientation.y = q[1]
            self.pose_target.pose.orientation.z = q[2]
            self.pose_target.pose.orientation.w = q[3]

            self.broadcast_tag_frame(
                self.pose_target.pose.position.x, 
                self.pose_target.pose.position.y , 
                self.pose_target.pose.position.z, 
                roll, pitch, yaw, TAG_BASE, TARGET_POSE)

            self.transform = self.tf_buffer.lookup_transform(
                BASE_LINK, TAG_BASE, rospy.Time(0), rospy.Duration(5.0))
            
            self.target_pos_in_base_link = tf2_geometry_msgs.do_transform_pose(
                self.pose_target, self.transform)

            # self.broadcaster.sendTransform(self.pose_target)
        
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            rospy.logerr(f"Error transforming pose: {e}")
        
    def robot_at_target_pos(self):
        
        if self.target_pos_in_base_link is None:
            return False
        else:
            current_pose = self.move_group.get_current_pose().pose
            xdiff = abs(current_pose.position.x - self.target_pos_in_base_link.position.x)
            ydiff = abs(current_pose.position.y - self.target_pos_in_base_link.position.y)
            zdiff = abs(current_pose.position.z - self.target_pos_in_base_link.position.z)
        return math.sqrt(xdiff**2 + ydiff**2 + zdiff**2) <= 0.02
    
    def check_tag_center(self, corners):
        avg_two_corners =  (corners[1] + corners[2] )/2 
        xdiff = abs(self.center_x_zed - avg_two_corners[0])
        ydiff = abs(self.center_y_zed - avg_two_corners[1])
        zdiff = abs(self.center_z_zed - avg_two_corners[2])
        
        return  math.sqrt(xdiff**2+ydiff**2+zdiff**2) <= 0.02
            
    
    def add_box(self, box_name, position, size, frame_id=BASE_LINK):
        scene = moveit_commander.PlanningSceneInterface()
        
        pose = geometry_msgs.msg.PoseStamped()
        pose.header.frame_id = frame_id
        pose.pose.position.x = position[0]
        pose.pose.position.y = position[1]
        pose.pose.position.z = position[2]
        pose.pose.orientation.w = 1.0
        
        scene.add_box(box_name, pose, size=size)
    
    def define_safety_zone(self,corners):
        # Güvenlik bölgeleri için parametreler (isim, pozisyon, boyut)
        boxes = [
            ("safety_zone", [0.0, 0.0, 0.6], [2, 2, 0.02]),
            ("safety_zone2", [0.0, 0.0, 0.0], [2, 2, 0.02]),
            ("safety_zonex", [0.5, 0.0, 0.0], [0.02, 2, 2]),
            ("safety_zoney", [0.0, 0.5, 0.0], [2, 0.02, 2])
        ]

        # Her güvenlik bölgesini oluştur
        for box_name, position, size in boxes:
            self.add_box(box_name, position, size)


if __name__ == "__main__":
    rospy.init_node("zed_apriltag")
    perception = Perception()   
    rospy.spin()
    
