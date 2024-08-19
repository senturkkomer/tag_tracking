#!/usr/bin/env python3
import robot_move_server
import threading
import math
import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, PointCloud2
import geometry_msgs.msg
import numpy as np
import ros_numpy
import cv2
import apriltag
import tf
import tf2_ros
import moveit_commander
from vector_to_rpy import Transformation as trans

# Constants
CAM_TOPIC   = "/zedm/zed_node/rgb_raw/image_raw_color"
DEPTH_TOPIC = "/zedm/zed_node/point_cloud/cloud_registered"
ZEDM_BASE = "zedm_base_link"
TAG_BASE = "april_tag"
TARGET_POSE_FRAME = "target_pose"
BASE_LINK = "base_link"

class Perception:
    # def __init__(self, model):
    def __init__(self):
        self.xyz_array, self.pose_target, self.tag_detected = None, None, False
        self.bridge = CvBridge()
        self.detector = apriltag.Detector()

        # Initialize TF Buffer and Listener

        self.broadcaster = tf2_ros.TransformBroadcaster()
        rate = rospy.Rate(10)  # 10 Hz

        rospy.Subscriber(DEPTH_TOPIC, PointCloud2, self.pc_data_callback)
        rospy.Subscriber(CAM_TOPIC, Image, self.cam_callback)
        self.define_safety_zone()
        
        self.thread_main_sub() 

    def thread_main_sub(self):
        
        while not rospy.is_shutdown():

            try:
                if self.tag_detected:
                    self.center_x_zed, self.center_y_zed, self.center_z_zed = self.get_3d_position(self.center_x, self.center_y)
                    self.corners_in_zed = np.zeros((4,3))
                    errors = []

                    for i in range(3):
                        self.corners_in_zed[i][0], self.corners_in_zed[i][1], self.corners_in_zed[i][2], err = self.get_3d_position(self.corners[i][0], self.corners[i][1])
                        errors.append(err)
       
                    err = any(errors)
                    if not err and not self.check_tag_center(self.corners_in_zed):
                        
                        tag_x_axis_zed, tag_y_axis_zed, tag_z_axis_zed = self.get_orientation(self.corners_in_zed)
                        Rx_zed_to_tag= trans.rotation_matrix_from_vectors(vec1 = np.array((1,0,0)),vec2=tag_x_axis_zed)
                        Ry_zed_to_tag= trans.rotation_matrix_from_vectors(vec1 = np.array((0,1,0)),vec2=tag_y_axis_zed)
                        Rz_zed_to_tag= trans.rotation_matrix_from_vectors(vec1 = np.array((0,0,1)),vec2=tag_z_axis_zed)
                        
                        roll, pitch, yaw = np.dot(np.dot(Rx_zed_to_tag,Ry_zed_to_tag), Rz_zed_to_tag)
                        
                        
                        self.broadcast_tag_frame(
                            self.center_x_zed, self.center_y_zed, self.center_z_zed, 
                            roll, pitch, yaw, ZEDM_BASE, TAG_BASE)

                        for i in range(3):
                            self.broadcast_tag_frame(
                                self.corners_in_zed[i][0], self.corners_in_zed[i][1], self.corners_in_zed[i][2],
                                roll, pitch, yaw,ZEDM_BASE, "corner: "+str(i))
                            
                        x_target, y_target, z_target = 0, 0, 0.4
                        roll, pitch, yaw = 0, 1.57, 1.57
                        
                        self.broadcast_tag_frame(
                            x_target, y_target, z_target, 
                            roll, pitch, yaw, TAG_BASE, TARGET_POSE_FRAME)


            except Exception as e:
                # rospy.logerr(e)
                pass

    
        
    
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
            
            if not self.detect_tag(gray_image):
                cv2.circle(cv_image, (self.center_x, self.center_y), 2, (0, 255, 0), 2)
                for corner in self.corners:
                    cv2.circle(cv_image, (corner[0], corner[1]), 2, (0, 255, 0), 2)
                    
            cv2.imshow("Apriltag", cv_image)
            cv2.waitKey(1)
        except Exception as e:
            rospy.logerr(f"Error in cam_callback: {e}")  
            
    def detect_tag(self,gray_image):
        try:
            self.center_x, self.center_y, self.corners = None, None, None
            
            detections = self.detector.detect(gray_image)
            if len(detections) is not 0:
                for detection in detections:
                    center = detection.center
                    corners = detection.corners
                self.corners = [(int(corner[0]), int(corner[1])) for corner in corners]
                self.center_x, self.center_y = int(center[0]), int(center[1])
                
            self.tag_detected = not(self.center_x is None or self.center_y is None or self.corners is None)
        except:
            pass
            
    def get_3d_position(self, x, y):
        if self.xyz_array is not None:
            height, width, _ = self.xyz_array.shape

            if 0 <= x < width and 0 <= y < height:
                sum_x_zed = sum_y_zed = sum_z_zed = 0.0
                valid_neighbors = 0

                for ix in range(-2, 3):
                    for iy in range(-2, 3):
                        nx, ny = x + ix, y + iy
                        
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
                else:
                    rospy.logwarn("No valid neighbors found, returning invalid position")
                    avg_x, avg_y, avg_z = -1.0, -1.0, -1.0
                return avg_x, avg_y, avg_z
            else:
                rospy.logerr(
                    f"Coordinates out of bounds: X={x}, Y={y}, width={width}, height={height}"
                )
                raise IndexError("Center coordinates out of bounds")
        else:
            raise ValueError("XYZ array is None")
    
    def get_orientation(self,corners):
        
        y_axis = corners[0]-corners[1]
        x_axis = corners[1]-corners[2]
        
        x_axis_unit = x_axis / (np.linalg.norm(x_axis))
        y_axis_unit = y_axis / (np.linalg.norm(y_axis))
        
        z_axis_unit = np.cross(x_axis_unit,y_axis_unit)
        
        return x_axis_unit, y_axis_unit, z_axis_unit
        
    def broadcast_tag_frame(self, x, y, z, roll, pitch, yaw, src_frame, child_frame):
        try:  

            q=tf.transformations.quaternion_from_euler(roll,pitch,yaw)
            
            t = geometry_msgs.msg.TransformStamped()
            t.header.stamp = rospy.Time.now()
            t.header.frame_id = src_frame
            t.child_frame_id = child_frame
            t.transform.translation.x = x
            t.transform.translation.y = y
            t.transform.translation.z = z
            
            t.transform.rotation.x = q[0]
            t.transform.rotation.y = q[1]
            t.transform.rotation.z = q[2]
            t.transform.rotation.w = q[3]
            
            self.broadcaster.sendTransform(t)
            
        except Exception as  e:
            rospy.logerr(f"Error broadcasting tag frame: {e}")

    def check_tag_center(self, corners):
        avg_two_corners =  (corners[1] + corners[2] )/2 
        xdiff = abs(self.center_x_zed - avg_two_corners[0])
        ydiff = abs(self.center_y_zed - avg_two_corners[1])
        zdiff = abs(self.center_z_zed - avg_two_corners[2])
        
        return  math.sqrt(xdiff**2+ydiff**2+zdiff**2) <= 0.02
            
    
    def define_safety_zone(self):
        # Güvenlik bölgeleri için parametreler (isim, pozisyon, boyut)
        boxes = [
            ("safety_zone", [0.0, 0.0, 0.8], [2, 2, 0.02]),
            ("safety_zone2", [0.0, 0.0, 0.0], [2, 2, 0.02]),
            ("safety_zonex", [0.5, 0.0, 0.0], [0.02, 2, 2]),
            ("safety_zoney", [0.0, 0.5, 0.0], [2, 0.02, 2])
        ]

        # Her güvenlik bölgesini oluştur
        for box_name, position, size in boxes:
            self.add_box(box_name, position, size)
            
    def add_box(self, box_name, position, size, frame_id=BASE_LINK):
        scene = moveit_commander.PlanningSceneInterface()
        
        pose = geometry_msgs.msg.PoseStamped()
        pose.header.frame_id = frame_id
        pose.pose.position.x = position[0]
        pose.pose.position.y = position[1]
        pose.pose.position.z = position[2]
        pose.pose.orientation.w = 1.0
        
        scene.add_box(box_name, pose, size=size)

if __name__ == "__main__":
    rospy.init_node("broadcast")
    perception = Perception()   
    rospy.spin()
    
