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

# Constants
CAM_TOPIC   = "/zedm/zed_node/rgb_raw/image_raw_color"
DEPTH_TOPIC = "/zedm/zed_node/point_cloud/cloud_registered"

class Perception:
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
                    zed_x, zed_y, zed_z, err = self.get_3d_position(self.center_x, self.center_y)
                    
                    if not err:
                        self.broadcast_tag_frame(zed_x, zed_y, zed_z)
                        self.tag_pose_to_target_pose(zed_x, zed_y, zed_z)
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
            
            if not self.detect_tag(cv_image,gray_image):
                cv2.circle(cv_image, (self.center_x, self.center_y), 2, (0, 255, 0), 2)
            
            cv2.imshow("Apriltag", cv_image)
            cv2.waitKey(1)
        except Exception as e:
            rospy.logerr(f"Error in cam_callback: {e}")  
            
    def detect_tag(self,cv_image,gray_image):
        self.center_x, self.center_y = None, None
        
        detections = self.detector.detect(gray_image)
        for detection in detections:
            center = detection.center
            self.center_x, self.center_y = int(center[0]), int(center[1])
        err = self.center_x is None or self.center_y is None
        return err
            
            
    def get_3d_position(self, center_x, center_y):
        if self.xyz_array is not None :
            height, width, _ = self.xyz_array.shape
            
            if 0 <= center_x < width and 0 <= center_y < height:
                zed_x, zed_y, zed_z = self.xyz_array[center_y, center_x]
                
                if zed_x == -1.0 or zed_y == -1.0 or zed_z == -1.0:
                    rospy.logwarn("3D position contains invalid value")
                err =  zed_x == -1.0 or zed_y == -1.0 or zed_z == -1.0
                return zed_x, zed_y, zed_z, err
            else:
                rospy.logerr(
                    f"Center coordinates out of bounds: center_x={center_x}, center_y={center_y}, width={width}, height={height}"
                )
                raise IndexError("Center coordinates out of bounds")
        else:
            raise ValueError("XYZ array is None")
        
    def broadcast_tag_frame(self,zed_x, zed_y, zed_z):
        try:  
            
            t = geometry_msgs.msg.TransformStamped()
            t.header.stamp = rospy.Time.now()
            t.header.frame_id = "zedm_base_link"
            t.child_frame_id = "april_tag"
            t.transform.translation.x = zed_x
            t.transform.translation.y = -zed_z
            t.transform.translation.z = zed_y
            t.transform.rotation.x = 0.0
            t.transform.rotation.y = 0.0
            t.transform.rotation.z = 0.0
            t.transform.rotation.w = 1.0
            
            self.broadcaster.sendTransform(t)
            
            self.transform = self.tf_buffer.lookup_transform('base_link', 'april_tag', rospy.Time(0), rospy.Duration(2.0))
            
        except Exception as  e:
            rospy.logerr(f"Error broadcasting tag frame: {e}")

    def tag_pose_to_target_pose(self, zed_x, zed_y, zed_z):
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




if __name__ == "__main__":
    rospy.init_node("zed_apriltag")
    perception = Perception()
    
    rospy.spin()
    
