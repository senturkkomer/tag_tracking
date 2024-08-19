#!/usr/bin/env python3

"""
    This sample demonstrates how to capture a live 3D point cloud
    with the ZED SDK and display the result in an OpenGL window.
"""
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
        self.xyz_array = None

        self.bridge = CvBridge()
        self.detector = apriltag.Detector()
        
        self.pose_pub = rospy.Publisher('/target_pose', geometry_msgs.msg.Pose, queue_size=10)
        
        rospy.Subscriber(DEPTH_TOPIC, PointCloud2, self.pc_data_callback)
        rospy.Subscriber(CAM_TOPIC, Image, self.cam_callback)

        # Initialize TF Buffer and Listener
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
  
        self.broadcaster = tf2_ros.TransformBroadcaster()
        rate = rospy.Rate(5)  # 10 Hz

        # MoveIt initializations
        moveit_commander.roscpp_initialize(sys.argv)
        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()
        self.move_group = moveit_commander.MoveGroupCommander("manipulator")
        
        self.move_group.set_max_velocity_scaling_factor(0.05) 
        self.move_group.set_max_acceleration_scaling_factor(0.05)  
        
        self.current_pose = self.move_group.get_current_pose().pose
        
                            
        
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
            
            self.detect_center_tag(cv_image,gray_image)
            
            cv2.imshow("Apriltag", cv_image)
            cv2.waitKey(1)
            if self.center_x is not None and self.center_y is not None :
                zed_x, zed_y, zed_z = self.get_3d_position(self.center_x, self.center_y)
                if zed_x != -1 and zed_y != -1 and zed_z != -1:
                    self.broadcast_tag_frame(zed_x, zed_y, zed_z)
                    self.tag_pose_to_target_pose(zed_x, zed_y, zed_z)
                    print("Current Pos:")
                    print(self.current_pose.position)
                    print(self.current_pose.orientation)
                    print("Target Pos:")
                    print(self.pose_target.position)
                    print(self.pose_target.orientation)

            self.robot_at_target_pos()


        except Exception as e:
            rospy.logerr(f"Error in cam_callback: {e}")  
  
    def detect_center_tag(self,cv_image,gray_image):
        self.center_x, self.center_y = None, None
        
        detections = self.detector.detect(gray_image)
        for detection in detections:
            center = detection.center
            self.center_x, self.center_y = int(center[0]), int(center[1])
            cv2.circle(cv_image, (self.center_x, self.center_y), 2, (0, 255, 0), 2)
            
    def get_3d_position(self, center_x, center_y):
        if self.xyz_array is not None :
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
            
            self.transform = self.tf_buffer.lookup_transform('base_link', 'april_tag', rospy.Time(0), rospy.Duration(1.0))
            
            point_tag_frame_origin = geometry_msgs.msg.PointStamped()
            point_tag_frame_origin.point.x = 0
            point_tag_frame_origin.point.y = 0
            point_tag_frame_origin.point.z = 0

            self.point_in_base_link = tf2_geometry_msgs.do_transform_point(point_tag_frame_origin, self.transform)
        except  e:
            rospy.logerr(f"Error broadcasting tag frame: {e}")

    def tag_pose_to_target_pose(self, zed_x, zed_y, zed_z):
        try:
            # Set the target pose for MoveIt
            self.pose_target = geometry_msgs.msg.Pose()
            self.pose_target.position.x = self.point_in_base_link.point.x
            self.pose_target.position.y = self.point_in_base_link.point.y
            self.pose_target.position.z = self.point_in_base_link.point.z + 0.3
            
            quaternion = tf.transformations.quaternion_from_euler(3.14,0, 1.57)  # (Roll, Pitch, Yaw) = (0, 90°, 0)

            
            self.pose_target.orientation.x = self.current_pose.orientation.x
            self.pose_target.orientation.y = self.current_pose.orientation.y
            self.pose_target.orientation.z = self.current_pose.orientation.z
            self.pose_target.orientation.w = self.current_pose.orientation.w
        

        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            rospy.logerr(f"Error transforming pose: {e}")
        
    def robot_at_target_pos(self):
        xdiff = abs(self.current_pose.position.x - self.pose_target.position.x)
        ydiff = abs(self.current_pose.position.y - self.pose_target.position.y)
        zdiff = abs(self.current_pose.position.z - self.pose_target.position.z)

        if xdiff > 0.05 or ydiff > 0.05 or zdiff > 0.05:
            
            self.pose_pub.publish(self.pose_target)
            self.move_group.set_pose_target(self.pose_target)
            self.move_group.go(wait=True)
            
        else:
            self.move_group.stop()
            self.move_group.clear_pose_targets()




if __name__ == "__main__":
    rospy.init_node("zed_apriltag")
    perception = Perception()
    rospy.spin()
