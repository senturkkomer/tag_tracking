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
        self.center_x, self.center_y = None, None

        self.bridge = CvBridge()
        self.detector = apriltag.Detector()
        
        rospy.Subscriber(CAM_TOPIC, Image, self.cam_callback)
        rospy.Subscriber(DEPTH_TOPIC, PointCloud2, self.pc_data_callback)

        # Initialize TF Buffer and Listener
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        # Initialize MoveIt
        moveit_commander.roscpp_initialize(sys.argv)
        
        # Initialize MoveIt components
        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()
        self.move_group = moveit_commander.MoveGroupCommander('manipulator')

    def cam_callback(self, msg: Image):
        try:
            self.center_x, self.center_y = None, None
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            gray_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
            detections = self.detector.detect(gray_image)

            for detection in detections:
                center = detection.center
                self.center_x, self.center_y = int(center[0]), int(center[1])
            
            self.show_image(cv_image, self.center_x, self.center_y)
            
        except Exception as e:
            rospy.logerr(f"Error in cam_callback: {e}")

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

    def show_image(self, cv_image, center_x, center_y):
        if center_x is not None and center_y is not None:
            zed_x, zed_y, zed_z = self.get_3d_position(center_x, center_y)
            cv2.circle(cv_image, (center_x, center_y), 2, (0, 255, 0), 2)

            # Create a PoseStamped message for the AprilTag position
            tag_pose = geometry_msgs.msg.PoseStamped()
            tag_pose.header.frame_id = '/zedm_base_link'
            tag_pose.header.stamp = rospy.Time.now()
            tag_pose.pose.position.x = zed_x
            tag_pose.pose.position.y = zed_y
            tag_pose.pose.position.z = zed_z
            print("x: ", zed_x)
            print("y: ", zed_y)
            print("z: ", zed_z)
            # tag_pose.pose.orientation.w = 1.0
            
            try:
                # Transform the tag pose to `base_link` frame
                # transform = self.tf_buffer.lookup_transform( 'base_link','zedm_base_link', rospy.Time(0), rospy.Duration(1.0))
                # tag_pose_in_base_link = tf2_geometry_msgs.do_transform_pose(tag_pose, transform)
                
                # # Set the target pose for MoveIt
                pose_target = geometry_msgs.msg.Pose()
                # pose_target.position = tag_pose_in_base_link.pose.position
                # pose_target.orientation = tag_pose_in_base_link.pose.orientation
                
                # cv2.putText(cv_image, f"{pose_target.position.x:.2f}m, {pose_target.position.y:.2f}m, {pose_target.position.z:.2f}m", (center_x + 5, center_y + 5), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
                # cv2.putText(cv_image, f"{zed_x:.2f}m, {zed_y:.2f}m, {zed_z:.2f}m", (center_x + 5, center_y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
                # self.move_group.set_pose_target(pose_target)
                # # Plan and execute the movement
                # self.move_group.go(wait=True)
                # self.move_group.stop()
                # self.move_group.clear_pose_targets()
                
                broadcaster = tf2_ros.TransformBroadcaster()
                rate = rospy.Rate(10)  # 10 Hz
                
                t = geometry_msgs.msg.TransformStamped()
                t.header.stamp = rospy.Time.now()
                t.header.frame_id = "zedm_base_link"
                t.child_frame_id = "april_tag"
                t.transform.translation.x = zed_x
                t.transform.translation.y = zed_y
                t.transform.translation.z = zed_z
                t.transform.rotation.x = 0.0
                t.transform.rotation.y = 0.0
                t.transform.rotation.z = 0.0
                t.transform.rotation.w = 1.0

                broadcaster.sendTransform(t)
                
                point_in_link = geometry_msgs.msg.PointStamped()
                point_in_link.header.frame_id = "april_tag_center"
                point_in_link.header.stamp = rospy.Time.now()
                point_in_link.point.x = 0
                point_in_link.point.y = 0
                point_in_link.point.z = 0
                

    
    
                transform = self.tf_buffer.lookup_transform('base', 'april_tag', rospy.Time(0), rospy.Duration(1.0))
                point_in_base_link = tf2_geometry_msgs.do_transform_point(point_in_link, transform)
                # print(point_in_base_link)
                
                
                
                
                
                
            
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
                rospy.logerr(f"Error transforming pose: {e}")
        
        cv2.imshow("Apriltag", cv_image)
        cv2.waitKey(1)

if __name__ == "__main__":
    rospy.init_node("zed_apriltag")
    perception = Perception()
    rospy.spin()
