#!/usr/bin/env python3
import rospy
from jointControl.srv import RobotMove
import geometry_msgs.msg
import tf
import tf2_ros
import tf2_geometry_msgs
import math
import moveit_commander
from sensor_msgs.msg import Image, PointCloud2

# Constants
CAM_TOPIC   = "/zedm/zed_node/rgb_raw/image_raw_color"
DEPTH_TOPIC = "/zedm/zed_node/point_cloud/cloud_registered"
ZEDM_BASE = "zedm_base_link"
TAG_BASE = "april_tag"
TARGET_POSE_FRAME = "target_pose"
TARGET_POSE_FRAME2 = "target_pose2"
BASE_LINK = "base_link"




class RobotControl:
    def __init__(self) -> None:
        self.flag = False
        self.camera_connected = False
        
        self.move_group = moveit_commander.MoveGroupCommander("manipulator")
        self.target_pos_in_base_link = None
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        rospy.Subscriber(CAM_TOPIC, Image, self.cam_callback)

        self.robot_go()


    
    def robot_go(self):
        while not rospy.is_shutdown():
            if self.camera_connected:
                rospy.wait_for_service('robot_move')
                pose_target1 = geometry_msgs.msg.PoseStamped()
                pose_target2 = geometry_msgs.msg.PoseStamped()

                self.flag = input("Write 'y' to track apriltag: ") =="y"
                pose_target1_in_base_link = self.tag_pose_to_target_pose(pose_target1,TARGET_POSE_FRAME)
                pose_target2_in_base_link = self.tag_pose_to_target_pose(pose_target2,TARGET_POSE_FRAME2)
                
                if self.flag: 
                    if pose_target1_in_base_link is not None and pose_target2_in_base_link is not None:    
                        if self.robot_move_client(pose_target1_in_base_link.pose):
                            rospy.loginfo("Robot is at target position 1")
                            if self.robot_move_client(pose_target2_in_base_link.pose):
                                rospy.loginfo("Robot is at target position 2")
                        else:
                            rospy.logwarn("Failed to move the robot to the target pose 1.")
                    else:
                        rospy.logwarn("pose_target1_in_base_link or pose_target2_in_base_link is None")
        
    def robot_move_client(self, target_pose):
        try:
            if not self.robot_at_target_pos() :
                move_robot = rospy.ServiceProxy('robot_move', RobotMove)
                response = move_robot(target_pose)
                return response.success
            return True
        except Exception as e:
            rospy.logerr("Service call failed: %s", e)
            return False     
        
    def tag_pose_to_target_pose(self,pose_target, target_pose_frame):
        try:
            
            # PoseStamped mesajını oluşturun
            

            # Header bilgilerini tanımlama
            pose_target.header.stamp = rospy.Time.now()  # Şu anki zamanı kullanın

            pose_target.header.frame_id = target_pose_frame    # Frame ID'yi tanımlayın
            pose_target.pose.position.x = 0
            pose_target.pose.position.y = 0
            pose_target.pose.position.z = 0
            pose_target.pose.orientation.x = 0
            pose_target.pose.orientation.y = 0
            pose_target.pose.orientation.z = 0
            pose_target.pose.orientation.w = 1
            
            if self.tf_buffer.can_transform(BASE_LINK, target_pose_frame, rospy.Time.now(), rospy.Duration(5)):
                transform = self.tf_buffer.lookup_transform(
                    BASE_LINK, target_pose_frame, rospy.Time.now(), rospy.Duration(5))
                
                target_pos_in_base_link = tf2_geometry_msgs.do_transform_pose(
                    pose_target, transform)
                return target_pos_in_base_link
            else:
                rospy.logwarn("It could not be transformed from target pose to base frame")

            # self.broadcaster.sendTransform(self.pose_target)
        
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            rospy.logerr(f"Error transforming pose: {e}")
            
    def robot_at_target_pos(self):
        
        if self.target_pos_in_base_link is None:
            return False
        else:
            current_pose = self.move_group.get_current_pose().pose
            xdiff = abs(current_pose.position.x - self.target_pos_in_base_link.pose.position.x)
            ydiff = abs(current_pose.position.y - self.target_pos_in_base_link.pose.position.y)
            zdiff = abs(current_pose.position.z - self.target_pos_in_base_link.pose.position.z)
        return math.sqrt(xdiff**2 + ydiff**2 + zdiff**2) <= 0.02
    def cam_callback(self, msg: Image):
        self.camera_connected = True     
if __name__ == "__main__":
    rospy.init_node("zed_apriltag2")
    robotcontrol = RobotControl()   
    rospy.spin()