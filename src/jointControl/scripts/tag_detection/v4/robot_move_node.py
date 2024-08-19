#!/usr/bin/env python3
import rospy
from jointControl.srv import RobotMove
import geometry_msgs.msg
import tf
import tf2_ros
import tf2_geometry_msgs
import math
import moveit_commander

# Constants
CAM_TOPIC   = "/zedm/zed_node/rgb_raw/image_raw_color"
DEPTH_TOPIC = "/zedm/zed_node/point_cloud/cloud_registered"
ZEDM_BASE = "zedm_base_link"
TAG_BASE = "april_tag"
TARGET_POSE_FRAME = "target_pose"
BASE_LINK = "base_link"




class RobotControl:
    def __init__(self) -> None:
        self.flag = False
        self.move_group = moveit_commander.MoveGroupCommander("manipulator")
        self.target_pos_in_base_link = None
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        while not rospy.is_shutdown():
            rospy.wait_for_service('robot_move')
            
            self.robot_go()


    
    def robot_go(self):
        
        
        self.tag_pose_to_target_pose()

        self.flag = input("Apriltag e gitmek için y yaziniz:") =="y"
        
        if self.flag: 
            if self.target_pos_in_base_link is not None:    
                if self.robot_move_client(self.target_pos_in_base_link.pose):
                    rospy.loginfo("Robot is at target position")
                else:
                    rospy.logwarn("Failed to move the robot to the target pose.")
            else:
                rospy.logwarn("self.target_pos_in_base_link is None")
        
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
        
    def tag_pose_to_target_pose(self):
        try:
            # PoseStamped mesajını oluşturun
            self.pose_target = geometry_msgs.msg.PoseStamped()

            # Header bilgilerini tanımlama
            self.pose_target.header.stamp = rospy.Time.now()  # Şu anki zamanı kullanın

            self.pose_target.header.frame_id = TARGET_POSE_FRAME    # Frame ID'yi tanımlayın
            self.pose_target.pose.position.x = 0
            self.pose_target.pose.position.y = 0
            self.pose_target.pose.position.z = 0
            self.pose_target.pose.orientation.x = 0
            self.pose_target.pose.orientation.y = 0
            self.pose_target.pose.orientation.z = 0
            self.pose_target.pose.orientation.w = 1

            self.transform = self.tf_buffer.lookup_transform(
                BASE_LINK, TARGET_POSE_FRAME, rospy.Time.now(), rospy.Duration(5))
            
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
            xdiff = abs(current_pose.position.x - self.target_pos_in_base_link.pose.position.x)
            ydiff = abs(current_pose.position.y - self.target_pos_in_base_link.pose.position.y)
            zdiff = abs(current_pose.position.z - self.target_pos_in_base_link.pose.position.z)
        return math.sqrt(xdiff**2 + ydiff**2 + zdiff**2) <= 0.05
    
if __name__ == "__main__":
    rospy.init_node("zed_apriltag2")
    robotcontrol = RobotControl()   
    rospy.spin()