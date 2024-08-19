#!/usr/bin/env python3
import sys
import rospy
from jointControl.srv import RobotMove, RobotMoveResponse
import moveit_commander
import geometry_msgs.msg

def handle_robot_move(req):
    rospy.loginfo("Received target pose: %s", req.target_pose)
    move_group = moveit_commander.MoveGroupCommander("manipulator")
    gripper_group = moveit_commander.move_group.MoveGroupCommander("gripper")
    
    current_pose = move_group.get_current_pose().pose
    rospy.loginfo("Current pose: %s", current_pose)
    
    # MoveIt! ile hareket etme kodunu buraya ekleyin
    moveit_commander.roscpp_initialize(sys.argv)
    robot = moveit_commander.RobotCommander()
    scene = moveit_commander.PlanningSceneInterface()

    move_group.set_pose_target(req.target_pose)
    move_group.go(wait=True)
    rospy.sleep(1)
    close_gripper = [gripper_group.get_named_target_values('closed')['robotiq_85_left_knuckle_joint']]
    gripper_group.go(close_gripper, wait=True)
    return RobotMoveResponse(success=True)

def robot_move_server():
    rospy.init_node('robot_move_server')
    s = rospy.Service('robot_move', RobotMove, handle_robot_move)
    
    rospy.loginfo("Ready to move the robot.")
    rospy.spin()


def define_safety_zone():
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
    robot_move_server()
    define_safety_zone()