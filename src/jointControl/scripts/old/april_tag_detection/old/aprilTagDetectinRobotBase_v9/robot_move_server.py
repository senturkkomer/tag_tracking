#!/usr/bin/env python3
import sys
import rospy
from jointControl.srv import RobotMove, RobotMoveResponse
import moveit_commander
import geometry_msgs.msg

def handle_robot_move(req):
    rospy.loginfo("Received target pose: %s", req.target_pose)
    move_group = moveit_commander.MoveGroupCommander("manipulator")
    # gripper_group = moveit_commander.move_group.MoveGroupCommander("gripper")
    
    current_pose = move_group.get_current_pose().pose
    rospy.loginfo("Current pose: %s", current_pose)
    
    # MoveIt! ile hareket etme kodunu buraya ekleyin
    moveit_commander.roscpp_initialize(sys.argv)
    robot = moveit_commander.RobotCommander()
    

    move_group.set_pose_target(req.target_pose)
    move_group.go(wait=True)
    rospy.sleep(1)

    # close_gripper = [gripper_group.get_named_target_values('closed')['robotiq_85_left_knuckle_joint']]
    # gripper_group.go(close_gripper, wait=True)
    return RobotMoveResponse(success=True)

def robot_move_server():
    rospy.init_node('robot_move_server')
    s = rospy.Service('robot_move', RobotMove, handle_robot_move)
    
    rospy.loginfo("Ready to move the robot.")
    rospy.spin()



    

if __name__ == "__main__":
    robot_move_server()