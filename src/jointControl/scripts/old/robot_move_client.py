#!/usr/bin/env python3

import rospy
from jointControl.srv import RobotMove
import geometry_msgs.msg

def robot_move_client(target_pose):
    rospy.wait_for_service('robot_move')
    try:
        move_robot = rospy.ServiceProxy('robot_move', RobotMove)
        response = move_robot(target_pose)
        return response.success
    except rospy.ServiceException as e:
        rospy.logerr("Service call failed: %s", e)
        return False

if __name__ == "__main__":
    rospy.init_node('robot_move_client')

    # Örnek hedef pozisyonu oluşturun
    target_pose = geometry_msgs.msg.Pose()
    target_pose.position.x = 0.5
    target_pose.position.y = 0.0
    target_pose.position.z = 0.5
    target_pose.orientation.w = 1.0

    success = robot_move_client(target_pose)
    if success:
        rospy.loginfo("Robot moved to target pose successfully.")
    else:
        rospy.logwarn("Failed to move the robot to the target pose.")
