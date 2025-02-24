#!/usr/bin/env python3

import sys

import rospy
import actionlib
from robotic_arm.msg import joint_position_array
from robotic_arm.msg import joint_position
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from trajectory_msgs.msg import JointTrajectoryPoint
from controller_manager_msgs.srv import SwitchControllerRequest, SwitchController
from controller_manager_msgs.srv import LoadControllerRequest, LoadController
from controller_manager_msgs.srv import ListControllers, ListControllersRequest
from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseArray
import geometry_msgs.msg as geometry_msgs

from cartesian_control_msgs.msg import (
    FollowCartesianTrajectoryAction,
    FollowCartesianTrajectoryGoal,
    CartesianTrajectoryPoint,
)

if sys.version_info[0] < 3:
    input = raw_input

JOINT_NAMES = [
    "shoulder_pan_joint",
    "shoulder_lift_joint",
    "elbow_joint",
    "wrist_1_joint",
    "wrist_2_joint",
    "wrist_3_joint",
]

JOINT_TRAJECTORY_CONTROLLERS = [
    "scaled_pos_joint_traj_controller",
    "scaled_vel_joint_traj_controller",
    "pos_joint_traj_controller",
    "vel_joint_traj_controller",
    "forward_joint_traj_controller",
]

CARTESIAN_TRAJECTORY_CONTROLLERS = [
    "pose_based_cartesian_traj_controller",
    "joint_based_cartesian_traj_controller",
    "forward_cartesian_traj_controller",
]

CONFLICTING_CONTROLLERS = ["joint_group_vel_controller", "twist_controller"]


class TrajectoryClient:
    """Small trajectory client to test a joint trajectory"""

    def __init__(self):
        timeout = rospy.Duration(5)
        self.switch_srv = rospy.ServiceProxy(
            "controller_manager/switch_controller", SwitchController
        )
        self.load_srv = rospy.ServiceProxy(
            "controller_manager/load_controller", LoadController
        )
        self.list_srv = rospy.ServiceProxy(
            "controller_manager/list_controllers", ListControllers
        )
        try:
            self.switch_srv.wait_for_service(timeout.to_sec())
        except rospy.exceptions.ROSException as err:
            rospy.logerr(
                "Could not reach controller switch service. Msg: {}".format(err)
            )
            sys.exit(-1)

        # self.joint_trajectory_controller = JOINT_TRAJECTORY_CONTROLLERS[0]
        self.cartesian_trajectory_controller = CARTESIAN_TRAJECTORY_CONTROLLERS[0]

    # def send_joint_trajectory(self, pose_list):
    #     """Creates a trajectory and sends it using the selected action server"""

    #     # make sure the correct controller is loaded and activated
    #     self.switch_controller(self.joint_trajectory_controller)
    #     trajectory_client = actionlib.SimpleActionClient(
    #         "{}/follow_joint_trajectory".format(self.joint_trajectory_controller),
    #         FollowJointTrajectoryAction,
    #     )

    #     # Wait for action server to be ready
    #     timeout = rospy.Duration(5)
    #     if not trajectory_client.wait_for_server(timeout):
    #         rospy.logerr("Could not reach controller action server.")
    #         sys.exit(-1)

    #     # Create and fill trajectory goal
    #     goal = FollowJointTrajectoryGoal()
    #     goal.trajectory.joint_names = JOINT_NAMES
    #     # The following list are arbitrary positions
    #     # Change to your own needs if desired
    #     position_list = pose_list

    #     velocity_list = [[0.2, 0, 0, 0, 0, 0]]
    #     velocity_list.append([-0.2, 0, 0, 0, 0, 0])
    #     velocity_list.append([0, 0, 0, 0, 0, 0])
    #     duration_list = [3.0, 7.0, 10.0]
    #     for i, position in enumerate(position_list):
    #         point = JointTrajectoryPoint()
    #         point.positions = position
    #         point.velocities = velocity_list[i]
    #         point.time_from_start = rospy.Duration(duration_list[i])
    #         goal.trajectory.points.append(point)

    #     self.ask_confirmation(position_list)
    #     rospy.loginfo(
    #         "Executing trajectory using the {}".format(self.joint_trajectory_controller)
    #     )

    #     trajectory_client.send_goal(goal)
    #     trajectory_client.wait_for_result()

    #     result = trajectory_client.get_result()
    #     rospy.loginfo(
    #         "Trajectory execution finished in state {}".format(result.error_code)
    #     )

    # def joint_callback(self, data):
    #     pose_list = data.data
    #     i = 0
    #     pose_list_array = []
    #     for i, pose in enumerate(pose_list):
    #         pose_list_array.append(pose.data)
    #     # pose_list_array.pop(0)
    #     self.send_joint_trajectory(pose_list_array)

    # def joint_listener(self):
    #     rospy.Subscriber("joint_pose", joint_position_array, self.joint_callback)

    def send_cartesian_trajectory(self, position):
        """Creates a Cartesian trajectory and sends it using the selected action server"""
        self.switch_controller(self.cartesian_trajectory_controller)

        # make sure the correct controller is loaded and activated
        goal = FollowCartesianTrajectoryGoal()
        trajectory_client = actionlib.SimpleActionClient(
            "{}/follow_cartesian_trajectory".format(
                self.cartesian_trajectory_controller
            ),
            FollowCartesianTrajectoryAction,
        )

        # Wait for action server to be ready
        timeout = rospy.Duration(5)
        if not trajectory_client.wait_for_server(timeout):
            rospy.logerr("Could not reach controller action server.")
            sys.exit(-1)

        pose_list = position

        duration_list = [3.0, 4.0, 5.0, 6.0, 7.0]
        for i, pose in enumerate(pose_list):
            point = CartesianTrajectoryPoint()
            point.pose = pose
            point.time_from_start = rospy.Duration(duration_list[i])
            goal.trajectory.points.append(point)

        self.ask_confirmation(pose_list)
        rospy.loginfo(
            "Executing trajectory using the {}".format(
                self.cartesian_trajectory_controller
            )
        )
        trajectory_client.send_goal(goal)
        trajectory_client.wait_for_result()

        result = trajectory_client.get_result()

        rospy.loginfo(
            "Trajectory execution finished in state {}".format(result.error_code)
        )

    def cartesian_callback(self, data):
        pose_list = data.poses

        self.send_cartesian_trajectory(pose_list)

    def cartesian_listener(self):
        rospy.Subscriber("cartesian_pose", PoseArray, self.cartesian_callback)

    ###
    def ask_confirmation(self, waypoint_list):
        """Ask the user for confirmation. This function is obviously not necessary, but makes sense
        in a testing script when you know nothing about the user's setup."""
        rospy.logwarn(
            "The robot will move to the following waypoints: \n{}".format(waypoint_list)
        )
        confirmed = False
        valid = False
        while not valid:
            input_str = input(
                "Please confirm that the robot path is clear of obstacles.\n"
                "Keep the EM-Stop available at all times. You are executing\n"
                "the motion at your own risk. Please type 'y' to proceed or 'n' to abort: "
            )
            valid = input_str in ["y", "n"]
            if not valid:
                rospy.loginfo("Please confirm by entering 'y' or abort by entering 'n'")
            else:
                confirmed = input_str == "y"
        if not confirmed:
            rospy.loginfo("Exiting as requested by user.")
            sys.exit(0)

    def choose_controller(self):
        # """Ask the user to select the desired controller from the available list."""
        # rospy.loginfo("Available trajectory controllers:")
        # for index, name in enumerate(JOINT_TRAJECTORY_CONTROLLERS):
        #     rospy.loginfo("{} (joint-based): {}".format(index, name))
        # for index, name in enumerate(CARTESIAN_TRAJECTORY_CONTROLLERS):
        #     rospy.loginfo(
        #         "{} (Cartesian): {}".format(
        #             index + len(JOINT_TRAJECTORY_CONTROLLERS), name
        #         )
        #     )
        choice = 0
        # while choice < 0:
        #     input_str = input(
        #         "Please choose a controller by entering its number (Enter '0' if "
        #         "you are unsure / don't care): "
        #     )
        #     try:
        #         choice = int(input_str)
        #         if choice < 0 or choice >= len(JOINT_TRAJECTORY_CONTROLLERS) + len(
        #             CARTESIAN_TRAJECTORY_CONTROLLERS
        #         ):
        #             rospy.loginfo(
        #                 "{} not inside the list of options. "
        #                 "Please enter a valid index from the list above.".format(choice)
        #             )
        #             choice = -1
        #     except ValueError:
        #         rospy.loginfo("Input is not a valid number. Please try again.")
        # if choice < len(JOINT_TRAJECTORY_CONTROLLERS):
        #     self.joint_trajectory_controller = JOINT_TRAJECTORY_CONTROLLERS[choice]
        #     return "joint_based"
        self.cartesian_trajectory_controller = CARTESIAN_TRAJECTORY_CONTROLLERS[
            choice
        ]
        return "cartesian"

    def switch_controller(self, target_controller):
        """Activates the desired controller and stops all others from the predefined list above"""
        other_controllers = (
            JOINT_TRAJECTORY_CONTROLLERS
            + CARTESIAN_TRAJECTORY_CONTROLLERS
            + CONFLICTING_CONTROLLERS
        )

        other_controllers.remove(target_controller)

        srv = ListControllersRequest()
        response = self.list_srv(srv)
        for controller in response.controller:
            if controller.name == target_controller and controller.state == "running":
                return

        srv = LoadControllerRequest()
        srv.name = target_controller
        self.load_srv(srv)

        srv = SwitchControllerRequest()
        srv.stop_controllers = other_controllers
        srv.start_controllers = [target_controller]
        srv.strictness = SwitchControllerRequest.BEST_EFFORT
        self.switch_srv(srv)


if __name__ == "__main__":
    rospy.init_node("robotic_arm_controller")
    client = TrajectoryClient()

    # The controller choice is obviously not required to move the robot. It is a part of this demo
    # script in order to show all available trajectory controllers.
    trajectory_type = client.choose_controller()
    if trajectory_type == "joint_based":
        client.joint_listener()
        rospy.spin()
    elif trajectory_type == "cartesian":
        client.cartesian_listener()
        rospy.spin()
        # client.send_cartesian_trajectory()
    else:
        raise ValueError(
            "I only understand types 'joint_based' and 'cartesian', but got '{}'".format(
                trajectory_type
            )
        )
