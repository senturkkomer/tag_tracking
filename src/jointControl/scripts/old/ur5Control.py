#!/usr/bin/env python3

import rospy
import moveit_commander
import geometry_msgs.msg
import sys
import pyzed.sl as sl
import cv2
import apriltag

def main():
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('move_group_python_interface', anonymous=True)

    robot = moveit_commander.RobotCommander()
    scene = moveit_commander.PlanningSceneInterface()
    group_name = "manipulator"
    move_group = moveit_commander.MoveGroupCommander(group_name)

    init = sl.InitParameters(depth_mode=sl.DEPTH_MODE.ULTRA,
                             coordinate_units=sl.UNIT.METER,
                             coordinate_system=sl.COORDINATE_SYSTEM.RIGHT_HANDED_Y_UP)

    zed = sl.Camera()
    status = zed.open(init)
    if status != sl.ERROR_CODE.SUCCESS:
        print(repr(status))
        exit()

    runtime = sl.RuntimeParameters()
    mat_image = sl.Mat()
    mat_depth = sl.Mat()

    detector = apriltag.Detector()

    while not rospy.is_shutdown():
        if zed.grab(runtime) == sl.ERROR_CODE.SUCCESS:
            zed.retrieve_image(mat_image, sl.VIEW.LEFT)
            zed.retrieve_measure(mat_depth, sl.MEASURE.DEPTH)

            image_ocv = mat_image.get_data()
            gray_image = cv2.cvtColor(image_ocv, cv2.COLOR_BGR2GRAY)
            detections = detector.detect(gray_image)

            for detection in detections:
                center = detection.center
                x, y = int(center[0]), int(center[1])
                depth_value = mat_depth.get_value(x, y)[1]
                print(f"AprilTag detected at ({x}, {y}) with depth {depth_value} meters")

                target_pose = geometry_msgs.msg.Pose()
                target_pose.orientation.w = 1.0
                target_pose.position.x = x / 1000.0  # Koordinatları metreye çevir
                target_pose.position.y = y / 1000.0
                target_pose.position.z = depth_value

                move_group.set_pose_target(target_pose)
                plan = move_group.go(wait=True)
                move_group.stop()
                move_group.clear_pose_targets()

            cv2.imshow("Image", image_ocv)
            if cv2.waitKey(1) & 0xFF == 27:  # Press 'Esc' to exit
                break

    zed.close()
    cv2.destroyAllWindows()
    362.roscpp_shutdown()

if __name__ == "__main__":
    main()
