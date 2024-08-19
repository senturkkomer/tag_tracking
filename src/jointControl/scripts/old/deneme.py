#!/usr/bin/env python3

"""
    This sample demonstrates how to capture a live 3D point cloud
    with the ZED SDK and display the result in an OpenGL window.
"""

import sys
import ogl_viewer.viewer as gl
import pyzed.sl as sl
import argparse
import cv2
import apriltag



def main():

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

    while True:
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

                cv2.circle(image_ocv, (x, y), 2, (0, 255, 0), 2)
                cv2.putText(image_ocv, f"{depth_value:.2f}m", (x + 5, y+5), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

            cv2.imshow("Image", image_ocv)
            if cv2.waitKey(1) & 0xFF == 27:  # Press 'Esc' to exit
                break

    zed.close()
    cv2.destroyAllWindows()

if __name__ == "__main__":

    main()
