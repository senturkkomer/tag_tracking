;; Auto-generated. Do not edit!


(when (boundp 'jointControl::RobotMove)
  (if (not (find-package "JOINTCONTROL"))
    (make-package "JOINTCONTROL"))
  (shadow 'RobotMove (find-package "JOINTCONTROL")))
(unless (find-package "JOINTCONTROL::ROBOTMOVE")
  (make-package "JOINTCONTROL::ROBOTMOVE"))
(unless (find-package "JOINTCONTROL::ROBOTMOVEREQUEST")
  (make-package "JOINTCONTROL::ROBOTMOVEREQUEST"))
(unless (find-package "JOINTCONTROL::ROBOTMOVERESPONSE")
  (make-package "JOINTCONTROL::ROBOTMOVERESPONSE"))

(in-package "ROS")

(if (not (find-package "GEOMETRY_MSGS"))
  (ros::roseus-add-msgs "geometry_msgs"))




(defclass jointControl::RobotMoveRequest
  :super ros::object
  :slots (_target_pose ))

(defmethod jointControl::RobotMoveRequest
  (:init
   (&key
    ((:target_pose __target_pose) (instance geometry_msgs::Pose :init))
    )
   (send-super :init)
   (setq _target_pose __target_pose)
   self)
  (:target_pose
   (&rest __target_pose)
   (if (keywordp (car __target_pose))
       (send* _target_pose __target_pose)
     (progn
       (if __target_pose (setq _target_pose (car __target_pose)))
       _target_pose)))
  (:serialization-length
   ()
   (+
    ;; geometry_msgs/Pose _target_pose
    (send _target_pose :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; geometry_msgs/Pose _target_pose
       (send _target_pose :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; geometry_msgs/Pose _target_pose
     (send _target_pose :deserialize buf ptr-) (incf ptr- (send _target_pose :serialization-length))
   ;;
   self)
  )

(defclass jointControl::RobotMoveResponse
  :super ros::object
  :slots (_success ))

(defmethod jointControl::RobotMoveResponse
  (:init
   (&key
    ((:success __success) nil)
    )
   (send-super :init)
   (setq _success __success)
   self)
  (:success
   (&optional (__success :null))
   (if (not (eq __success :null)) (setq _success __success)) _success)
  (:serialization-length
   ()
   (+
    ;; bool _success
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; bool _success
       (if _success (write-byte -1 s) (write-byte 0 s))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; bool _success
     (setq _success (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;;
   self)
  )

(defclass jointControl::RobotMove
  :super ros::object
  :slots ())

(setf (get jointControl::RobotMove :md5sum-) "fca097bf97be1a76fa923cfcb956b244")
(setf (get jointControl::RobotMove :datatype-) "jointControl/RobotMove")
(setf (get jointControl::RobotMove :request) jointControl::RobotMoveRequest)
(setf (get jointControl::RobotMove :response) jointControl::RobotMoveResponse)

(defmethod jointControl::RobotMoveRequest
  (:response () (instance jointControl::RobotMoveResponse :init)))

(setf (get jointControl::RobotMoveRequest :md5sum-) "fca097bf97be1a76fa923cfcb956b244")
(setf (get jointControl::RobotMoveRequest :datatype-) "jointControl/RobotMoveRequest")
(setf (get jointControl::RobotMoveRequest :definition-)
      "geometry_msgs/Pose target_pose

================================================================================
MSG: geometry_msgs/Pose
# A representation of pose in free space, composed of position and orientation. 
Point position
Quaternion orientation

================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z

================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x
float64 y
float64 z
float64 w
---
bool success
")

(setf (get jointControl::RobotMoveResponse :md5sum-) "fca097bf97be1a76fa923cfcb956b244")
(setf (get jointControl::RobotMoveResponse :datatype-) "jointControl/RobotMoveResponse")
(setf (get jointControl::RobotMoveResponse :definition-)
      "geometry_msgs/Pose target_pose

================================================================================
MSG: geometry_msgs/Pose
# A representation of pose in free space, composed of position and orientation. 
Point position
Quaternion orientation

================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z

================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x
float64 y
float64 z
float64 w
---
bool success
")



(provide :jointControl/RobotMove "fca097bf97be1a76fa923cfcb956b244")


