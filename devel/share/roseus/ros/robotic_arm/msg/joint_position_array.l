;; Auto-generated. Do not edit!


(when (boundp 'robotic_arm::joint_position_array)
  (if (not (find-package "ROBOTIC_ARM"))
    (make-package "ROBOTIC_ARM"))
  (shadow 'joint_position_array (find-package "ROBOTIC_ARM")))
(unless (find-package "ROBOTIC_ARM::JOINT_POSITION_ARRAY")
  (make-package "ROBOTIC_ARM::JOINT_POSITION_ARRAY"))

(in-package "ROS")
;;//! \htmlinclude joint_position_array.msg.html


(defclass robotic_arm::joint_position_array
  :super ros::object
  :slots (_data ))

(defmethod robotic_arm::joint_position_array
  (:init
   (&key
    ((:data __data) ())
    )
   (send-super :init)
   (setq _data __data)
   self)
  (:data
   (&rest __data)
   (if (keywordp (car __data))
       (send* _data __data)
     (progn
       (if __data (setq _data (car __data)))
       _data)))
  (:serialization-length
   ()
   (+
    ;; robotic_arm/joint_position[] _data
    (apply #'+ (send-all _data :serialization-length)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; robotic_arm/joint_position[] _data
     (write-long (length _data) s)
     (dolist (elem _data)
       (send elem :serialize s)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; robotic_arm/joint_position[] _data
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _data (let (r) (dotimes (i n) (push (instance robotic_arm::joint_position :init) r)) r))
     (dolist (elem- _data)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;;
   self)
  )

(setf (get robotic_arm::joint_position_array :md5sum-) "2f6cb9944d2c5ab5cff8aff4ba87d255")
(setf (get robotic_arm::joint_position_array :datatype-) "robotic_arm/joint_position_array")
(setf (get robotic_arm::joint_position_array :definition-)
      "joint_position[] data

================================================================================
MSG: robotic_arm/joint_position
float32[] data

")



(provide :robotic_arm/joint_position_array "2f6cb9944d2c5ab5cff8aff4ba87d255")


