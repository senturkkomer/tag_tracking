; Auto-generated. Do not edit!


(cl:in-package robotic_arm-msg)


;//! \htmlinclude joint_position_array.msg.html

(cl:defclass <joint_position_array> (roslisp-msg-protocol:ros-message)
  ((data
    :reader data
    :initarg :data
    :type (cl:vector robotic_arm-msg:joint_position)
   :initform (cl:make-array 0 :element-type 'robotic_arm-msg:joint_position :initial-element (cl:make-instance 'robotic_arm-msg:joint_position))))
)

(cl:defclass joint_position_array (<joint_position_array>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <joint_position_array>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'joint_position_array)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name robotic_arm-msg:<joint_position_array> is deprecated: use robotic_arm-msg:joint_position_array instead.")))

(cl:ensure-generic-function 'data-val :lambda-list '(m))
(cl:defmethod data-val ((m <joint_position_array>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader robotic_arm-msg:data-val is deprecated.  Use robotic_arm-msg:data instead.")
  (data m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <joint_position_array>) ostream)
  "Serializes a message object of type '<joint_position_array>"
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'data))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'data))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <joint_position_array>) istream)
  "Deserializes a message object of type '<joint_position_array>"
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'data) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'data)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'robotic_arm-msg:joint_position))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<joint_position_array>)))
  "Returns string type for a message object of type '<joint_position_array>"
  "robotic_arm/joint_position_array")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'joint_position_array)))
  "Returns string type for a message object of type 'joint_position_array"
  "robotic_arm/joint_position_array")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<joint_position_array>)))
  "Returns md5sum for a message object of type '<joint_position_array>"
  "2f6cb9944d2c5ab5cff8aff4ba87d255")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'joint_position_array)))
  "Returns md5sum for a message object of type 'joint_position_array"
  "2f6cb9944d2c5ab5cff8aff4ba87d255")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<joint_position_array>)))
  "Returns full string definition for message of type '<joint_position_array>"
  (cl:format cl:nil "joint_position[] data~%~%================================================================================~%MSG: robotic_arm/joint_position~%float32[] data~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'joint_position_array)))
  "Returns full string definition for message of type 'joint_position_array"
  (cl:format cl:nil "joint_position[] data~%~%================================================================================~%MSG: robotic_arm/joint_position~%float32[] data~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <joint_position_array>))
  (cl:+ 0
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'data) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <joint_position_array>))
  "Converts a ROS message object to a list"
  (cl:list 'joint_position_array
    (cl:cons ':data (data msg))
))
