; Auto-generated. Do not edit!


(cl:in-package jointControl-srv)


;//! \htmlinclude RobotMove-request.msg.html

(cl:defclass <RobotMove-request> (roslisp-msg-protocol:ros-message)
  ((target_pose
    :reader target_pose
    :initarg :target_pose
    :type geometry_msgs-msg:Pose
    :initform (cl:make-instance 'geometry_msgs-msg:Pose)))
)

(cl:defclass RobotMove-request (<RobotMove-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <RobotMove-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'RobotMove-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name jointControl-srv:<RobotMove-request> is deprecated: use jointControl-srv:RobotMove-request instead.")))

(cl:ensure-generic-function 'target_pose-val :lambda-list '(m))
(cl:defmethod target_pose-val ((m <RobotMove-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader jointControl-srv:target_pose-val is deprecated.  Use jointControl-srv:target_pose instead.")
  (target_pose m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <RobotMove-request>) ostream)
  "Serializes a message object of type '<RobotMove-request>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'target_pose) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <RobotMove-request>) istream)
  "Deserializes a message object of type '<RobotMove-request>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'target_pose) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<RobotMove-request>)))
  "Returns string type for a service object of type '<RobotMove-request>"
  "jointControl/RobotMoveRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'RobotMove-request)))
  "Returns string type for a service object of type 'RobotMove-request"
  "jointControl/RobotMoveRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<RobotMove-request>)))
  "Returns md5sum for a message object of type '<RobotMove-request>"
  "fca097bf97be1a76fa923cfcb956b244")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'RobotMove-request)))
  "Returns md5sum for a message object of type 'RobotMove-request"
  "fca097bf97be1a76fa923cfcb956b244")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<RobotMove-request>)))
  "Returns full string definition for message of type '<RobotMove-request>"
  (cl:format cl:nil "geometry_msgs/Pose target_pose~%~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of position and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'RobotMove-request)))
  "Returns full string definition for message of type 'RobotMove-request"
  (cl:format cl:nil "geometry_msgs/Pose target_pose~%~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of position and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <RobotMove-request>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'target_pose))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <RobotMove-request>))
  "Converts a ROS message object to a list"
  (cl:list 'RobotMove-request
    (cl:cons ':target_pose (target_pose msg))
))
;//! \htmlinclude RobotMove-response.msg.html

(cl:defclass <RobotMove-response> (roslisp-msg-protocol:ros-message)
  ((success
    :reader success
    :initarg :success
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass RobotMove-response (<RobotMove-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <RobotMove-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'RobotMove-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name jointControl-srv:<RobotMove-response> is deprecated: use jointControl-srv:RobotMove-response instead.")))

(cl:ensure-generic-function 'success-val :lambda-list '(m))
(cl:defmethod success-val ((m <RobotMove-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader jointControl-srv:success-val is deprecated.  Use jointControl-srv:success instead.")
  (success m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <RobotMove-response>) ostream)
  "Serializes a message object of type '<RobotMove-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'success) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <RobotMove-response>) istream)
  "Deserializes a message object of type '<RobotMove-response>"
    (cl:setf (cl:slot-value msg 'success) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<RobotMove-response>)))
  "Returns string type for a service object of type '<RobotMove-response>"
  "jointControl/RobotMoveResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'RobotMove-response)))
  "Returns string type for a service object of type 'RobotMove-response"
  "jointControl/RobotMoveResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<RobotMove-response>)))
  "Returns md5sum for a message object of type '<RobotMove-response>"
  "fca097bf97be1a76fa923cfcb956b244")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'RobotMove-response)))
  "Returns md5sum for a message object of type 'RobotMove-response"
  "fca097bf97be1a76fa923cfcb956b244")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<RobotMove-response>)))
  "Returns full string definition for message of type '<RobotMove-response>"
  (cl:format cl:nil "bool success~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'RobotMove-response)))
  "Returns full string definition for message of type 'RobotMove-response"
  (cl:format cl:nil "bool success~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <RobotMove-response>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <RobotMove-response>))
  "Converts a ROS message object to a list"
  (cl:list 'RobotMove-response
    (cl:cons ':success (success msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'RobotMove)))
  'RobotMove-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'RobotMove)))
  'RobotMove-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'RobotMove)))
  "Returns string type for a service object of type '<RobotMove>"
  "jointControl/RobotMove")