
(cl:in-package :asdf)

(defsystem "robotic_arm-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "joint_position" :depends-on ("_package_joint_position"))
    (:file "_package_joint_position" :depends-on ("_package"))
    (:file "joint_position_array" :depends-on ("_package_joint_position_array"))
    (:file "_package_joint_position_array" :depends-on ("_package"))
  ))