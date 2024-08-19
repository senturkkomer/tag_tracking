
(cl:in-package :asdf)

(defsystem "jointControl-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :geometry_msgs-msg
)
  :components ((:file "_package")
    (:file "BroadcastTagFrame" :depends-on ("_package_BroadcastTagFrame"))
    (:file "_package_BroadcastTagFrame" :depends-on ("_package"))
    (:file "RobotMove" :depends-on ("_package_RobotMove"))
    (:file "_package_RobotMove" :depends-on ("_package"))
  ))