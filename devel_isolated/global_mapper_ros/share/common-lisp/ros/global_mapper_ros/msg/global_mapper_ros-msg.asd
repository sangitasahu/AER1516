
(cl:in-package :asdf)

(defsystem "global_mapper_ros-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :std_msgs-msg
)
  :components ((:file "_package")
    (:file "PlanningGrids" :depends-on ("_package_PlanningGrids"))
    (:file "_package_PlanningGrids" :depends-on ("_package"))
  ))