
(cl:in-package :asdf)

(defsystem "map_simulator-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :geometry_msgs-msg
)
  :components ((:file "_package")
    (:file "Map3D" :depends-on ("_package_Map3D"))
    (:file "_package_Map3D" :depends-on ("_package"))
  ))