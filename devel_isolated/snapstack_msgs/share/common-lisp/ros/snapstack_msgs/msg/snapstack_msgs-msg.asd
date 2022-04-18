
(cl:in-package :asdf)

(defsystem "snapstack_msgs-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :geometry_msgs-msg
               :std_msgs-msg
)
  :components ((:file "_package")
    (:file "AttitudeCommand" :depends-on ("_package_AttitudeCommand"))
    (:file "_package_AttitudeCommand" :depends-on ("_package"))
    (:file "CommAge" :depends-on ("_package_CommAge"))
    (:file "_package_CommAge" :depends-on ("_package"))
    (:file "ControlLog" :depends-on ("_package_ControlLog"))
    (:file "_package_ControlLog" :depends-on ("_package"))
    (:file "Goal" :depends-on ("_package_Goal"))
    (:file "_package_Goal" :depends-on ("_package"))
    (:file "IMU" :depends-on ("_package_IMU"))
    (:file "_package_IMU" :depends-on ("_package"))
    (:file "Motors" :depends-on ("_package_Motors"))
    (:file "_package_Motors" :depends-on ("_package"))
    (:file "QuadFlightMode" :depends-on ("_package_QuadFlightMode"))
    (:file "_package_QuadFlightMode" :depends-on ("_package"))
    (:file "SMCData" :depends-on ("_package_SMCData"))
    (:file "_package_SMCData" :depends-on ("_package"))
    (:file "State" :depends-on ("_package_State"))
    (:file "_package_State" :depends-on ("_package"))
  ))