
(cl:in-package :asdf)

(defsystem "faster_msgs-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :std_msgs-msg
)
  :components ((:file "_package")
    (:file "Mode" :depends-on ("_package_Mode"))
    (:file "_package_Mode" :depends-on ("_package"))
  ))