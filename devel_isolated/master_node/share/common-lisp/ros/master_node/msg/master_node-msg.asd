
(cl:in-package :asdf)

(defsystem "master_node-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :std_msgs-msg
)
  :components ((:file "_package")
    (:file "MasterNodeState" :depends-on ("_package_MasterNodeState"))
    (:file "_package_MasterNodeState" :depends-on ("_package"))
  ))