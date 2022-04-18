
(cl:in-package :asdf)

(defsystem "convex_decomposer-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :shape_msgs-msg
               :std_msgs-msg
)
  :components ((:file "_package")
    (:file "CvxDecomp" :depends-on ("_package_CvxDecomp"))
    (:file "_package_CvxDecomp" :depends-on ("_package"))
    (:file "Polyhedron" :depends-on ("_package_Polyhedron"))
    (:file "_package_Polyhedron" :depends-on ("_package"))
  ))