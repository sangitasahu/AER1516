;; Auto-generated. Do not edit!


(when (boundp 'convex_decomposer::CvxDecomp)
  (if (not (find-package "CONVEX_DECOMPOSER"))
    (make-package "CONVEX_DECOMPOSER"))
  (shadow 'CvxDecomp (find-package "CONVEX_DECOMPOSER")))
(unless (find-package "CONVEX_DECOMPOSER::CVXDECOMP")
  (make-package "CONVEX_DECOMPOSER::CVXDECOMP"))

(in-package "ROS")
;;//! \htmlinclude CvxDecomp.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass convex_decomposer::CvxDecomp
  :super ros::object
  :slots (_header _polyhedra ))

(defmethod convex_decomposer::CvxDecomp
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:polyhedra __polyhedra) (let (r) (dotimes (i 0) (push (instance convex_decomposer::Polyhedron :init) r)) r))
    )
   (send-super :init)
   (setq _header __header)
   (setq _polyhedra __polyhedra)
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:polyhedra
   (&rest __polyhedra)
   (if (keywordp (car __polyhedra))
       (send* _polyhedra __polyhedra)
     (progn
       (if __polyhedra (setq _polyhedra (car __polyhedra)))
       _polyhedra)))
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; convex_decomposer/Polyhedron[] _polyhedra
    (apply #'+ (send-all _polyhedra :serialization-length)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; convex_decomposer/Polyhedron[] _polyhedra
     (write-long (length _polyhedra) s)
     (dolist (elem _polyhedra)
       (send elem :serialize s)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; convex_decomposer/Polyhedron[] _polyhedra
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _polyhedra (let (r) (dotimes (i n) (push (instance convex_decomposer::Polyhedron :init) r)) r))
     (dolist (elem- _polyhedra)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;;
   self)
  )

(setf (get convex_decomposer::CvxDecomp :md5sum-) "fa0b44be5161dc2f552ca9c3ef45157c")
(setf (get convex_decomposer::CvxDecomp :datatype-) "convex_decomposer/CvxDecomp")
(setf (get convex_decomposer::CvxDecomp :definition-)
      "std_msgs/Header header
convex_decomposer/Polyhedron[] polyhedra

================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data 
# in a particular coordinate frame.
# 
# sequence ID: consecutively increasing ID 
uint32 seq
#Two-integer timestamp that is expressed as:
# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
# time-handling sugar is provided by the client library
time stamp
#Frame this data is associated with
string frame_id

================================================================================
MSG: convex_decomposer/Polyhedron
shape_msgs/Plane[] planes

================================================================================
MSG: shape_msgs/Plane
# Representation of a plane, using the plane equation ax + by + cz + d = 0

# a := coef[0]
# b := coef[1]
# c := coef[2]
# d := coef[3]

float64[4] coef

")



(provide :convex_decomposer/CvxDecomp "fa0b44be5161dc2f552ca9c3ef45157c")


