;; Auto-generated. Do not edit!


(when (boundp 'convex_decomposer::Polyhedron)
  (if (not (find-package "CONVEX_DECOMPOSER"))
    (make-package "CONVEX_DECOMPOSER"))
  (shadow 'Polyhedron (find-package "CONVEX_DECOMPOSER")))
(unless (find-package "CONVEX_DECOMPOSER::POLYHEDRON")
  (make-package "CONVEX_DECOMPOSER::POLYHEDRON"))

(in-package "ROS")
;;//! \htmlinclude Polyhedron.msg.html
(if (not (find-package "SHAPE_MSGS"))
  (ros::roseus-add-msgs "shape_msgs"))


(defclass convex_decomposer::Polyhedron
  :super ros::object
  :slots (_planes ))

(defmethod convex_decomposer::Polyhedron
  (:init
   (&key
    ((:planes __planes) (let (r) (dotimes (i 0) (push (instance shape_msgs::Plane :init) r)) r))
    )
   (send-super :init)
   (setq _planes __planes)
   self)
  (:planes
   (&rest __planes)
   (if (keywordp (car __planes))
       (send* _planes __planes)
     (progn
       (if __planes (setq _planes (car __planes)))
       _planes)))
  (:serialization-length
   ()
   (+
    ;; shape_msgs/Plane[] _planes
    (apply #'+ (send-all _planes :serialization-length)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; shape_msgs/Plane[] _planes
     (write-long (length _planes) s)
     (dolist (elem _planes)
       (send elem :serialize s)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; shape_msgs/Plane[] _planes
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _planes (let (r) (dotimes (i n) (push (instance shape_msgs::Plane :init) r)) r))
     (dolist (elem- _planes)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;;
   self)
  )

(setf (get convex_decomposer::Polyhedron :md5sum-) "18bc595cd5fca2d6b49e654a7e19a442")
(setf (get convex_decomposer::Polyhedron :datatype-) "convex_decomposer/Polyhedron")
(setf (get convex_decomposer::Polyhedron :definition-)
      "shape_msgs/Plane[] planes

================================================================================
MSG: shape_msgs/Plane
# Representation of a plane, using the plane equation ax + by + cz + d = 0

# a := coef[0]
# b := coef[1]
# c := coef[2]
# d := coef[3]

float64[4] coef

")



(provide :convex_decomposer/Polyhedron "18bc595cd5fca2d6b49e654a7e19a442")


