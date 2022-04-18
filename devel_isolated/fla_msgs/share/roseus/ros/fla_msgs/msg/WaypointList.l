;; Auto-generated. Do not edit!


(when (boundp 'fla_msgs::WaypointList)
  (if (not (find-package "FLA_MSGS"))
    (make-package "FLA_MSGS"))
  (shadow 'WaypointList (find-package "FLA_MSGS")))
(unless (find-package "FLA_MSGS::WAYPOINTLIST")
  (make-package "FLA_MSGS::WAYPOINTLIST"))

(in-package "ROS")
;;//! \htmlinclude WaypointList.msg.html
(if (not (find-package "GEOMETRY_MSGS"))
  (ros::roseus-add-msgs "geometry_msgs"))
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass fla_msgs::WaypointList
  :super ros::object
  :slots (_header _poses _speeds ))

(defmethod fla_msgs::WaypointList
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:poses __poses) (let (r) (dotimes (i 0) (push (instance geometry_msgs::PoseStamped :init) r)) r))
    ((:speeds __speeds) (let (r) (dotimes (i 0) (push (instance std_msgs::Float64 :init) r)) r))
    )
   (send-super :init)
   (setq _header __header)
   (setq _poses __poses)
   (setq _speeds __speeds)
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:poses
   (&rest __poses)
   (if (keywordp (car __poses))
       (send* _poses __poses)
     (progn
       (if __poses (setq _poses (car __poses)))
       _poses)))
  (:speeds
   (&rest __speeds)
   (if (keywordp (car __speeds))
       (send* _speeds __speeds)
     (progn
       (if __speeds (setq _speeds (car __speeds)))
       _speeds)))
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; geometry_msgs/PoseStamped[] _poses
    (apply #'+ (send-all _poses :serialization-length)) 4
    ;; std_msgs/Float64[] _speeds
    (apply #'+ (send-all _speeds :serialization-length)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; geometry_msgs/PoseStamped[] _poses
     (write-long (length _poses) s)
     (dolist (elem _poses)
       (send elem :serialize s)
       )
     ;; std_msgs/Float64[] _speeds
     (write-long (length _speeds) s)
     (dolist (elem _speeds)
       (send elem :serialize s)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; geometry_msgs/PoseStamped[] _poses
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _poses (let (r) (dotimes (i n) (push (instance geometry_msgs::PoseStamped :init) r)) r))
     (dolist (elem- _poses)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;; std_msgs/Float64[] _speeds
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _speeds (let (r) (dotimes (i n) (push (instance std_msgs::Float64 :init) r)) r))
     (dolist (elem- _speeds)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;;
   self)
  )

(setf (get fla_msgs::WaypointList :md5sum-) "0d705e35e06b0f87f64099aeaa643811")
(setf (get fla_msgs::WaypointList :datatype-) "fla_msgs/WaypointList")
(setf (get fla_msgs::WaypointList :definition-)
      "std_msgs/Header header
geometry_msgs/PoseStamped[] poses
std_msgs/Float64[] speeds

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
MSG: geometry_msgs/PoseStamped
# A Pose with reference coordinate frame and timestamp
Header header
Pose pose

================================================================================
MSG: geometry_msgs/Pose
# A representation of pose in free space, composed of position and orientation. 
Point position
Quaternion orientation

================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z

================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x
float64 y
float64 z
float64 w

================================================================================
MSG: std_msgs/Float64
float64 data
")



(provide :fla_msgs/WaypointList "0d705e35e06b0f87f64099aeaa643811")


