;; Auto-generated. Do not edit!


(when (boundp 'snapstack_msgs::CommAge)
  (if (not (find-package "SNAPSTACK_MSGS"))
    (make-package "SNAPSTACK_MSGS"))
  (shadow 'CommAge (find-package "SNAPSTACK_MSGS")))
(unless (find-package "SNAPSTACK_MSGS::COMMAGE")
  (make-package "SNAPSTACK_MSGS::COMMAGE"))

(in-package "ROS")
;;//! \htmlinclude CommAge.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass snapstack_msgs::CommAge
  :super ros::object
  :slots (_header _vicon_age_secs _goal_age_secs ))

(defmethod snapstack_msgs::CommAge
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:vicon_age_secs __vicon_age_secs) 0.0)
    ((:goal_age_secs __goal_age_secs) 0.0)
    )
   (send-super :init)
   (setq _header __header)
   (setq _vicon_age_secs (float __vicon_age_secs))
   (setq _goal_age_secs (float __goal_age_secs))
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:vicon_age_secs
   (&optional __vicon_age_secs)
   (if __vicon_age_secs (setq _vicon_age_secs __vicon_age_secs)) _vicon_age_secs)
  (:goal_age_secs
   (&optional __goal_age_secs)
   (if __goal_age_secs (setq _goal_age_secs __goal_age_secs)) _goal_age_secs)
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; float32 _vicon_age_secs
    4
    ;; float32 _goal_age_secs
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; float32 _vicon_age_secs
       (sys::poke _vicon_age_secs (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _goal_age_secs
       (sys::poke _goal_age_secs (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; float32 _vicon_age_secs
     (setq _vicon_age_secs (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _goal_age_secs
     (setq _goal_age_secs (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get snapstack_msgs::CommAge :md5sum-) "37ee5d091cfb61db7a1dcd668b6244ed")
(setf (get snapstack_msgs::CommAge :datatype-) "snapstack_msgs/CommAge")
(setf (get snapstack_msgs::CommAge :definition-)
      "Header header
float32 vicon_age_secs
float32 goal_age_secs

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

")



(provide :snapstack_msgs/CommAge "37ee5d091cfb61db7a1dcd668b6244ed")


