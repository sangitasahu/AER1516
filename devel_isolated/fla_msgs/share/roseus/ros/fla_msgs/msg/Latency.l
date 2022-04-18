;; Auto-generated. Do not edit!


(when (boundp 'fla_msgs::Latency)
  (if (not (find-package "FLA_MSGS"))
    (make-package "FLA_MSGS"))
  (shadow 'Latency (find-package "FLA_MSGS")))
(unless (find-package "FLA_MSGS::LATENCY")
  (make-package "FLA_MSGS::LATENCY"))

(in-package "ROS")
;;//! \htmlinclude Latency.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass fla_msgs::Latency
  :super ros::object
  :slots (_header _latency ))

(defmethod fla_msgs::Latency
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:latency __latency) 0.0)
    )
   (send-super :init)
   (setq _header __header)
   (setq _latency (float __latency))
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:latency
   (&optional __latency)
   (if __latency (setq _latency __latency)) _latency)
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; float64 _latency
    8
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; float64 _latency
       (sys::poke _latency (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; float64 _latency
     (setq _latency (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;;
   self)
  )

(setf (get fla_msgs::Latency :md5sum-) "7d52a49060461c16e2604d5e22a14f13")
(setf (get fla_msgs::Latency :datatype-) "fla_msgs/Latency")
(setf (get fla_msgs::Latency :definition-)
      "Header header
float64 latency

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



(provide :fla_msgs/Latency "7d52a49060461c16e2604d5e22a14f13")


