;; Auto-generated. Do not edit!


(when (boundp 'snapstack_msgs::Motors)
  (if (not (find-package "SNAPSTACK_MSGS"))
    (make-package "SNAPSTACK_MSGS"))
  (shadow 'Motors (find-package "SNAPSTACK_MSGS")))
(unless (find-package "SNAPSTACK_MSGS::MOTORS")
  (make-package "SNAPSTACK_MSGS::MOTORS"))

(in-package "ROS")
;;//! \htmlinclude Motors.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass snapstack_msgs::Motors
  :super ros::object
  :slots (_header _m1 _m2 _m3 _m4 _m5 _m6 _m7 _m8 ))

(defmethod snapstack_msgs::Motors
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:m1 __m1) 0.0)
    ((:m2 __m2) 0.0)
    ((:m3 __m3) 0.0)
    ((:m4 __m4) 0.0)
    ((:m5 __m5) 0.0)
    ((:m6 __m6) 0.0)
    ((:m7 __m7) 0.0)
    ((:m8 __m8) 0.0)
    )
   (send-super :init)
   (setq _header __header)
   (setq _m1 (float __m1))
   (setq _m2 (float __m2))
   (setq _m3 (float __m3))
   (setq _m4 (float __m4))
   (setq _m5 (float __m5))
   (setq _m6 (float __m6))
   (setq _m7 (float __m7))
   (setq _m8 (float __m8))
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:m1
   (&optional __m1)
   (if __m1 (setq _m1 __m1)) _m1)
  (:m2
   (&optional __m2)
   (if __m2 (setq _m2 __m2)) _m2)
  (:m3
   (&optional __m3)
   (if __m3 (setq _m3 __m3)) _m3)
  (:m4
   (&optional __m4)
   (if __m4 (setq _m4 __m4)) _m4)
  (:m5
   (&optional __m5)
   (if __m5 (setq _m5 __m5)) _m5)
  (:m6
   (&optional __m6)
   (if __m6 (setq _m6 __m6)) _m6)
  (:m7
   (&optional __m7)
   (if __m7 (setq _m7 __m7)) _m7)
  (:m8
   (&optional __m8)
   (if __m8 (setq _m8 __m8)) _m8)
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; float64 _m1
    8
    ;; float64 _m2
    8
    ;; float64 _m3
    8
    ;; float64 _m4
    8
    ;; float64 _m5
    8
    ;; float64 _m6
    8
    ;; float64 _m7
    8
    ;; float64 _m8
    8
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; float64 _m1
       (sys::poke _m1 (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _m2
       (sys::poke _m2 (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _m3
       (sys::poke _m3 (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _m4
       (sys::poke _m4 (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _m5
       (sys::poke _m5 (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _m6
       (sys::poke _m6 (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _m7
       (sys::poke _m7 (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _m8
       (sys::poke _m8 (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; float64 _m1
     (setq _m1 (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _m2
     (setq _m2 (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _m3
     (setq _m3 (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _m4
     (setq _m4 (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _m5
     (setq _m5 (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _m6
     (setq _m6 (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _m7
     (setq _m7 (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _m8
     (setq _m8 (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;;
   self)
  )

(setf (get snapstack_msgs::Motors :md5sum-) "2f7b2f19c4fbfd9dffd8b02620937b0a")
(setf (get snapstack_msgs::Motors :datatype-) "snapstack_msgs/Motors")
(setf (get snapstack_msgs::Motors :definition-)
      "Header header
float64 m1
float64 m2
float64 m3
float64 m4
float64 m5
float64 m6
float64 m7
float64 m8

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



(provide :snapstack_msgs/Motors "2f7b2f19c4fbfd9dffd8b02620937b0a")


