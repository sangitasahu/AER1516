;; Auto-generated. Do not edit!


(when (boundp 'faster_msgs::Mode)
  (if (not (find-package "FASTER_MSGS"))
    (make-package "FASTER_MSGS"))
  (shadow 'Mode (find-package "FASTER_MSGS")))
(unless (find-package "FASTER_MSGS::MODE")
  (make-package "FASTER_MSGS::MODE"))

(in-package "ROS")
;;//! \htmlinclude Mode.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(intern "*ON_GROUND*" (find-package "FASTER_MSGS::MODE"))
(shadow '*ON_GROUND* (find-package "FASTER_MSGS::MODE"))
(defconstant faster_msgs::Mode::*ON_GROUND* 0)
(intern "*GO*" (find-package "FASTER_MSGS::MODE"))
(shadow '*GO* (find-package "FASTER_MSGS::MODE"))
(defconstant faster_msgs::Mode::*GO* 1)
(defclass faster_msgs::Mode
  :super ros::object
  :slots (_header _mode ))

(defmethod faster_msgs::Mode
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:mode __mode) 0)
    )
   (send-super :init)
   (setq _header __header)
   (setq _mode (round __mode))
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:mode
   (&optional __mode)
   (if __mode (setq _mode __mode)) _mode)
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; uint8 _mode
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; uint8 _mode
       (write-byte _mode s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; uint8 _mode
     (setq _mode (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;;
   self)
  )

(setf (get faster_msgs::Mode :md5sum-) "46808a00b0dbe958093b017bcb10cf21")
(setf (get faster_msgs::Mode :datatype-) "faster_msgs/Mode")
(setf (get faster_msgs::Mode :definition-)
      "Header header
uint8 mode

uint8 ON_GROUND   = 0
uint8 GO	      = 1


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



(provide :faster_msgs/Mode "46808a00b0dbe958093b017bcb10cf21")


