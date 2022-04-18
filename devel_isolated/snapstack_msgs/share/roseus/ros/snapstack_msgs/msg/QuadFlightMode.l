;; Auto-generated. Do not edit!


(when (boundp 'snapstack_msgs::QuadFlightMode)
  (if (not (find-package "SNAPSTACK_MSGS"))
    (make-package "SNAPSTACK_MSGS"))
  (shadow 'QuadFlightMode (find-package "SNAPSTACK_MSGS")))
(unless (find-package "SNAPSTACK_MSGS::QUADFLIGHTMODE")
  (make-package "SNAPSTACK_MSGS::QUADFLIGHTMODE"))

(in-package "ROS")
;;//! \htmlinclude QuadFlightMode.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(intern "*NOT_FLYING*" (find-package "SNAPSTACK_MSGS::QUADFLIGHTMODE"))
(shadow '*NOT_FLYING* (find-package "SNAPSTACK_MSGS::QUADFLIGHTMODE"))
(defconstant snapstack_msgs::QuadFlightMode::*NOT_FLYING* 0)
(intern "*TAKEOFF*" (find-package "SNAPSTACK_MSGS::QUADFLIGHTMODE"))
(shadow '*TAKEOFF* (find-package "SNAPSTACK_MSGS::QUADFLIGHTMODE"))
(defconstant snapstack_msgs::QuadFlightMode::*TAKEOFF* 1)
(intern "*LAND*" (find-package "SNAPSTACK_MSGS::QUADFLIGHTMODE"))
(shadow '*LAND* (find-package "SNAPSTACK_MSGS::QUADFLIGHTMODE"))
(defconstant snapstack_msgs::QuadFlightMode::*LAND* 2)
(intern "*INIT*" (find-package "SNAPSTACK_MSGS::QUADFLIGHTMODE"))
(shadow '*INIT* (find-package "SNAPSTACK_MSGS::QUADFLIGHTMODE"))
(defconstant snapstack_msgs::QuadFlightMode::*INIT* 3)
(intern "*GO*" (find-package "SNAPSTACK_MSGS::QUADFLIGHTMODE"))
(shadow '*GO* (find-package "SNAPSTACK_MSGS::QUADFLIGHTMODE"))
(defconstant snapstack_msgs::QuadFlightMode::*GO* 4)
(intern "*ESTOP*" (find-package "SNAPSTACK_MSGS::QUADFLIGHTMODE"))
(shadow '*ESTOP* (find-package "SNAPSTACK_MSGS::QUADFLIGHTMODE"))
(defconstant snapstack_msgs::QuadFlightMode::*ESTOP* 5)
(intern "*KILL*" (find-package "SNAPSTACK_MSGS::QUADFLIGHTMODE"))
(shadow '*KILL* (find-package "SNAPSTACK_MSGS::QUADFLIGHTMODE"))
(defconstant snapstack_msgs::QuadFlightMode::*KILL* 6)
(defclass snapstack_msgs::QuadFlightMode
  :super ros::object
  :slots (_header _mode ))

(defmethod snapstack_msgs::QuadFlightMode
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

(setf (get snapstack_msgs::QuadFlightMode :md5sum-) "e5d2af0214158b4bd2c1a618cd003a25")
(setf (get snapstack_msgs::QuadFlightMode :datatype-) "snapstack_msgs/QuadFlightMode")
(setf (get snapstack_msgs::QuadFlightMode :definition-)
      "Header header
uint8 mode
uint8 NOT_FLYING   = 0
uint8 TAKEOFF      = 1
uint8 LAND	       = 2
uint8 INIT 	       = 3
uint8 GO		   = 4
uint8 ESTOP        = 5
uint8 KILL		   = 6

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



(provide :snapstack_msgs/QuadFlightMode "e5d2af0214158b4bd2c1a618cd003a25")


