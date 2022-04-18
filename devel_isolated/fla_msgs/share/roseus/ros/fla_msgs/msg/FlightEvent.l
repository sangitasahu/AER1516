;; Auto-generated. Do not edit!


(when (boundp 'fla_msgs::FlightEvent)
  (if (not (find-package "FLA_MSGS"))
    (make-package "FLA_MSGS"))
  (shadow 'FlightEvent (find-package "FLA_MSGS")))
(unless (find-package "FLA_MSGS::FLIGHTEVENT")
  (make-package "FLA_MSGS::FLIGHTEVENT"))

(in-package "ROS")
;;//! \htmlinclude FlightEvent.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(intern "*BEGIN*" (find-package "FLA_MSGS::FLIGHTEVENT"))
(shadow '*BEGIN* (find-package "FLA_MSGS::FLIGHTEVENT"))
(defconstant fla_msgs::FlightEvent::*BEGIN* 0)
(intern "*INITIALIZED*" (find-package "FLA_MSGS::FLIGHTEVENT"))
(shadow '*INITIALIZED* (find-package "FLA_MSGS::FLIGHTEVENT"))
(defconstant fla_msgs::FlightEvent::*INITIALIZED* 1)
(intern "*ALTITUDE_STABLE*" (find-package "FLA_MSGS::FLIGHTEVENT"))
(shadow '*ALTITUDE_STABLE* (find-package "FLA_MSGS::FLIGHTEVENT"))
(defconstant fla_msgs::FlightEvent::*ALTITUDE_STABLE* 2)
(intern "*GOAL_REACHED*" (find-package "FLA_MSGS::FLIGHTEVENT"))
(shadow '*GOAL_REACHED* (find-package "FLA_MSGS::FLIGHTEVENT"))
(defconstant fla_msgs::FlightEvent::*GOAL_REACHED* 3)
(intern "*LAND_COMMANDED*" (find-package "FLA_MSGS::FLIGHTEVENT"))
(shadow '*LAND_COMMANDED* (find-package "FLA_MSGS::FLIGHTEVENT"))
(defconstant fla_msgs::FlightEvent::*LAND_COMMANDED* 4)
(intern "*HOVER_COMMANDED*" (find-package "FLA_MSGS::FLIGHTEVENT"))
(shadow '*HOVER_COMMANDED* (find-package "FLA_MSGS::FLIGHTEVENT"))
(defconstant fla_msgs::FlightEvent::*HOVER_COMMANDED* 5)
(intern "*RESET_COMMANDED*" (find-package "FLA_MSGS::FLIGHTEVENT"))
(shadow '*RESET_COMMANDED* (find-package "FLA_MSGS::FLIGHTEVENT"))
(defconstant fla_msgs::FlightEvent::*RESET_COMMANDED* 6)
(intern "*LANDED*" (find-package "FLA_MSGS::FLIGHTEVENT"))
(shadow '*LANDED* (find-package "FLA_MSGS::FLIGHTEVENT"))
(defconstant fla_msgs::FlightEvent::*LANDED* 7)
(intern "*RC_COMM_LOSS*" (find-package "FLA_MSGS::FLIGHTEVENT"))
(shadow '*RC_COMM_LOSS* (find-package "FLA_MSGS::FLIGHTEVENT"))
(defconstant fla_msgs::FlightEvent::*RC_COMM_LOSS* 8)
(intern "*TARGET_DETECTED_OUT_OF_RANGE*" (find-package "FLA_MSGS::FLIGHTEVENT"))
(shadow '*TARGET_DETECTED_OUT_OF_RANGE* (find-package "FLA_MSGS::FLIGHTEVENT"))
(defconstant fla_msgs::FlightEvent::*TARGET_DETECTED_OUT_OF_RANGE* 9)
(intern "*TARGET_DETECTED_IN_RANGE*" (find-package "FLA_MSGS::FLIGHTEVENT"))
(shadow '*TARGET_DETECTED_IN_RANGE* (find-package "FLA_MSGS::FLIGHTEVENT"))
(defconstant fla_msgs::FlightEvent::*TARGET_DETECTED_IN_RANGE* 10)
(intern "*SEARCH_COMPLETE*" (find-package "FLA_MSGS::FLIGHTEVENT"))
(shadow '*SEARCH_COMPLETE* (find-package "FLA_MSGS::FLIGHTEVENT"))
(defconstant fla_msgs::FlightEvent::*SEARCH_COMPLETE* 11)
(intern "*RTL_COMMANDED*" (find-package "FLA_MSGS::FLIGHTEVENT"))
(shadow '*RTL_COMMANDED* (find-package "FLA_MSGS::FLIGHTEVENT"))
(defconstant fla_msgs::FlightEvent::*RTL_COMMANDED* 12)
(intern "*FLIGHTEVENT_MAX*" (find-package "FLA_MSGS::FLIGHTEVENT"))
(shadow '*FLIGHTEVENT_MAX* (find-package "FLA_MSGS::FLIGHTEVENT"))
(defconstant fla_msgs::FlightEvent::*FLIGHTEVENT_MAX* 13)
(defclass fla_msgs::FlightEvent
  :super ros::object
  :slots (_header _event_id ))

(defmethod fla_msgs::FlightEvent
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:event_id __event_id) 0)
    )
   (send-super :init)
   (setq _header __header)
   (setq _event_id (round __event_id))
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:event_id
   (&optional __event_id)
   (if __event_id (setq _event_id __event_id)) _event_id)
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; uint8 _event_id
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; uint8 _event_id
       (write-byte _event_id s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; uint8 _event_id
     (setq _event_id (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;;
   self)
  )

(setf (get fla_msgs::FlightEvent :md5sum-) "4110787c7e1d3f4e3706ce25b95d01e2")
(setf (get fla_msgs::FlightEvent :datatype-) "fla_msgs/FlightEvent")
(setf (get fla_msgs::FlightEvent :definition-)
      "uint8 BEGIN=0
uint8 INITIALIZED=1
uint8 ALTITUDE_STABLE=2
uint8 GOAL_REACHED=3
uint8 LAND_COMMANDED=4
uint8 HOVER_COMMANDED=5
uint8 RESET_COMMANDED=6
uint8 LANDED=7
uint8 RC_COMM_LOSS=8
uint8 TARGET_DETECTED_OUT_OF_RANGE=9
uint8 TARGET_DETECTED_IN_RANGE=10
uint8 SEARCH_COMPLETE=11
uint8 RTL_COMMANDED=12
uint8 FLIGHTEVENT_MAX=13
std_msgs/Header header
uint8 event_id

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



(provide :fla_msgs/FlightEvent "4110787c7e1d3f4e3706ce25b95d01e2")


