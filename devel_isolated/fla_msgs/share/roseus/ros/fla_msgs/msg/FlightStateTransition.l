;; Auto-generated. Do not edit!


(when (boundp 'fla_msgs::FlightStateTransition)
  (if (not (find-package "FLA_MSGS"))
    (make-package "FLA_MSGS"))
  (shadow 'FlightStateTransition (find-package "FLA_MSGS")))
(unless (find-package "FLA_MSGS::FLIGHTSTATETRANSITION")
  (make-package "FLA_MSGS::FLIGHTSTATETRANSITION"))

(in-package "ROS")
;;//! \htmlinclude FlightStateTransition.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(intern "*IDLE*" (find-package "FLA_MSGS::FLIGHTSTATETRANSITION"))
(shadow '*IDLE* (find-package "FLA_MSGS::FLIGHTSTATETRANSITION"))
(defconstant fla_msgs::FlightStateTransition::*IDLE* 0)
(intern "*INITIALIZING*" (find-package "FLA_MSGS::FLIGHTSTATETRANSITION"))
(shadow '*INITIALIZING* (find-package "FLA_MSGS::FLIGHTSTATETRANSITION"))
(defconstant fla_msgs::FlightStateTransition::*INITIALIZING* 1)
(intern "*TAKING_OFF*" (find-package "FLA_MSGS::FLIGHTSTATETRANSITION"))
(shadow '*TAKING_OFF* (find-package "FLA_MSGS::FLIGHTSTATETRANSITION"))
(defconstant fla_msgs::FlightStateTransition::*TAKING_OFF* 2)
(intern "*FOLLOWING_PATH*" (find-package "FLA_MSGS::FLIGHTSTATETRANSITION"))
(shadow '*FOLLOWING_PATH* (find-package "FLA_MSGS::FLIGHTSTATETRANSITION"))
(defconstant fla_msgs::FlightStateTransition::*FOLLOWING_PATH* 3)
(intern "*HOVERING*" (find-package "FLA_MSGS::FLIGHTSTATETRANSITION"))
(shadow '*HOVERING* (find-package "FLA_MSGS::FLIGHTSTATETRANSITION"))
(defconstant fla_msgs::FlightStateTransition::*HOVERING* 4)
(intern "*LANDING*" (find-package "FLA_MSGS::FLIGHTSTATETRANSITION"))
(shadow '*LANDING* (find-package "FLA_MSGS::FLIGHTSTATETRANSITION"))
(defconstant fla_msgs::FlightStateTransition::*LANDING* 5)
(intern "*RETURN_TO_LAUNCH*" (find-package "FLA_MSGS::FLIGHTSTATETRANSITION"))
(shadow '*RETURN_TO_LAUNCH* (find-package "FLA_MSGS::FLIGHTSTATETRANSITION"))
(defconstant fla_msgs::FlightStateTransition::*RETURN_TO_LAUNCH* 6)
(intern "*LANDED*" (find-package "FLA_MSGS::FLIGHTSTATETRANSITION"))
(shadow '*LANDED* (find-package "FLA_MSGS::FLIGHTSTATETRANSITION"))
(defconstant fla_msgs::FlightStateTransition::*LANDED* 7)
(intern "*TARGET_SEARCH*" (find-package "FLA_MSGS::FLIGHTSTATETRANSITION"))
(shadow '*TARGET_SEARCH* (find-package "FLA_MSGS::FLIGHTSTATETRANSITION"))
(defconstant fla_msgs::FlightStateTransition::*TARGET_SEARCH* 8)
(intern "*TARGET_APPROACH*" (find-package "FLA_MSGS::FLIGHTSTATETRANSITION"))
(shadow '*TARGET_APPROACH* (find-package "FLA_MSGS::FLIGHTSTATETRANSITION"))
(defconstant fla_msgs::FlightStateTransition::*TARGET_APPROACH* 9)
(intern "*LANDING_SEARCH*" (find-package "FLA_MSGS::FLIGHTSTATETRANSITION"))
(shadow '*LANDING_SEARCH* (find-package "FLA_MSGS::FLIGHTSTATETRANSITION"))
(defconstant fla_msgs::FlightStateTransition::*LANDING_SEARCH* 10)
(intern "*LANDING_APPROACH*" (find-package "FLA_MSGS::FLIGHTSTATETRANSITION"))
(shadow '*LANDING_APPROACH* (find-package "FLA_MSGS::FLIGHTSTATETRANSITION"))
(defconstant fla_msgs::FlightStateTransition::*LANDING_APPROACH* 11)
(defclass fla_msgs::FlightStateTransition
  :super ros::object
  :slots (_header _from_state _to_state _from_state_id _to_state_id ))

(defmethod fla_msgs::FlightStateTransition
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:from_state __from_state) (instance std_msgs::String :init))
    ((:to_state __to_state) (instance std_msgs::String :init))
    ((:from_state_id __from_state_id) 0)
    ((:to_state_id __to_state_id) 0)
    )
   (send-super :init)
   (setq _header __header)
   (setq _from_state __from_state)
   (setq _to_state __to_state)
   (setq _from_state_id (round __from_state_id))
   (setq _to_state_id (round __to_state_id))
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:from_state
   (&rest __from_state)
   (if (keywordp (car __from_state))
       (send* _from_state __from_state)
     (progn
       (if __from_state (setq _from_state (car __from_state)))
       _from_state)))
  (:to_state
   (&rest __to_state)
   (if (keywordp (car __to_state))
       (send* _to_state __to_state)
     (progn
       (if __to_state (setq _to_state (car __to_state)))
       _to_state)))
  (:from_state_id
   (&optional __from_state_id)
   (if __from_state_id (setq _from_state_id __from_state_id)) _from_state_id)
  (:to_state_id
   (&optional __to_state_id)
   (if __to_state_id (setq _to_state_id __to_state_id)) _to_state_id)
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; std_msgs/String _from_state
    (send _from_state :serialization-length)
    ;; std_msgs/String _to_state
    (send _to_state :serialization-length)
    ;; uint8 _from_state_id
    1
    ;; uint8 _to_state_id
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; std_msgs/String _from_state
       (send _from_state :serialize s)
     ;; std_msgs/String _to_state
       (send _to_state :serialize s)
     ;; uint8 _from_state_id
       (write-byte _from_state_id s)
     ;; uint8 _to_state_id
       (write-byte _to_state_id s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; std_msgs/String _from_state
     (send _from_state :deserialize buf ptr-) (incf ptr- (send _from_state :serialization-length))
   ;; std_msgs/String _to_state
     (send _to_state :deserialize buf ptr-) (incf ptr- (send _to_state :serialization-length))
   ;; uint8 _from_state_id
     (setq _from_state_id (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; uint8 _to_state_id
     (setq _to_state_id (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;;
   self)
  )

(setf (get fla_msgs::FlightStateTransition :md5sum-) "994f9c657664b2f00c832165b7503f8a")
(setf (get fla_msgs::FlightStateTransition :datatype-) "fla_msgs/FlightStateTransition")
(setf (get fla_msgs::FlightStateTransition :definition-)
      "uint8 IDLE=0
uint8 INITIALIZING=1
uint8 TAKING_OFF=2
uint8 FOLLOWING_PATH=3
uint8 HOVERING=4
uint8 LANDING=5
uint8 RETURN_TO_LAUNCH=6
uint8 LANDED=7
uint8 TARGET_SEARCH=8
uint8 TARGET_APPROACH=9
uint8 LANDING_SEARCH=10
uint8 LANDING_APPROACH=11

std_msgs/Header header
std_msgs/String from_state
std_msgs/String to_state
uint8 from_state_id
uint8 to_state_id

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
MSG: std_msgs/String
string data

")



(provide :fla_msgs/FlightStateTransition "994f9c657664b2f00c832165b7503f8a")


