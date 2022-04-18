;; Auto-generated. Do not edit!


(when (boundp 'master_node::MasterNodeState)
  (if (not (find-package "MASTER_NODE"))
    (make-package "MASTER_NODE"))
  (shadow 'MasterNodeState (find-package "MASTER_NODE")))
(unless (find-package "MASTER_NODE::MASTERNODESTATE")
  (make-package "MASTER_NODE::MASTERNODESTATE"))

(in-package "ROS")
;;//! \htmlinclude MasterNodeState.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(intern "*IDLE*" (find-package "MASTER_NODE::MASTERNODESTATE"))
(shadow '*IDLE* (find-package "MASTER_NODE::MASTERNODESTATE"))
(defconstant master_node::MasterNodeState::*IDLE* 0)
(intern "*TAKEOFF*" (find-package "MASTER_NODE::MASTERNODESTATE"))
(shadow '*TAKEOFF* (find-package "MASTER_NODE::MASTERNODESTATE"))
(defconstant master_node::MasterNodeState::*TAKEOFF* 1)
(intern "*FLIGHT_LOCAL*" (find-package "MASTER_NODE::MASTERNODESTATE"))
(shadow '*FLIGHT_LOCAL* (find-package "MASTER_NODE::MASTERNODESTATE"))
(defconstant master_node::MasterNodeState::*FLIGHT_LOCAL* 2)
(intern "*FLIGHT_GLOBAL*" (find-package "MASTER_NODE::MASTERNODESTATE"))
(shadow '*FLIGHT_GLOBAL* (find-package "MASTER_NODE::MASTERNODESTATE"))
(defconstant master_node::MasterNodeState::*FLIGHT_GLOBAL* 3)
(intern "*FLIGHT_HOLD*" (find-package "MASTER_NODE::MASTERNODESTATE"))
(shadow '*FLIGHT_HOLD* (find-package "MASTER_NODE::MASTERNODESTATE"))
(defconstant master_node::MasterNodeState::*FLIGHT_HOLD* 4)
(intern "*UNKNOWN*" (find-package "MASTER_NODE::MASTERNODESTATE"))
(shadow '*UNKNOWN* (find-package "MASTER_NODE::MASTERNODESTATE"))
(defconstant master_node::MasterNodeState::*UNKNOWN* 10)
(defclass master_node::MasterNodeState
  :super ros::object
  :slots (_header _state ))

(defmethod master_node::MasterNodeState
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:state __state) 0)
    )
   (send-super :init)
   (setq _header __header)
   (setq _state (round __state))
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:state
   (&optional __state)
   (if __state (setq _state __state)) _state)
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; int16 _state
    2
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; int16 _state
       (write-word _state s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; int16 _state
     (setq _state (sys::peek buf ptr- :short)) (incf ptr- 2)
   ;;
   self)
  )

(setf (get master_node::MasterNodeState :md5sum-) "94ab713c34436e1423557ad1082b0f2c")
(setf (get master_node::MasterNodeState :datatype-) "master_node/MasterNodeState")
(setf (get master_node::MasterNodeState :definition-)
      "std_msgs/Header header
int16 IDLE = 0
int16 TAKEOFF = 1
int16 FLIGHT_LOCAL = 2
int16 FLIGHT_GLOBAL = 3
int16 FLIGHT_HOLD = 4
int16 UNKNOWN = 10
int16 state
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



(provide :master_node/MasterNodeState "94ab713c34436e1423557ad1082b0f2c")


