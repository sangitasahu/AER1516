; Auto-generated. Do not edit!


(cl:in-package fla_msgs-msg)


;//! \htmlinclude FlightEvent.msg.html

(cl:defclass <FlightEvent> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (event_id
    :reader event_id
    :initarg :event_id
    :type cl:fixnum
    :initform 0))
)

(cl:defclass FlightEvent (<FlightEvent>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <FlightEvent>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'FlightEvent)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name fla_msgs-msg:<FlightEvent> is deprecated: use fla_msgs-msg:FlightEvent instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <FlightEvent>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader fla_msgs-msg:header-val is deprecated.  Use fla_msgs-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'event_id-val :lambda-list '(m))
(cl:defmethod event_id-val ((m <FlightEvent>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader fla_msgs-msg:event_id-val is deprecated.  Use fla_msgs-msg:event_id instead.")
  (event_id m))
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql '<FlightEvent>)))
    "Constants for message type '<FlightEvent>"
  '((:BEGIN . 0)
    (:INITIALIZED . 1)
    (:ALTITUDE_STABLE . 2)
    (:GOAL_REACHED . 3)
    (:LAND_COMMANDED . 4)
    (:HOVER_COMMANDED . 5)
    (:RESET_COMMANDED . 6)
    (:LANDED . 7)
    (:RC_COMM_LOSS . 8)
    (:TARGET_DETECTED_OUT_OF_RANGE . 9)
    (:TARGET_DETECTED_IN_RANGE . 10)
    (:SEARCH_COMPLETE . 11)
    (:RTL_COMMANDED . 12)
    (:FLIGHTEVENT_MAX . 13))
)
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql 'FlightEvent)))
    "Constants for message type 'FlightEvent"
  '((:BEGIN . 0)
    (:INITIALIZED . 1)
    (:ALTITUDE_STABLE . 2)
    (:GOAL_REACHED . 3)
    (:LAND_COMMANDED . 4)
    (:HOVER_COMMANDED . 5)
    (:RESET_COMMANDED . 6)
    (:LANDED . 7)
    (:RC_COMM_LOSS . 8)
    (:TARGET_DETECTED_OUT_OF_RANGE . 9)
    (:TARGET_DETECTED_IN_RANGE . 10)
    (:SEARCH_COMPLETE . 11)
    (:RTL_COMMANDED . 12)
    (:FLIGHTEVENT_MAX . 13))
)
(cl:defmethod roslisp-msg-protocol:serialize ((msg <FlightEvent>) ostream)
  "Serializes a message object of type '<FlightEvent>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'event_id)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <FlightEvent>) istream)
  "Deserializes a message object of type '<FlightEvent>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'event_id)) (cl:read-byte istream))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<FlightEvent>)))
  "Returns string type for a message object of type '<FlightEvent>"
  "fla_msgs/FlightEvent")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'FlightEvent)))
  "Returns string type for a message object of type 'FlightEvent"
  "fla_msgs/FlightEvent")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<FlightEvent>)))
  "Returns md5sum for a message object of type '<FlightEvent>"
  "4110787c7e1d3f4e3706ce25b95d01e2")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'FlightEvent)))
  "Returns md5sum for a message object of type 'FlightEvent"
  "4110787c7e1d3f4e3706ce25b95d01e2")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<FlightEvent>)))
  "Returns full string definition for message of type '<FlightEvent>"
  (cl:format cl:nil "uint8 BEGIN=0~%uint8 INITIALIZED=1~%uint8 ALTITUDE_STABLE=2~%uint8 GOAL_REACHED=3~%uint8 LAND_COMMANDED=4~%uint8 HOVER_COMMANDED=5~%uint8 RESET_COMMANDED=6~%uint8 LANDED=7~%uint8 RC_COMM_LOSS=8~%uint8 TARGET_DETECTED_OUT_OF_RANGE=9~%uint8 TARGET_DETECTED_IN_RANGE=10~%uint8 SEARCH_COMPLETE=11~%uint8 RTL_COMMANDED=12~%uint8 FLIGHTEVENT_MAX=13~%std_msgs/Header header~%uint8 event_id~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'FlightEvent)))
  "Returns full string definition for message of type 'FlightEvent"
  (cl:format cl:nil "uint8 BEGIN=0~%uint8 INITIALIZED=1~%uint8 ALTITUDE_STABLE=2~%uint8 GOAL_REACHED=3~%uint8 LAND_COMMANDED=4~%uint8 HOVER_COMMANDED=5~%uint8 RESET_COMMANDED=6~%uint8 LANDED=7~%uint8 RC_COMM_LOSS=8~%uint8 TARGET_DETECTED_OUT_OF_RANGE=9~%uint8 TARGET_DETECTED_IN_RANGE=10~%uint8 SEARCH_COMPLETE=11~%uint8 RTL_COMMANDED=12~%uint8 FLIGHTEVENT_MAX=13~%std_msgs/Header header~%uint8 event_id~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <FlightEvent>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <FlightEvent>))
  "Converts a ROS message object to a list"
  (cl:list 'FlightEvent
    (cl:cons ':header (header msg))
    (cl:cons ':event_id (event_id msg))
))
