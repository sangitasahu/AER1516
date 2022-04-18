; Auto-generated. Do not edit!


(cl:in-package fla_msgs-msg)


;//! \htmlinclude FlightStateTransition.msg.html

(cl:defclass <FlightStateTransition> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (from_state
    :reader from_state
    :initarg :from_state
    :type std_msgs-msg:String
    :initform (cl:make-instance 'std_msgs-msg:String))
   (to_state
    :reader to_state
    :initarg :to_state
    :type std_msgs-msg:String
    :initform (cl:make-instance 'std_msgs-msg:String))
   (from_state_id
    :reader from_state_id
    :initarg :from_state_id
    :type cl:fixnum
    :initform 0)
   (to_state_id
    :reader to_state_id
    :initarg :to_state_id
    :type cl:fixnum
    :initform 0))
)

(cl:defclass FlightStateTransition (<FlightStateTransition>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <FlightStateTransition>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'FlightStateTransition)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name fla_msgs-msg:<FlightStateTransition> is deprecated: use fla_msgs-msg:FlightStateTransition instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <FlightStateTransition>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader fla_msgs-msg:header-val is deprecated.  Use fla_msgs-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'from_state-val :lambda-list '(m))
(cl:defmethod from_state-val ((m <FlightStateTransition>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader fla_msgs-msg:from_state-val is deprecated.  Use fla_msgs-msg:from_state instead.")
  (from_state m))

(cl:ensure-generic-function 'to_state-val :lambda-list '(m))
(cl:defmethod to_state-val ((m <FlightStateTransition>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader fla_msgs-msg:to_state-val is deprecated.  Use fla_msgs-msg:to_state instead.")
  (to_state m))

(cl:ensure-generic-function 'from_state_id-val :lambda-list '(m))
(cl:defmethod from_state_id-val ((m <FlightStateTransition>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader fla_msgs-msg:from_state_id-val is deprecated.  Use fla_msgs-msg:from_state_id instead.")
  (from_state_id m))

(cl:ensure-generic-function 'to_state_id-val :lambda-list '(m))
(cl:defmethod to_state_id-val ((m <FlightStateTransition>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader fla_msgs-msg:to_state_id-val is deprecated.  Use fla_msgs-msg:to_state_id instead.")
  (to_state_id m))
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql '<FlightStateTransition>)))
    "Constants for message type '<FlightStateTransition>"
  '((:IDLE . 0)
    (:INITIALIZING . 1)
    (:TAKING_OFF . 2)
    (:FOLLOWING_PATH . 3)
    (:HOVERING . 4)
    (:LANDING . 5)
    (:RETURN_TO_LAUNCH . 6)
    (:LANDED . 7)
    (:TARGET_SEARCH . 8)
    (:TARGET_APPROACH . 9)
    (:LANDING_SEARCH . 10)
    (:LANDING_APPROACH . 11))
)
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql 'FlightStateTransition)))
    "Constants for message type 'FlightStateTransition"
  '((:IDLE . 0)
    (:INITIALIZING . 1)
    (:TAKING_OFF . 2)
    (:FOLLOWING_PATH . 3)
    (:HOVERING . 4)
    (:LANDING . 5)
    (:RETURN_TO_LAUNCH . 6)
    (:LANDED . 7)
    (:TARGET_SEARCH . 8)
    (:TARGET_APPROACH . 9)
    (:LANDING_SEARCH . 10)
    (:LANDING_APPROACH . 11))
)
(cl:defmethod roslisp-msg-protocol:serialize ((msg <FlightStateTransition>) ostream)
  "Serializes a message object of type '<FlightStateTransition>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'from_state) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'to_state) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'from_state_id)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'to_state_id)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <FlightStateTransition>) istream)
  "Deserializes a message object of type '<FlightStateTransition>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'from_state) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'to_state) istream)
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'from_state_id)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'to_state_id)) (cl:read-byte istream))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<FlightStateTransition>)))
  "Returns string type for a message object of type '<FlightStateTransition>"
  "fla_msgs/FlightStateTransition")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'FlightStateTransition)))
  "Returns string type for a message object of type 'FlightStateTransition"
  "fla_msgs/FlightStateTransition")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<FlightStateTransition>)))
  "Returns md5sum for a message object of type '<FlightStateTransition>"
  "994f9c657664b2f00c832165b7503f8a")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'FlightStateTransition)))
  "Returns md5sum for a message object of type 'FlightStateTransition"
  "994f9c657664b2f00c832165b7503f8a")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<FlightStateTransition>)))
  "Returns full string definition for message of type '<FlightStateTransition>"
  (cl:format cl:nil "uint8 IDLE=0~%uint8 INITIALIZING=1~%uint8 TAKING_OFF=2~%uint8 FOLLOWING_PATH=3~%uint8 HOVERING=4~%uint8 LANDING=5~%uint8 RETURN_TO_LAUNCH=6~%uint8 LANDED=7~%uint8 TARGET_SEARCH=8~%uint8 TARGET_APPROACH=9~%uint8 LANDING_SEARCH=10~%uint8 LANDING_APPROACH=11~%~%std_msgs/Header header~%std_msgs/String from_state~%std_msgs/String to_state~%uint8 from_state_id~%uint8 to_state_id~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: std_msgs/String~%string data~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'FlightStateTransition)))
  "Returns full string definition for message of type 'FlightStateTransition"
  (cl:format cl:nil "uint8 IDLE=0~%uint8 INITIALIZING=1~%uint8 TAKING_OFF=2~%uint8 FOLLOWING_PATH=3~%uint8 HOVERING=4~%uint8 LANDING=5~%uint8 RETURN_TO_LAUNCH=6~%uint8 LANDED=7~%uint8 TARGET_SEARCH=8~%uint8 TARGET_APPROACH=9~%uint8 LANDING_SEARCH=10~%uint8 LANDING_APPROACH=11~%~%std_msgs/Header header~%std_msgs/String from_state~%std_msgs/String to_state~%uint8 from_state_id~%uint8 to_state_id~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: std_msgs/String~%string data~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <FlightStateTransition>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'from_state))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'to_state))
     1
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <FlightStateTransition>))
  "Converts a ROS message object to a list"
  (cl:list 'FlightStateTransition
    (cl:cons ':header (header msg))
    (cl:cons ':from_state (from_state msg))
    (cl:cons ':to_state (to_state msg))
    (cl:cons ':from_state_id (from_state_id msg))
    (cl:cons ':to_state_id (to_state_id msg))
))
