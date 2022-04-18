; Auto-generated. Do not edit!


(cl:in-package master_node-msg)


;//! \htmlinclude MasterNodeState.msg.html

(cl:defclass <MasterNodeState> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (state
    :reader state
    :initarg :state
    :type cl:fixnum
    :initform 0))
)

(cl:defclass MasterNodeState (<MasterNodeState>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <MasterNodeState>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'MasterNodeState)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name master_node-msg:<MasterNodeState> is deprecated: use master_node-msg:MasterNodeState instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <MasterNodeState>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader master_node-msg:header-val is deprecated.  Use master_node-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'state-val :lambda-list '(m))
(cl:defmethod state-val ((m <MasterNodeState>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader master_node-msg:state-val is deprecated.  Use master_node-msg:state instead.")
  (state m))
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql '<MasterNodeState>)))
    "Constants for message type '<MasterNodeState>"
  '((:IDLE . 0)
    (:TAKEOFF . 1)
    (:FLIGHT_LOCAL . 2)
    (:FLIGHT_GLOBAL . 3)
    (:FLIGHT_HOLD . 4)
    (:UNKNOWN . 10))
)
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql 'MasterNodeState)))
    "Constants for message type 'MasterNodeState"
  '((:IDLE . 0)
    (:TAKEOFF . 1)
    (:FLIGHT_LOCAL . 2)
    (:FLIGHT_GLOBAL . 3)
    (:FLIGHT_HOLD . 4)
    (:UNKNOWN . 10))
)
(cl:defmethod roslisp-msg-protocol:serialize ((msg <MasterNodeState>) ostream)
  "Serializes a message object of type '<MasterNodeState>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:let* ((signed (cl:slot-value msg 'state)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 65536) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <MasterNodeState>) istream)
  "Deserializes a message object of type '<MasterNodeState>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'state) (cl:if (cl:< unsigned 32768) unsigned (cl:- unsigned 65536))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<MasterNodeState>)))
  "Returns string type for a message object of type '<MasterNodeState>"
  "master_node/MasterNodeState")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'MasterNodeState)))
  "Returns string type for a message object of type 'MasterNodeState"
  "master_node/MasterNodeState")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<MasterNodeState>)))
  "Returns md5sum for a message object of type '<MasterNodeState>"
  "94ab713c34436e1423557ad1082b0f2c")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'MasterNodeState)))
  "Returns md5sum for a message object of type 'MasterNodeState"
  "94ab713c34436e1423557ad1082b0f2c")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<MasterNodeState>)))
  "Returns full string definition for message of type '<MasterNodeState>"
  (cl:format cl:nil "std_msgs/Header header~%int16 IDLE = 0~%int16 TAKEOFF = 1~%int16 FLIGHT_LOCAL = 2~%int16 FLIGHT_GLOBAL = 3~%int16 FLIGHT_HOLD = 4~%int16 UNKNOWN = 10~%int16 state~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'MasterNodeState)))
  "Returns full string definition for message of type 'MasterNodeState"
  (cl:format cl:nil "std_msgs/Header header~%int16 IDLE = 0~%int16 TAKEOFF = 1~%int16 FLIGHT_LOCAL = 2~%int16 FLIGHT_GLOBAL = 3~%int16 FLIGHT_HOLD = 4~%int16 UNKNOWN = 10~%int16 state~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <MasterNodeState>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     2
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <MasterNodeState>))
  "Converts a ROS message object to a list"
  (cl:list 'MasterNodeState
    (cl:cons ':header (header msg))
    (cl:cons ':state (state msg))
))
