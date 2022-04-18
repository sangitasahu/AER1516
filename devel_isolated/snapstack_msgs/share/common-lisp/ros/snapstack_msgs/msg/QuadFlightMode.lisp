; Auto-generated. Do not edit!


(cl:in-package snapstack_msgs-msg)


;//! \htmlinclude QuadFlightMode.msg.html

(cl:defclass <QuadFlightMode> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (mode
    :reader mode
    :initarg :mode
    :type cl:fixnum
    :initform 0))
)

(cl:defclass QuadFlightMode (<QuadFlightMode>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <QuadFlightMode>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'QuadFlightMode)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name snapstack_msgs-msg:<QuadFlightMode> is deprecated: use snapstack_msgs-msg:QuadFlightMode instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <QuadFlightMode>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader snapstack_msgs-msg:header-val is deprecated.  Use snapstack_msgs-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'mode-val :lambda-list '(m))
(cl:defmethod mode-val ((m <QuadFlightMode>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader snapstack_msgs-msg:mode-val is deprecated.  Use snapstack_msgs-msg:mode instead.")
  (mode m))
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql '<QuadFlightMode>)))
    "Constants for message type '<QuadFlightMode>"
  '((:NOT_FLYING . 0)
    (:TAKEOFF . 1)
    (:LAND . 2)
    (:INIT . 3)
    (:GO . 4)
    (:ESTOP . 5)
    (:KILL . 6))
)
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql 'QuadFlightMode)))
    "Constants for message type 'QuadFlightMode"
  '((:NOT_FLYING . 0)
    (:TAKEOFF . 1)
    (:LAND . 2)
    (:INIT . 3)
    (:GO . 4)
    (:ESTOP . 5)
    (:KILL . 6))
)
(cl:defmethod roslisp-msg-protocol:serialize ((msg <QuadFlightMode>) ostream)
  "Serializes a message object of type '<QuadFlightMode>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'mode)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <QuadFlightMode>) istream)
  "Deserializes a message object of type '<QuadFlightMode>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'mode)) (cl:read-byte istream))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<QuadFlightMode>)))
  "Returns string type for a message object of type '<QuadFlightMode>"
  "snapstack_msgs/QuadFlightMode")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'QuadFlightMode)))
  "Returns string type for a message object of type 'QuadFlightMode"
  "snapstack_msgs/QuadFlightMode")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<QuadFlightMode>)))
  "Returns md5sum for a message object of type '<QuadFlightMode>"
  "e5d2af0214158b4bd2c1a618cd003a25")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'QuadFlightMode)))
  "Returns md5sum for a message object of type 'QuadFlightMode"
  "e5d2af0214158b4bd2c1a618cd003a25")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<QuadFlightMode>)))
  "Returns full string definition for message of type '<QuadFlightMode>"
  (cl:format cl:nil "Header header~%uint8 mode~%uint8 NOT_FLYING   = 0~%uint8 TAKEOFF      = 1~%uint8 LAND	       = 2~%uint8 INIT 	       = 3~%uint8 GO		   = 4~%uint8 ESTOP        = 5~%uint8 KILL		   = 6~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'QuadFlightMode)))
  "Returns full string definition for message of type 'QuadFlightMode"
  (cl:format cl:nil "Header header~%uint8 mode~%uint8 NOT_FLYING   = 0~%uint8 TAKEOFF      = 1~%uint8 LAND	       = 2~%uint8 INIT 	       = 3~%uint8 GO		   = 4~%uint8 ESTOP        = 5~%uint8 KILL		   = 6~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <QuadFlightMode>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <QuadFlightMode>))
  "Converts a ROS message object to a list"
  (cl:list 'QuadFlightMode
    (cl:cons ':header (header msg))
    (cl:cons ':mode (mode msg))
))
