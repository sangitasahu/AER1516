; Auto-generated. Do not edit!


(cl:in-package faster_msgs-msg)


;//! \htmlinclude Mode.msg.html

(cl:defclass <Mode> (roslisp-msg-protocol:ros-message)
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

(cl:defclass Mode (<Mode>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Mode>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Mode)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name faster_msgs-msg:<Mode> is deprecated: use faster_msgs-msg:Mode instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <Mode>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader faster_msgs-msg:header-val is deprecated.  Use faster_msgs-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'mode-val :lambda-list '(m))
(cl:defmethod mode-val ((m <Mode>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader faster_msgs-msg:mode-val is deprecated.  Use faster_msgs-msg:mode instead.")
  (mode m))
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql '<Mode>)))
    "Constants for message type '<Mode>"
  '((:ON_GROUND . 0)
    (:GO . 1))
)
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql 'Mode)))
    "Constants for message type 'Mode"
  '((:ON_GROUND . 0)
    (:GO . 1))
)
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Mode>) ostream)
  "Serializes a message object of type '<Mode>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'mode)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Mode>) istream)
  "Deserializes a message object of type '<Mode>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'mode)) (cl:read-byte istream))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Mode>)))
  "Returns string type for a message object of type '<Mode>"
  "faster_msgs/Mode")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Mode)))
  "Returns string type for a message object of type 'Mode"
  "faster_msgs/Mode")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Mode>)))
  "Returns md5sum for a message object of type '<Mode>"
  "46808a00b0dbe958093b017bcb10cf21")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Mode)))
  "Returns md5sum for a message object of type 'Mode"
  "46808a00b0dbe958093b017bcb10cf21")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Mode>)))
  "Returns full string definition for message of type '<Mode>"
  (cl:format cl:nil "Header header~%uint8 mode~%~%uint8 ON_GROUND   = 0~%uint8 GO	      = 1~%~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Mode)))
  "Returns full string definition for message of type 'Mode"
  (cl:format cl:nil "Header header~%uint8 mode~%~%uint8 ON_GROUND   = 0~%uint8 GO	      = 1~%~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Mode>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Mode>))
  "Converts a ROS message object to a list"
  (cl:list 'Mode
    (cl:cons ':header (header msg))
    (cl:cons ':mode (mode msg))
))
