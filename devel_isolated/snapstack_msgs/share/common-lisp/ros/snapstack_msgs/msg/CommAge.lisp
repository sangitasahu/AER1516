; Auto-generated. Do not edit!


(cl:in-package snapstack_msgs-msg)


;//! \htmlinclude CommAge.msg.html

(cl:defclass <CommAge> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (vicon_age_secs
    :reader vicon_age_secs
    :initarg :vicon_age_secs
    :type cl:float
    :initform 0.0)
   (goal_age_secs
    :reader goal_age_secs
    :initarg :goal_age_secs
    :type cl:float
    :initform 0.0))
)

(cl:defclass CommAge (<CommAge>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <CommAge>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'CommAge)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name snapstack_msgs-msg:<CommAge> is deprecated: use snapstack_msgs-msg:CommAge instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <CommAge>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader snapstack_msgs-msg:header-val is deprecated.  Use snapstack_msgs-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'vicon_age_secs-val :lambda-list '(m))
(cl:defmethod vicon_age_secs-val ((m <CommAge>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader snapstack_msgs-msg:vicon_age_secs-val is deprecated.  Use snapstack_msgs-msg:vicon_age_secs instead.")
  (vicon_age_secs m))

(cl:ensure-generic-function 'goal_age_secs-val :lambda-list '(m))
(cl:defmethod goal_age_secs-val ((m <CommAge>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader snapstack_msgs-msg:goal_age_secs-val is deprecated.  Use snapstack_msgs-msg:goal_age_secs instead.")
  (goal_age_secs m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <CommAge>) ostream)
  "Serializes a message object of type '<CommAge>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'vicon_age_secs))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'goal_age_secs))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <CommAge>) istream)
  "Deserializes a message object of type '<CommAge>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'vicon_age_secs) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'goal_age_secs) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<CommAge>)))
  "Returns string type for a message object of type '<CommAge>"
  "snapstack_msgs/CommAge")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'CommAge)))
  "Returns string type for a message object of type 'CommAge"
  "snapstack_msgs/CommAge")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<CommAge>)))
  "Returns md5sum for a message object of type '<CommAge>"
  "37ee5d091cfb61db7a1dcd668b6244ed")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'CommAge)))
  "Returns md5sum for a message object of type 'CommAge"
  "37ee5d091cfb61db7a1dcd668b6244ed")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<CommAge>)))
  "Returns full string definition for message of type '<CommAge>"
  (cl:format cl:nil "Header header~%float32 vicon_age_secs~%float32 goal_age_secs~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'CommAge)))
  "Returns full string definition for message of type 'CommAge"
  (cl:format cl:nil "Header header~%float32 vicon_age_secs~%float32 goal_age_secs~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <CommAge>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <CommAge>))
  "Converts a ROS message object to a list"
  (cl:list 'CommAge
    (cl:cons ':header (header msg))
    (cl:cons ':vicon_age_secs (vicon_age_secs msg))
    (cl:cons ':goal_age_secs (goal_age_secs msg))
))
