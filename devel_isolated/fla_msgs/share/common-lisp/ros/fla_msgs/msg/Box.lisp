; Auto-generated. Do not edit!


(cl:in-package fla_msgs-msg)


;//! \htmlinclude Box.msg.html

(cl:defclass <Box> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (origin
    :reader origin
    :initarg :origin
    :type geometry_msgs-msg:Vector3
    :initform (cl:make-instance 'geometry_msgs-msg:Vector3))
   (size
    :reader size
    :initarg :size
    :type geometry_msgs-msg:Vector3
    :initform (cl:make-instance 'geometry_msgs-msg:Vector3)))
)

(cl:defclass Box (<Box>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Box>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Box)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name fla_msgs-msg:<Box> is deprecated: use fla_msgs-msg:Box instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <Box>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader fla_msgs-msg:header-val is deprecated.  Use fla_msgs-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'origin-val :lambda-list '(m))
(cl:defmethod origin-val ((m <Box>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader fla_msgs-msg:origin-val is deprecated.  Use fla_msgs-msg:origin instead.")
  (origin m))

(cl:ensure-generic-function 'size-val :lambda-list '(m))
(cl:defmethod size-val ((m <Box>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader fla_msgs-msg:size-val is deprecated.  Use fla_msgs-msg:size instead.")
  (size m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Box>) ostream)
  "Serializes a message object of type '<Box>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'origin) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'size) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Box>) istream)
  "Deserializes a message object of type '<Box>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'origin) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'size) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Box>)))
  "Returns string type for a message object of type '<Box>"
  "fla_msgs/Box")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Box)))
  "Returns string type for a message object of type 'Box"
  "fla_msgs/Box")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Box>)))
  "Returns md5sum for a message object of type '<Box>"
  "3423b5be7e72312adb724b75aba39e1f")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Box)))
  "Returns md5sum for a message object of type 'Box"
  "3423b5be7e72312adb724b75aba39e1f")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Box>)))
  "Returns full string definition for message of type '<Box>"
  (cl:format cl:nil "# Message describing a 3-dimensional rectangular box.~%# Origin is defined as the corner of the box with the minimum x, y, and z values~%# in the cartesian coordinate system provided by header.frame.~%# Size gives the x, y, and z dimensions of the box.~%~%std_msgs/Header header~%geometry_msgs/Vector3 origin~%geometry_msgs/Vector3 size~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/Vector3~%# This represents a vector in free space. ~%# It is only meant to represent a direction. Therefore, it does not~%# make sense to apply a translation to it (e.g., when applying a ~%# generic rigid transformation to a Vector3, tf2 will only apply the~%# rotation). If you want your data to be translatable too, use the~%# geometry_msgs/Point message instead.~%~%float64 x~%float64 y~%float64 z~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Box)))
  "Returns full string definition for message of type 'Box"
  (cl:format cl:nil "# Message describing a 3-dimensional rectangular box.~%# Origin is defined as the corner of the box with the minimum x, y, and z values~%# in the cartesian coordinate system provided by header.frame.~%# Size gives the x, y, and z dimensions of the box.~%~%std_msgs/Header header~%geometry_msgs/Vector3 origin~%geometry_msgs/Vector3 size~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/Vector3~%# This represents a vector in free space. ~%# It is only meant to represent a direction. Therefore, it does not~%# make sense to apply a translation to it (e.g., when applying a ~%# generic rigid transformation to a Vector3, tf2 will only apply the~%# rotation). If you want your data to be translatable too, use the~%# geometry_msgs/Point message instead.~%~%float64 x~%float64 y~%float64 z~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Box>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'origin))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'size))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Box>))
  "Converts a ROS message object to a list"
  (cl:list 'Box
    (cl:cons ':header (header msg))
    (cl:cons ':origin (origin msg))
    (cl:cons ':size (size msg))
))
