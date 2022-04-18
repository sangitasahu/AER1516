; Auto-generated. Do not edit!


(cl:in-package snapstack_msgs-msg)


;//! \htmlinclude Goal.msg.html

(cl:defclass <Goal> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (p
    :reader p
    :initarg :p
    :type geometry_msgs-msg:Vector3
    :initform (cl:make-instance 'geometry_msgs-msg:Vector3))
   (v
    :reader v
    :initarg :v
    :type geometry_msgs-msg:Vector3
    :initform (cl:make-instance 'geometry_msgs-msg:Vector3))
   (a
    :reader a
    :initarg :a
    :type geometry_msgs-msg:Vector3
    :initform (cl:make-instance 'geometry_msgs-msg:Vector3))
   (j
    :reader j
    :initarg :j
    :type geometry_msgs-msg:Vector3
    :initform (cl:make-instance 'geometry_msgs-msg:Vector3))
   (yaw
    :reader yaw
    :initarg :yaw
    :type cl:float
    :initform 0.0)
   (dyaw
    :reader dyaw
    :initarg :dyaw
    :type cl:float
    :initform 0.0)
   (power
    :reader power
    :initarg :power
    :type cl:boolean
    :initform cl:nil)
   (mode_xy
    :reader mode_xy
    :initarg :mode_xy
    :type cl:fixnum
    :initform 0)
   (mode_z
    :reader mode_z
    :initarg :mode_z
    :type cl:fixnum
    :initform 0))
)

(cl:defclass Goal (<Goal>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Goal>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Goal)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name snapstack_msgs-msg:<Goal> is deprecated: use snapstack_msgs-msg:Goal instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <Goal>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader snapstack_msgs-msg:header-val is deprecated.  Use snapstack_msgs-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'p-val :lambda-list '(m))
(cl:defmethod p-val ((m <Goal>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader snapstack_msgs-msg:p-val is deprecated.  Use snapstack_msgs-msg:p instead.")
  (p m))

(cl:ensure-generic-function 'v-val :lambda-list '(m))
(cl:defmethod v-val ((m <Goal>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader snapstack_msgs-msg:v-val is deprecated.  Use snapstack_msgs-msg:v instead.")
  (v m))

(cl:ensure-generic-function 'a-val :lambda-list '(m))
(cl:defmethod a-val ((m <Goal>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader snapstack_msgs-msg:a-val is deprecated.  Use snapstack_msgs-msg:a instead.")
  (a m))

(cl:ensure-generic-function 'j-val :lambda-list '(m))
(cl:defmethod j-val ((m <Goal>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader snapstack_msgs-msg:j-val is deprecated.  Use snapstack_msgs-msg:j instead.")
  (j m))

(cl:ensure-generic-function 'yaw-val :lambda-list '(m))
(cl:defmethod yaw-val ((m <Goal>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader snapstack_msgs-msg:yaw-val is deprecated.  Use snapstack_msgs-msg:yaw instead.")
  (yaw m))

(cl:ensure-generic-function 'dyaw-val :lambda-list '(m))
(cl:defmethod dyaw-val ((m <Goal>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader snapstack_msgs-msg:dyaw-val is deprecated.  Use snapstack_msgs-msg:dyaw instead.")
  (dyaw m))

(cl:ensure-generic-function 'power-val :lambda-list '(m))
(cl:defmethod power-val ((m <Goal>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader snapstack_msgs-msg:power-val is deprecated.  Use snapstack_msgs-msg:power instead.")
  (power m))

(cl:ensure-generic-function 'mode_xy-val :lambda-list '(m))
(cl:defmethod mode_xy-val ((m <Goal>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader snapstack_msgs-msg:mode_xy-val is deprecated.  Use snapstack_msgs-msg:mode_xy instead.")
  (mode_xy m))

(cl:ensure-generic-function 'mode_z-val :lambda-list '(m))
(cl:defmethod mode_z-val ((m <Goal>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader snapstack_msgs-msg:mode_z-val is deprecated.  Use snapstack_msgs-msg:mode_z instead.")
  (mode_z m))
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql '<Goal>)))
    "Constants for message type '<Goal>"
  '((:MODE_POSITION_CONTROL . 0)
    (:MODE_VELOCITY_CONTROL . 1)
    (:MODE_ACCELERATION_CONTROL . 2))
)
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql 'Goal)))
    "Constants for message type 'Goal"
  '((:MODE_POSITION_CONTROL . 0)
    (:MODE_VELOCITY_CONTROL . 1)
    (:MODE_ACCELERATION_CONTROL . 2))
)
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Goal>) ostream)
  "Serializes a message object of type '<Goal>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'p) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'v) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'a) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'j) ostream)
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'yaw))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'dyaw))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'power) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'mode_xy)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'mode_z)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Goal>) istream)
  "Deserializes a message object of type '<Goal>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'p) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'v) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'a) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'j) istream)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'yaw) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'dyaw) (roslisp-utils:decode-double-float-bits bits)))
    (cl:setf (cl:slot-value msg 'power) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'mode_xy)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'mode_z)) (cl:read-byte istream))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Goal>)))
  "Returns string type for a message object of type '<Goal>"
  "snapstack_msgs/Goal")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Goal)))
  "Returns string type for a message object of type 'Goal"
  "snapstack_msgs/Goal")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Goal>)))
  "Returns md5sum for a message object of type '<Goal>"
  "29f7a5b62089bdabd9ea1780f356bc8b")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Goal)))
  "Returns md5sum for a message object of type 'Goal"
  "29f7a5b62089bdabd9ea1780f356bc8b")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Goal>)))
  "Returns full string definition for message of type '<Goal>"
  (cl:format cl:nil "# Use this message to command the outer loop to track~%# a trajectory generated from a high-level trajectory planner.~%~%Header header~%~%# Current time-slice of desired trajectory~%geometry_msgs/Vector3 p # position~%geometry_msgs/Vector3 v # velocity~%geometry_msgs/Vector3 a # acceleration~%geometry_msgs/Vector3 j # jerk~%~%float64 yaw # heading / yaw angle~%float64 dyaw # d/dt{unrolled, unpitched body heading w.r.t world}~%# n.b., recall that dyaw = d/dt{psi} != r. Angular heading rate r is defined in~%# the body frame, but yaw is the heading of the local level frame w.r.t world.~%# For slow, nearly-level flight, dyaw ~~= r. For more agile flight, it will~%# be useful to make sure you are commanding the correct quantity.~%# See, e.g., eq (7) and (8) in ~%# https://scholarsarchive.byu.edu/cgi/viewcontent.cgi?article=2324&context=facpub~%~%bool power # true if motors should be able to spin~%~%# Trajectory tracking mode constants~%uint8 MODE_POSITION_CONTROL     = 0~%uint8 MODE_VELOCITY_CONTROL     = 1~%uint8 MODE_ACCELERATION_CONTROL = 2~%~%# Trajectory tracking mode for x/y and z components.~%# The default is POSITION control, which uses position and velocity error~%# to calculate the control effort. VELOCITY control only uses vel error.~%# ACCELERATION mode does not use tracking error and could be used to provide~%# a control signal computed from something other than the default PID cntrl.~%uint8 mode_xy~%uint8 mode_z~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/Vector3~%# This represents a vector in free space. ~%# It is only meant to represent a direction. Therefore, it does not~%# make sense to apply a translation to it (e.g., when applying a ~%# generic rigid transformation to a Vector3, tf2 will only apply the~%# rotation). If you want your data to be translatable too, use the~%# geometry_msgs/Point message instead.~%~%float64 x~%float64 y~%float64 z~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Goal)))
  "Returns full string definition for message of type 'Goal"
  (cl:format cl:nil "# Use this message to command the outer loop to track~%# a trajectory generated from a high-level trajectory planner.~%~%Header header~%~%# Current time-slice of desired trajectory~%geometry_msgs/Vector3 p # position~%geometry_msgs/Vector3 v # velocity~%geometry_msgs/Vector3 a # acceleration~%geometry_msgs/Vector3 j # jerk~%~%float64 yaw # heading / yaw angle~%float64 dyaw # d/dt{unrolled, unpitched body heading w.r.t world}~%# n.b., recall that dyaw = d/dt{psi} != r. Angular heading rate r is defined in~%# the body frame, but yaw is the heading of the local level frame w.r.t world.~%# For slow, nearly-level flight, dyaw ~~= r. For more agile flight, it will~%# be useful to make sure you are commanding the correct quantity.~%# See, e.g., eq (7) and (8) in ~%# https://scholarsarchive.byu.edu/cgi/viewcontent.cgi?article=2324&context=facpub~%~%bool power # true if motors should be able to spin~%~%# Trajectory tracking mode constants~%uint8 MODE_POSITION_CONTROL     = 0~%uint8 MODE_VELOCITY_CONTROL     = 1~%uint8 MODE_ACCELERATION_CONTROL = 2~%~%# Trajectory tracking mode for x/y and z components.~%# The default is POSITION control, which uses position and velocity error~%# to calculate the control effort. VELOCITY control only uses vel error.~%# ACCELERATION mode does not use tracking error and could be used to provide~%# a control signal computed from something other than the default PID cntrl.~%uint8 mode_xy~%uint8 mode_z~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/Vector3~%# This represents a vector in free space. ~%# It is only meant to represent a direction. Therefore, it does not~%# make sense to apply a translation to it (e.g., when applying a ~%# generic rigid transformation to a Vector3, tf2 will only apply the~%# rotation). If you want your data to be translatable too, use the~%# geometry_msgs/Point message instead.~%~%float64 x~%float64 y~%float64 z~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Goal>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'p))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'v))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'a))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'j))
     8
     8
     1
     1
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Goal>))
  "Converts a ROS message object to a list"
  (cl:list 'Goal
    (cl:cons ':header (header msg))
    (cl:cons ':p (p msg))
    (cl:cons ':v (v msg))
    (cl:cons ':a (a msg))
    (cl:cons ':j (j msg))
    (cl:cons ':yaw (yaw msg))
    (cl:cons ':dyaw (dyaw msg))
    (cl:cons ':power (power msg))
    (cl:cons ':mode_xy (mode_xy msg))
    (cl:cons ':mode_z (mode_z msg))
))
