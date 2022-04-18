; Auto-generated. Do not edit!


(cl:in-package snapstack_msgs-msg)


;//! \htmlinclude ControlLog.msg.html

(cl:defclass <ControlLog> (roslisp-msg-protocol:ros-message)
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
   (p_ref
    :reader p_ref
    :initarg :p_ref
    :type geometry_msgs-msg:Vector3
    :initform (cl:make-instance 'geometry_msgs-msg:Vector3))
   (p_err
    :reader p_err
    :initarg :p_err
    :type geometry_msgs-msg:Vector3
    :initform (cl:make-instance 'geometry_msgs-msg:Vector3))
   (p_err_int
    :reader p_err_int
    :initarg :p_err_int
    :type geometry_msgs-msg:Vector3
    :initform (cl:make-instance 'geometry_msgs-msg:Vector3))
   (v
    :reader v
    :initarg :v
    :type geometry_msgs-msg:Vector3
    :initform (cl:make-instance 'geometry_msgs-msg:Vector3))
   (v_ref
    :reader v_ref
    :initarg :v_ref
    :type geometry_msgs-msg:Vector3
    :initform (cl:make-instance 'geometry_msgs-msg:Vector3))
   (v_err
    :reader v_err
    :initarg :v_err
    :type geometry_msgs-msg:Vector3
    :initform (cl:make-instance 'geometry_msgs-msg:Vector3))
   (a_ff
    :reader a_ff
    :initarg :a_ff
    :type geometry_msgs-msg:Vector3
    :initform (cl:make-instance 'geometry_msgs-msg:Vector3))
   (a_fb
    :reader a_fb
    :initarg :a_fb
    :type geometry_msgs-msg:Vector3
    :initform (cl:make-instance 'geometry_msgs-msg:Vector3))
   (j_ff
    :reader j_ff
    :initarg :j_ff
    :type geometry_msgs-msg:Vector3
    :initform (cl:make-instance 'geometry_msgs-msg:Vector3))
   (j_fb
    :reader j_fb
    :initarg :j_fb
    :type geometry_msgs-msg:Vector3
    :initform (cl:make-instance 'geometry_msgs-msg:Vector3))
   (q
    :reader q
    :initarg :q
    :type geometry_msgs-msg:Quaternion
    :initform (cl:make-instance 'geometry_msgs-msg:Quaternion))
   (q_ref
    :reader q_ref
    :initarg :q_ref
    :type geometry_msgs-msg:Quaternion
    :initform (cl:make-instance 'geometry_msgs-msg:Quaternion))
   (rpy
    :reader rpy
    :initarg :rpy
    :type geometry_msgs-msg:Vector3
    :initform (cl:make-instance 'geometry_msgs-msg:Vector3))
   (rpy_ref
    :reader rpy_ref
    :initarg :rpy_ref
    :type geometry_msgs-msg:Vector3
    :initform (cl:make-instance 'geometry_msgs-msg:Vector3))
   (w
    :reader w
    :initarg :w
    :type geometry_msgs-msg:Vector3
    :initform (cl:make-instance 'geometry_msgs-msg:Vector3))
   (w_ref
    :reader w_ref
    :initarg :w_ref
    :type geometry_msgs-msg:Vector3
    :initform (cl:make-instance 'geometry_msgs-msg:Vector3))
   (F_W
    :reader F_W
    :initarg :F_W
    :type geometry_msgs-msg:Vector3
    :initform (cl:make-instance 'geometry_msgs-msg:Vector3))
   (thrust
    :reader thrust
    :initarg :thrust
    :type cl:float
    :initform 0.0)
   (throttle
    :reader throttle
    :initarg :throttle
    :type cl:float
    :initform 0.0)
   (power
    :reader power
    :initarg :power
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass ControlLog (<ControlLog>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ControlLog>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ControlLog)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name snapstack_msgs-msg:<ControlLog> is deprecated: use snapstack_msgs-msg:ControlLog instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <ControlLog>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader snapstack_msgs-msg:header-val is deprecated.  Use snapstack_msgs-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'p-val :lambda-list '(m))
(cl:defmethod p-val ((m <ControlLog>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader snapstack_msgs-msg:p-val is deprecated.  Use snapstack_msgs-msg:p instead.")
  (p m))

(cl:ensure-generic-function 'p_ref-val :lambda-list '(m))
(cl:defmethod p_ref-val ((m <ControlLog>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader snapstack_msgs-msg:p_ref-val is deprecated.  Use snapstack_msgs-msg:p_ref instead.")
  (p_ref m))

(cl:ensure-generic-function 'p_err-val :lambda-list '(m))
(cl:defmethod p_err-val ((m <ControlLog>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader snapstack_msgs-msg:p_err-val is deprecated.  Use snapstack_msgs-msg:p_err instead.")
  (p_err m))

(cl:ensure-generic-function 'p_err_int-val :lambda-list '(m))
(cl:defmethod p_err_int-val ((m <ControlLog>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader snapstack_msgs-msg:p_err_int-val is deprecated.  Use snapstack_msgs-msg:p_err_int instead.")
  (p_err_int m))

(cl:ensure-generic-function 'v-val :lambda-list '(m))
(cl:defmethod v-val ((m <ControlLog>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader snapstack_msgs-msg:v-val is deprecated.  Use snapstack_msgs-msg:v instead.")
  (v m))

(cl:ensure-generic-function 'v_ref-val :lambda-list '(m))
(cl:defmethod v_ref-val ((m <ControlLog>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader snapstack_msgs-msg:v_ref-val is deprecated.  Use snapstack_msgs-msg:v_ref instead.")
  (v_ref m))

(cl:ensure-generic-function 'v_err-val :lambda-list '(m))
(cl:defmethod v_err-val ((m <ControlLog>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader snapstack_msgs-msg:v_err-val is deprecated.  Use snapstack_msgs-msg:v_err instead.")
  (v_err m))

(cl:ensure-generic-function 'a_ff-val :lambda-list '(m))
(cl:defmethod a_ff-val ((m <ControlLog>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader snapstack_msgs-msg:a_ff-val is deprecated.  Use snapstack_msgs-msg:a_ff instead.")
  (a_ff m))

(cl:ensure-generic-function 'a_fb-val :lambda-list '(m))
(cl:defmethod a_fb-val ((m <ControlLog>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader snapstack_msgs-msg:a_fb-val is deprecated.  Use snapstack_msgs-msg:a_fb instead.")
  (a_fb m))

(cl:ensure-generic-function 'j_ff-val :lambda-list '(m))
(cl:defmethod j_ff-val ((m <ControlLog>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader snapstack_msgs-msg:j_ff-val is deprecated.  Use snapstack_msgs-msg:j_ff instead.")
  (j_ff m))

(cl:ensure-generic-function 'j_fb-val :lambda-list '(m))
(cl:defmethod j_fb-val ((m <ControlLog>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader snapstack_msgs-msg:j_fb-val is deprecated.  Use snapstack_msgs-msg:j_fb instead.")
  (j_fb m))

(cl:ensure-generic-function 'q-val :lambda-list '(m))
(cl:defmethod q-val ((m <ControlLog>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader snapstack_msgs-msg:q-val is deprecated.  Use snapstack_msgs-msg:q instead.")
  (q m))

(cl:ensure-generic-function 'q_ref-val :lambda-list '(m))
(cl:defmethod q_ref-val ((m <ControlLog>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader snapstack_msgs-msg:q_ref-val is deprecated.  Use snapstack_msgs-msg:q_ref instead.")
  (q_ref m))

(cl:ensure-generic-function 'rpy-val :lambda-list '(m))
(cl:defmethod rpy-val ((m <ControlLog>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader snapstack_msgs-msg:rpy-val is deprecated.  Use snapstack_msgs-msg:rpy instead.")
  (rpy m))

(cl:ensure-generic-function 'rpy_ref-val :lambda-list '(m))
(cl:defmethod rpy_ref-val ((m <ControlLog>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader snapstack_msgs-msg:rpy_ref-val is deprecated.  Use snapstack_msgs-msg:rpy_ref instead.")
  (rpy_ref m))

(cl:ensure-generic-function 'w-val :lambda-list '(m))
(cl:defmethod w-val ((m <ControlLog>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader snapstack_msgs-msg:w-val is deprecated.  Use snapstack_msgs-msg:w instead.")
  (w m))

(cl:ensure-generic-function 'w_ref-val :lambda-list '(m))
(cl:defmethod w_ref-val ((m <ControlLog>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader snapstack_msgs-msg:w_ref-val is deprecated.  Use snapstack_msgs-msg:w_ref instead.")
  (w_ref m))

(cl:ensure-generic-function 'F_W-val :lambda-list '(m))
(cl:defmethod F_W-val ((m <ControlLog>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader snapstack_msgs-msg:F_W-val is deprecated.  Use snapstack_msgs-msg:F_W instead.")
  (F_W m))

(cl:ensure-generic-function 'thrust-val :lambda-list '(m))
(cl:defmethod thrust-val ((m <ControlLog>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader snapstack_msgs-msg:thrust-val is deprecated.  Use snapstack_msgs-msg:thrust instead.")
  (thrust m))

(cl:ensure-generic-function 'throttle-val :lambda-list '(m))
(cl:defmethod throttle-val ((m <ControlLog>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader snapstack_msgs-msg:throttle-val is deprecated.  Use snapstack_msgs-msg:throttle instead.")
  (throttle m))

(cl:ensure-generic-function 'power-val :lambda-list '(m))
(cl:defmethod power-val ((m <ControlLog>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader snapstack_msgs-msg:power-val is deprecated.  Use snapstack_msgs-msg:power instead.")
  (power m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ControlLog>) ostream)
  "Serializes a message object of type '<ControlLog>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'p) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'p_ref) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'p_err) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'p_err_int) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'v) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'v_ref) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'v_err) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'a_ff) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'a_fb) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'j_ff) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'j_fb) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'q) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'q_ref) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'rpy) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'rpy_ref) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'w) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'w_ref) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'F_W) ostream)
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'thrust))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'throttle))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'power) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ControlLog>) istream)
  "Deserializes a message object of type '<ControlLog>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'p) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'p_ref) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'p_err) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'p_err_int) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'v) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'v_ref) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'v_err) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'a_ff) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'a_fb) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'j_ff) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'j_fb) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'q) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'q_ref) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'rpy) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'rpy_ref) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'w) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'w_ref) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'F_W) istream)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'thrust) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'throttle) (roslisp-utils:decode-double-float-bits bits)))
    (cl:setf (cl:slot-value msg 'power) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ControlLog>)))
  "Returns string type for a message object of type '<ControlLog>"
  "snapstack_msgs/ControlLog")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ControlLog)))
  "Returns string type for a message object of type 'ControlLog"
  "snapstack_msgs/ControlLog")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ControlLog>)))
  "Returns md5sum for a message object of type '<ControlLog>"
  "0127ad6ed84894e6d10c273726d40503")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ControlLog)))
  "Returns md5sum for a message object of type 'ControlLog"
  "0127ad6ed84894e6d10c273726d40503")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ControlLog>)))
  "Returns full string definition for message of type '<ControlLog>"
  (cl:format cl:nil "# The outer loop trajectory tracker generates this msg for analysis / debugging~%~%Header header~%~%# position signals~%geometry_msgs/Vector3 p~%geometry_msgs/Vector3 p_ref~%geometry_msgs/Vector3 p_err~%geometry_msgs/Vector3 p_err_int~%~%# velocity signals~%geometry_msgs/Vector3 v~%geometry_msgs/Vector3 v_ref~%geometry_msgs/Vector3 v_err~%~%# acceleration signals~%geometry_msgs/Vector3 a_ff~%geometry_msgs/Vector3 a_fb~%~%# jerk signals~%geometry_msgs/Vector3 j_ff~%geometry_msgs/Vector3 j_fb~%~%# attitude signals~%geometry_msgs/Quaternion q~%geometry_msgs/Quaternion q_ref~%geometry_msgs/Vector3 rpy~%geometry_msgs/Vector3 rpy_ref~%~%# angular rate signals~%geometry_msgs/Vector3 w~%geometry_msgs/Vector3 w_ref~%~%geometry_msgs/Vector3 F_W # Desired total force [N], expressed in world~%float64 thrust   # total desired force [N]~%float64 throttle # percent throttle sent to each motor~%~%bool power # true if motors should be able to spin~%~%# TODO: add outer (and inner?) parameters~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/Vector3~%# This represents a vector in free space. ~%# It is only meant to represent a direction. Therefore, it does not~%# make sense to apply a translation to it (e.g., when applying a ~%# generic rigid transformation to a Vector3, tf2 will only apply the~%# rotation). If you want your data to be translatable too, use the~%# geometry_msgs/Point message instead.~%~%float64 x~%float64 y~%float64 z~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ControlLog)))
  "Returns full string definition for message of type 'ControlLog"
  (cl:format cl:nil "# The outer loop trajectory tracker generates this msg for analysis / debugging~%~%Header header~%~%# position signals~%geometry_msgs/Vector3 p~%geometry_msgs/Vector3 p_ref~%geometry_msgs/Vector3 p_err~%geometry_msgs/Vector3 p_err_int~%~%# velocity signals~%geometry_msgs/Vector3 v~%geometry_msgs/Vector3 v_ref~%geometry_msgs/Vector3 v_err~%~%# acceleration signals~%geometry_msgs/Vector3 a_ff~%geometry_msgs/Vector3 a_fb~%~%# jerk signals~%geometry_msgs/Vector3 j_ff~%geometry_msgs/Vector3 j_fb~%~%# attitude signals~%geometry_msgs/Quaternion q~%geometry_msgs/Quaternion q_ref~%geometry_msgs/Vector3 rpy~%geometry_msgs/Vector3 rpy_ref~%~%# angular rate signals~%geometry_msgs/Vector3 w~%geometry_msgs/Vector3 w_ref~%~%geometry_msgs/Vector3 F_W # Desired total force [N], expressed in world~%float64 thrust   # total desired force [N]~%float64 throttle # percent throttle sent to each motor~%~%bool power # true if motors should be able to spin~%~%# TODO: add outer (and inner?) parameters~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/Vector3~%# This represents a vector in free space. ~%# It is only meant to represent a direction. Therefore, it does not~%# make sense to apply a translation to it (e.g., when applying a ~%# generic rigid transformation to a Vector3, tf2 will only apply the~%# rotation). If you want your data to be translatable too, use the~%# geometry_msgs/Point message instead.~%~%float64 x~%float64 y~%float64 z~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ControlLog>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'p))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'p_ref))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'p_err))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'p_err_int))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'v))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'v_ref))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'v_err))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'a_ff))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'a_fb))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'j_ff))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'j_fb))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'q))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'q_ref))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'rpy))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'rpy_ref))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'w))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'w_ref))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'F_W))
     8
     8
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ControlLog>))
  "Converts a ROS message object to a list"
  (cl:list 'ControlLog
    (cl:cons ':header (header msg))
    (cl:cons ':p (p msg))
    (cl:cons ':p_ref (p_ref msg))
    (cl:cons ':p_err (p_err msg))
    (cl:cons ':p_err_int (p_err_int msg))
    (cl:cons ':v (v msg))
    (cl:cons ':v_ref (v_ref msg))
    (cl:cons ':v_err (v_err msg))
    (cl:cons ':a_ff (a_ff msg))
    (cl:cons ':a_fb (a_fb msg))
    (cl:cons ':j_ff (j_ff msg))
    (cl:cons ':j_fb (j_fb msg))
    (cl:cons ':q (q msg))
    (cl:cons ':q_ref (q_ref msg))
    (cl:cons ':rpy (rpy msg))
    (cl:cons ':rpy_ref (rpy_ref msg))
    (cl:cons ':w (w msg))
    (cl:cons ':w_ref (w_ref msg))
    (cl:cons ':F_W (F_W msg))
    (cl:cons ':thrust (thrust msg))
    (cl:cons ':throttle (throttle msg))
    (cl:cons ':power (power msg))
))
