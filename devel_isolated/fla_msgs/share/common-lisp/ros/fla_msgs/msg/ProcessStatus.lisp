; Auto-generated. Do not edit!


(cl:in-package fla_msgs-msg)


;//! \htmlinclude ProcessStatus.msg.html

(cl:defclass <ProcessStatus> (roslisp-msg-protocol:ros-message)
  ((id
    :reader id
    :initarg :id
    :type cl:integer
    :initform 0)
   (pid
    :reader pid
    :initarg :pid
    :type cl:integer
    :initform 0)
   (status
    :reader status
    :initarg :status
    :type cl:fixnum
    :initform 0)
   (arg
    :reader arg
    :initarg :arg
    :type cl:integer
    :initform 0))
)

(cl:defclass ProcessStatus (<ProcessStatus>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ProcessStatus>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ProcessStatus)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name fla_msgs-msg:<ProcessStatus> is deprecated: use fla_msgs-msg:ProcessStatus instead.")))

(cl:ensure-generic-function 'id-val :lambda-list '(m))
(cl:defmethod id-val ((m <ProcessStatus>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader fla_msgs-msg:id-val is deprecated.  Use fla_msgs-msg:id instead.")
  (id m))

(cl:ensure-generic-function 'pid-val :lambda-list '(m))
(cl:defmethod pid-val ((m <ProcessStatus>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader fla_msgs-msg:pid-val is deprecated.  Use fla_msgs-msg:pid instead.")
  (pid m))

(cl:ensure-generic-function 'status-val :lambda-list '(m))
(cl:defmethod status-val ((m <ProcessStatus>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader fla_msgs-msg:status-val is deprecated.  Use fla_msgs-msg:status instead.")
  (status m))

(cl:ensure-generic-function 'arg-val :lambda-list '(m))
(cl:defmethod arg-val ((m <ProcessStatus>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader fla_msgs-msg:arg-val is deprecated.  Use fla_msgs-msg:arg instead.")
  (arg m))
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql '<ProcessStatus>)))
    "Constants for message type '<ProcessStatus>"
  '((:INIT . 3)
    (:READY . 4)
    (:ALARM . 5)
    (:FAIL . 6))
)
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql 'ProcessStatus)))
    "Constants for message type 'ProcessStatus"
  '((:INIT . 3)
    (:READY . 4)
    (:ALARM . 5)
    (:FAIL . 6))
)
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ProcessStatus>) ostream)
  "Serializes a message object of type '<ProcessStatus>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'id)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'id)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'id)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'id)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'pid)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'pid)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'pid)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'pid)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'status)) ostream)
  (cl:let* ((signed (cl:slot-value msg 'arg)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ProcessStatus>) istream)
  "Deserializes a message object of type '<ProcessStatus>"
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'id)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'id)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'id)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'id)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'pid)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'pid)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'pid)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'pid)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'status)) (cl:read-byte istream))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'arg) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ProcessStatus>)))
  "Returns string type for a message object of type '<ProcessStatus>"
  "fla_msgs/ProcessStatus")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ProcessStatus)))
  "Returns string type for a message object of type 'ProcessStatus"
  "fla_msgs/ProcessStatus")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ProcessStatus>)))
  "Returns md5sum for a message object of type '<ProcessStatus>"
  "af0cc9ae0197397f393fc2c50073a7a2")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ProcessStatus)))
  "Returns md5sum for a message object of type 'ProcessStatus"
  "af0cc9ae0197397f393fc2c50073a7a2")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ProcessStatus>)))
  "Returns full string definition for message of type '<ProcessStatus>"
  (cl:format cl:nil "uint8 INIT=3~%uint8 READY=4~%uint8 ALARM=5~%uint8 FAIL=6~%uint32 id~%uint32 pid~%uint8 status~%int32 arg~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ProcessStatus)))
  "Returns full string definition for message of type 'ProcessStatus"
  (cl:format cl:nil "uint8 INIT=3~%uint8 READY=4~%uint8 ALARM=5~%uint8 FAIL=6~%uint32 id~%uint32 pid~%uint8 status~%int32 arg~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ProcessStatus>))
  (cl:+ 0
     4
     4
     1
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ProcessStatus>))
  "Converts a ROS message object to a list"
  (cl:list 'ProcessStatus
    (cl:cons ':id (id msg))
    (cl:cons ':pid (pid msg))
    (cl:cons ':status (status msg))
    (cl:cons ':arg (arg msg))
))
