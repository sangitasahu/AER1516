; Auto-generated. Do not edit!


(cl:in-package fla_msgs-msg)


;//! \htmlinclude NodeStatus.msg.html

(cl:defclass <NodeStatus> (roslisp-msg-protocol:ros-message)
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
    :initform 0)
   (load
    :reader load
    :initarg :load
    :type cl:float
    :initform 0.0))
)

(cl:defclass NodeStatus (<NodeStatus>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <NodeStatus>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'NodeStatus)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name fla_msgs-msg:<NodeStatus> is deprecated: use fla_msgs-msg:NodeStatus instead.")))

(cl:ensure-generic-function 'id-val :lambda-list '(m))
(cl:defmethod id-val ((m <NodeStatus>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader fla_msgs-msg:id-val is deprecated.  Use fla_msgs-msg:id instead.")
  (id m))

(cl:ensure-generic-function 'pid-val :lambda-list '(m))
(cl:defmethod pid-val ((m <NodeStatus>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader fla_msgs-msg:pid-val is deprecated.  Use fla_msgs-msg:pid instead.")
  (pid m))

(cl:ensure-generic-function 'status-val :lambda-list '(m))
(cl:defmethod status-val ((m <NodeStatus>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader fla_msgs-msg:status-val is deprecated.  Use fla_msgs-msg:status instead.")
  (status m))

(cl:ensure-generic-function 'arg-val :lambda-list '(m))
(cl:defmethod arg-val ((m <NodeStatus>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader fla_msgs-msg:arg-val is deprecated.  Use fla_msgs-msg:arg instead.")
  (arg m))

(cl:ensure-generic-function 'load-val :lambda-list '(m))
(cl:defmethod load-val ((m <NodeStatus>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader fla_msgs-msg:load-val is deprecated.  Use fla_msgs-msg:load instead.")
  (load m))
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql '<NodeStatus>)))
    "Constants for message type '<NodeStatus>"
  '((:INACTIVE . 1)
    (:STARTING . 2)
    (:INIT . 3)
    (:READY . 4)
    (:ALARM . 5)
    (:FAIL . 6)
    (:SILENT . 7))
)
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql 'NodeStatus)))
    "Constants for message type 'NodeStatus"
  '((:INACTIVE . 1)
    (:STARTING . 2)
    (:INIT . 3)
    (:READY . 4)
    (:ALARM . 5)
    (:FAIL . 6)
    (:SILENT . 7))
)
(cl:defmethod roslisp-msg-protocol:serialize ((msg <NodeStatus>) ostream)
  "Serializes a message object of type '<NodeStatus>"
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
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'load))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <NodeStatus>) istream)
  "Deserializes a message object of type '<NodeStatus>"
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
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'load) (roslisp-utils:decode-double-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<NodeStatus>)))
  "Returns string type for a message object of type '<NodeStatus>"
  "fla_msgs/NodeStatus")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'NodeStatus)))
  "Returns string type for a message object of type 'NodeStatus"
  "fla_msgs/NodeStatus")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<NodeStatus>)))
  "Returns md5sum for a message object of type '<NodeStatus>"
  "bc0e9b8f7a292a18c6b50b43586a86d8")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'NodeStatus)))
  "Returns md5sum for a message object of type 'NodeStatus"
  "bc0e9b8f7a292a18c6b50b43586a86d8")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<NodeStatus>)))
  "Returns full string definition for message of type '<NodeStatus>"
  (cl:format cl:nil "uint8 INACTIVE=1~%uint8 STARTING=2~%uint8 INIT=3~%uint8 READY=4~%uint8 ALARM=5~%uint8 FAIL=6~%uint8 SILENT=7~%uint32 id~%uint32 pid~%uint8 status~%int32 arg~%float64 load~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'NodeStatus)))
  "Returns full string definition for message of type 'NodeStatus"
  (cl:format cl:nil "uint8 INACTIVE=1~%uint8 STARTING=2~%uint8 INIT=3~%uint8 READY=4~%uint8 ALARM=5~%uint8 FAIL=6~%uint8 SILENT=7~%uint32 id~%uint32 pid~%uint8 status~%int32 arg~%float64 load~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <NodeStatus>))
  (cl:+ 0
     4
     4
     1
     4
     8
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <NodeStatus>))
  "Converts a ROS message object to a list"
  (cl:list 'NodeStatus
    (cl:cons ':id (id msg))
    (cl:cons ':pid (pid msg))
    (cl:cons ':status (status msg))
    (cl:cons ':arg (arg msg))
    (cl:cons ':load (load msg))
))
