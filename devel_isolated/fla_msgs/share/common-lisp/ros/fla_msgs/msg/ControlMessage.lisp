; Auto-generated. Do not edit!


(cl:in-package fla_msgs-msg)


;//! \htmlinclude ControlMessage.msg.html

(cl:defclass <ControlMessage> (roslisp-msg-protocol:ros-message)
  ((stamp
    :reader stamp
    :initarg :stamp
    :type cl:real
    :initform 0)
   (len
    :reader len
    :initarg :len
    :type cl:integer
    :initform 0)
   (data
    :reader data
    :initarg :data
    :type (cl:vector cl:fixnum)
   :initform (cl:make-array 32 :element-type 'cl:fixnum :initial-element 0)))
)

(cl:defclass ControlMessage (<ControlMessage>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ControlMessage>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ControlMessage)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name fla_msgs-msg:<ControlMessage> is deprecated: use fla_msgs-msg:ControlMessage instead.")))

(cl:ensure-generic-function 'stamp-val :lambda-list '(m))
(cl:defmethod stamp-val ((m <ControlMessage>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader fla_msgs-msg:stamp-val is deprecated.  Use fla_msgs-msg:stamp instead.")
  (stamp m))

(cl:ensure-generic-function 'len-val :lambda-list '(m))
(cl:defmethod len-val ((m <ControlMessage>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader fla_msgs-msg:len-val is deprecated.  Use fla_msgs-msg:len instead.")
  (len m))

(cl:ensure-generic-function 'data-val :lambda-list '(m))
(cl:defmethod data-val ((m <ControlMessage>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader fla_msgs-msg:data-val is deprecated.  Use fla_msgs-msg:data instead.")
  (data m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ControlMessage>) ostream)
  "Serializes a message object of type '<ControlMessage>"
  (cl:let ((__sec (cl:floor (cl:slot-value msg 'stamp)))
        (__nsec (cl:round (cl:* 1e9 (cl:- (cl:slot-value msg 'stamp) (cl:floor (cl:slot-value msg 'stamp)))))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __sec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __sec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __sec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __sec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 0) __nsec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __nsec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __nsec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __nsec) ostream))
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'len)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'len)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'len)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'len)) ostream)
  (cl:map cl:nil #'(cl:lambda (ele) (cl:write-byte (cl:ldb (cl:byte 8 0) ele) ostream))
   (cl:slot-value msg 'data))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ControlMessage>) istream)
  "Deserializes a message object of type '<ControlMessage>"
    (cl:let ((__sec 0) (__nsec 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __sec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __sec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __sec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __sec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 0) __nsec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __nsec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __nsec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __nsec) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'stamp) (cl:+ (cl:coerce __sec 'cl:double-float) (cl:/ __nsec 1e9))))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'len)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'len)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'len)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'len)) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'data) (cl:make-array 32))
  (cl:let ((vals (cl:slot-value msg 'data)))
    (cl:dotimes (i 32)
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:aref vals i)) (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ControlMessage>)))
  "Returns string type for a message object of type '<ControlMessage>"
  "fla_msgs/ControlMessage")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ControlMessage)))
  "Returns string type for a message object of type 'ControlMessage"
  "fla_msgs/ControlMessage")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ControlMessage>)))
  "Returns md5sum for a message object of type '<ControlMessage>"
  "61a82e2f5b8cbb1d6b3046fb9bb5d79f")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ControlMessage)))
  "Returns md5sum for a message object of type 'ControlMessage"
  "61a82e2f5b8cbb1d6b3046fb9bb5d79f")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ControlMessage>)))
  "Returns full string definition for message of type '<ControlMessage>"
  (cl:format cl:nil "time stamp~%uint32 len~%uint8[32] data~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ControlMessage)))
  "Returns full string definition for message of type 'ControlMessage"
  (cl:format cl:nil "time stamp~%uint32 len~%uint8[32] data~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ControlMessage>))
  (cl:+ 0
     8
     4
     0 (cl:reduce #'cl:+ (cl:slot-value msg 'data) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 1)))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ControlMessage>))
  "Converts a ROS message object to a list"
  (cl:list 'ControlMessage
    (cl:cons ':stamp (stamp msg))
    (cl:cons ':len (len msg))
    (cl:cons ':data (data msg))
))
