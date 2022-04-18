; Auto-generated. Do not edit!


(cl:in-package fla_msgs-msg)


;//! \htmlinclude TelemString.msg.html

(cl:defclass <TelemString> (roslisp-msg-protocol:ros-message)
  ((nodeid
    :reader nodeid
    :initarg :nodeid
    :type cl:integer
    :initform 0)
   (text
    :reader text
    :initarg :text
    :type cl:string
    :initform ""))
)

(cl:defclass TelemString (<TelemString>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <TelemString>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'TelemString)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name fla_msgs-msg:<TelemString> is deprecated: use fla_msgs-msg:TelemString instead.")))

(cl:ensure-generic-function 'nodeid-val :lambda-list '(m))
(cl:defmethod nodeid-val ((m <TelemString>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader fla_msgs-msg:nodeid-val is deprecated.  Use fla_msgs-msg:nodeid instead.")
  (nodeid m))

(cl:ensure-generic-function 'text-val :lambda-list '(m))
(cl:defmethod text-val ((m <TelemString>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader fla_msgs-msg:text-val is deprecated.  Use fla_msgs-msg:text instead.")
  (text m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <TelemString>) ostream)
  "Serializes a message object of type '<TelemString>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'nodeid)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'nodeid)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'nodeid)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'nodeid)) ostream)
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'text))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'text))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <TelemString>) istream)
  "Deserializes a message object of type '<TelemString>"
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'nodeid)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'nodeid)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'nodeid)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'nodeid)) (cl:read-byte istream))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'text) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'text) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<TelemString>)))
  "Returns string type for a message object of type '<TelemString>"
  "fla_msgs/TelemString")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'TelemString)))
  "Returns string type for a message object of type 'TelemString"
  "fla_msgs/TelemString")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<TelemString>)))
  "Returns md5sum for a message object of type '<TelemString>"
  "90c5a36c3a388c55ce89c05cc31213e3")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'TelemString)))
  "Returns md5sum for a message object of type 'TelemString"
  "90c5a36c3a388c55ce89c05cc31213e3")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<TelemString>)))
  "Returns full string definition for message of type '<TelemString>"
  (cl:format cl:nil "uint32 nodeid~%string text~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'TelemString)))
  "Returns full string definition for message of type 'TelemString"
  (cl:format cl:nil "uint32 nodeid~%string text~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <TelemString>))
  (cl:+ 0
     4
     4 (cl:length (cl:slot-value msg 'text))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <TelemString>))
  "Converts a ROS message object to a list"
  (cl:list 'TelemString
    (cl:cons ':nodeid (nodeid msg))
    (cl:cons ':text (text msg))
))
