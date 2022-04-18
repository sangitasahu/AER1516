; Auto-generated. Do not edit!


(cl:in-package fla_msgs-msg)


;//! \htmlinclude NodeList.msg.html

(cl:defclass <NodeList> (roslisp-msg-protocol:ros-message)
  ((stamp
    :reader stamp
    :initarg :stamp
    :type cl:real
    :initform 0)
   (nodelog
    :reader nodelog
    :initarg :nodelog
    :type (cl:vector fla_msgs-msg:NodeStatus)
   :initform (cl:make-array 0 :element-type 'fla_msgs-msg:NodeStatus :initial-element (cl:make-instance 'fla_msgs-msg:NodeStatus))))
)

(cl:defclass NodeList (<NodeList>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <NodeList>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'NodeList)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name fla_msgs-msg:<NodeList> is deprecated: use fla_msgs-msg:NodeList instead.")))

(cl:ensure-generic-function 'stamp-val :lambda-list '(m))
(cl:defmethod stamp-val ((m <NodeList>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader fla_msgs-msg:stamp-val is deprecated.  Use fla_msgs-msg:stamp instead.")
  (stamp m))

(cl:ensure-generic-function 'nodelog-val :lambda-list '(m))
(cl:defmethod nodelog-val ((m <NodeList>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader fla_msgs-msg:nodelog-val is deprecated.  Use fla_msgs-msg:nodelog instead.")
  (nodelog m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <NodeList>) ostream)
  "Serializes a message object of type '<NodeList>"
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
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'nodelog))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'nodelog))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <NodeList>) istream)
  "Deserializes a message object of type '<NodeList>"
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
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'nodelog) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'nodelog)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'fla_msgs-msg:NodeStatus))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<NodeList>)))
  "Returns string type for a message object of type '<NodeList>"
  "fla_msgs/NodeList")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'NodeList)))
  "Returns string type for a message object of type 'NodeList"
  "fla_msgs/NodeList")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<NodeList>)))
  "Returns md5sum for a message object of type '<NodeList>"
  "ef5328a9c9c28a94588f0215d3bce4ca")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'NodeList)))
  "Returns md5sum for a message object of type 'NodeList"
  "ef5328a9c9c28a94588f0215d3bce4ca")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<NodeList>)))
  "Returns full string definition for message of type '<NodeList>"
  (cl:format cl:nil "time stamp~%NodeStatus[] nodelog~%~%================================================================================~%MSG: fla_msgs/NodeStatus~%uint8 INACTIVE=1~%uint8 STARTING=2~%uint8 INIT=3~%uint8 READY=4~%uint8 ALARM=5~%uint8 FAIL=6~%uint8 SILENT=7~%uint32 id~%uint32 pid~%uint8 status~%int32 arg~%float64 load~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'NodeList)))
  "Returns full string definition for message of type 'NodeList"
  (cl:format cl:nil "time stamp~%NodeStatus[] nodelog~%~%================================================================================~%MSG: fla_msgs/NodeStatus~%uint8 INACTIVE=1~%uint8 STARTING=2~%uint8 INIT=3~%uint8 READY=4~%uint8 ALARM=5~%uint8 FAIL=6~%uint8 SILENT=7~%uint32 id~%uint32 pid~%uint8 status~%int32 arg~%float64 load~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <NodeList>))
  (cl:+ 0
     8
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'nodelog) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <NodeList>))
  "Converts a ROS message object to a list"
  (cl:list 'NodeList
    (cl:cons ':stamp (stamp msg))
    (cl:cons ':nodelog (nodelog msg))
))
