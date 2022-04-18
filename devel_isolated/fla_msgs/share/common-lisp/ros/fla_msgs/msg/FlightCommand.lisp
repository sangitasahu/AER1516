; Auto-generated. Do not edit!


(cl:in-package fla_msgs-msg)


;//! \htmlinclude FlightCommand.msg.html

(cl:defclass <FlightCommand> (roslisp-msg-protocol:ros-message)
  ((command
    :reader command
    :initarg :command
    :type cl:fixnum
    :initform 0))
)

(cl:defclass FlightCommand (<FlightCommand>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <FlightCommand>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'FlightCommand)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name fla_msgs-msg:<FlightCommand> is deprecated: use fla_msgs-msg:FlightCommand instead.")))

(cl:ensure-generic-function 'command-val :lambda-list '(m))
(cl:defmethod command-val ((m <FlightCommand>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader fla_msgs-msg:command-val is deprecated.  Use fla_msgs-msg:command instead.")
  (command m))
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql '<FlightCommand>)))
    "Constants for message type '<FlightCommand>"
  '((:CMD_TAKEOFF . 0)
    (:CMD_GO . 1)
    (:CMD_LAND . 2)
    (:CMD_HOVER . 3)
    (:CMD_INIT . 4)
    (:CMD_KILL . 5)
    (:CMD_START_SEARCH . 6))
)
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql 'FlightCommand)))
    "Constants for message type 'FlightCommand"
  '((:CMD_TAKEOFF . 0)
    (:CMD_GO . 1)
    (:CMD_LAND . 2)
    (:CMD_HOVER . 3)
    (:CMD_INIT . 4)
    (:CMD_KILL . 5)
    (:CMD_START_SEARCH . 6))
)
(cl:defmethod roslisp-msg-protocol:serialize ((msg <FlightCommand>) ostream)
  "Serializes a message object of type '<FlightCommand>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'command)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <FlightCommand>) istream)
  "Deserializes a message object of type '<FlightCommand>"
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'command)) (cl:read-byte istream))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<FlightCommand>)))
  "Returns string type for a message object of type '<FlightCommand>"
  "fla_msgs/FlightCommand")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'FlightCommand)))
  "Returns string type for a message object of type 'FlightCommand"
  "fla_msgs/FlightCommand")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<FlightCommand>)))
  "Returns md5sum for a message object of type '<FlightCommand>"
  "3fa8261f600a67acfe8d66a49672fd7b")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'FlightCommand)))
  "Returns md5sum for a message object of type 'FlightCommand"
  "3fa8261f600a67acfe8d66a49672fd7b")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<FlightCommand>)))
  "Returns full string definition for message of type '<FlightCommand>"
  (cl:format cl:nil "uint8 CMD_TAKEOFF=0~%uint8 CMD_GO=1~%uint8 CMD_LAND=2~%uint8 CMD_HOVER=3~%uint8 CMD_INIT=4~%uint8 CMD_KILL=5~%uint8 CMD_START_SEARCH=6~%uint8 command~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'FlightCommand)))
  "Returns full string definition for message of type 'FlightCommand"
  (cl:format cl:nil "uint8 CMD_TAKEOFF=0~%uint8 CMD_GO=1~%uint8 CMD_LAND=2~%uint8 CMD_HOVER=3~%uint8 CMD_INIT=4~%uint8 CMD_KILL=5~%uint8 CMD_START_SEARCH=6~%uint8 command~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <FlightCommand>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <FlightCommand>))
  "Converts a ROS message object to a list"
  (cl:list 'FlightCommand
    (cl:cons ':command (command msg))
))
