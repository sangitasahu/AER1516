; Auto-generated. Do not edit!


(cl:in-package fla_msgs-msg)


;//! \htmlinclude JoyDef.msg.html

(cl:defclass <JoyDef> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass JoyDef (<JoyDef>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <JoyDef>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'JoyDef)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name fla_msgs-msg:<JoyDef> is deprecated: use fla_msgs-msg:JoyDef instead.")))
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql '<JoyDef>)))
    "Constants for message type '<JoyDef>"
  '((:A . 0)
    (:B . 1)
    (:X . 2)
    (:Y . 3)
    (:LB . 4)
    (:RB . 5)
    (:BACK . 6)
    (:START . 7)
    (:CENTER . 8)
    (:LS . 9)
    (:RS . 10)
    (:LEFT . 11)
    (:RIGHT . 12)
    (:UP . 13)
    (:DOWN . 14)
    (:LEFT_X . 0)
    (:LEFT_Y . 1)
    (:LT . 2)
    (:RIGHT_X . 3)
    (:RIGHT_Y . 4)
    (:RT . 5))
)
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql 'JoyDef)))
    "Constants for message type 'JoyDef"
  '((:A . 0)
    (:B . 1)
    (:X . 2)
    (:Y . 3)
    (:LB . 4)
    (:RB . 5)
    (:BACK . 6)
    (:START . 7)
    (:CENTER . 8)
    (:LS . 9)
    (:RS . 10)
    (:LEFT . 11)
    (:RIGHT . 12)
    (:UP . 13)
    (:DOWN . 14)
    (:LEFT_X . 0)
    (:LEFT_Y . 1)
    (:LT . 2)
    (:RIGHT_X . 3)
    (:RIGHT_Y . 4)
    (:RT . 5))
)
(cl:defmethod roslisp-msg-protocol:serialize ((msg <JoyDef>) ostream)
  "Serializes a message object of type '<JoyDef>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <JoyDef>) istream)
  "Deserializes a message object of type '<JoyDef>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<JoyDef>)))
  "Returns string type for a message object of type '<JoyDef>"
  "fla_msgs/JoyDef")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'JoyDef)))
  "Returns string type for a message object of type 'JoyDef"
  "fla_msgs/JoyDef")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<JoyDef>)))
  "Returns md5sum for a message object of type '<JoyDef>"
  "e5483257fd0a1adbc2dd471d331e6470")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'JoyDef)))
  "Returns md5sum for a message object of type 'JoyDef"
  "e5483257fd0a1adbc2dd471d331e6470")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<JoyDef>)))
  "Returns full string definition for message of type '<JoyDef>"
  (cl:format cl:nil "# Message defining constants for the wireless joysticks used by ACL~%~%# buttons~%int64 A=0~%int64 B=1~%int64 X=2~%int64 Y=3~%int64 LB=4~%int64 RB=5~%int64 BACK=6~%int64 START=7~%int64 CENTER=8 # center 'on' button~%int64 LS=9 # left joystick~%int64 RS=10 # right joystick~%int64 LEFT=11~%int64 RIGHT=12~%int64 UP=13~%int64 DOWN=14~%~%# axes~%int64 LEFT_X=0~%int64 LEFT_Y=1~%int64 LT=2~%int64 RIGHT_X=3~%int64 RIGHT_Y=4~%int64 RT=5~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'JoyDef)))
  "Returns full string definition for message of type 'JoyDef"
  (cl:format cl:nil "# Message defining constants for the wireless joysticks used by ACL~%~%# buttons~%int64 A=0~%int64 B=1~%int64 X=2~%int64 Y=3~%int64 LB=4~%int64 RB=5~%int64 BACK=6~%int64 START=7~%int64 CENTER=8 # center 'on' button~%int64 LS=9 # left joystick~%int64 RS=10 # right joystick~%int64 LEFT=11~%int64 RIGHT=12~%int64 UP=13~%int64 DOWN=14~%~%# axes~%int64 LEFT_X=0~%int64 LEFT_Y=1~%int64 LT=2~%int64 RIGHT_X=3~%int64 RIGHT_Y=4~%int64 RT=5~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <JoyDef>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <JoyDef>))
  "Converts a ROS message object to a list"
  (cl:list 'JoyDef
))
