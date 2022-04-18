; Auto-generated. Do not edit!


(cl:in-package fla_msgs-msg)


;//! \htmlinclude ImageDetections.msg.html

(cl:defclass <ImageDetections> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (frame_id
    :reader frame_id
    :initarg :frame_id
    :type cl:integer
    :initform 0)
   (detections
    :reader detections
    :initarg :detections
    :type (cl:vector fla_msgs-msg:Detection)
   :initform (cl:make-array 0 :element-type 'fla_msgs-msg:Detection :initial-element (cl:make-instance 'fla_msgs-msg:Detection))))
)

(cl:defclass ImageDetections (<ImageDetections>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ImageDetections>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ImageDetections)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name fla_msgs-msg:<ImageDetections> is deprecated: use fla_msgs-msg:ImageDetections instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <ImageDetections>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader fla_msgs-msg:header-val is deprecated.  Use fla_msgs-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'frame_id-val :lambda-list '(m))
(cl:defmethod frame_id-val ((m <ImageDetections>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader fla_msgs-msg:frame_id-val is deprecated.  Use fla_msgs-msg:frame_id instead.")
  (frame_id m))

(cl:ensure-generic-function 'detections-val :lambda-list '(m))
(cl:defmethod detections-val ((m <ImageDetections>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader fla_msgs-msg:detections-val is deprecated.  Use fla_msgs-msg:detections instead.")
  (detections m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ImageDetections>) ostream)
  "Serializes a message object of type '<ImageDetections>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'frame_id)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'frame_id)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'frame_id)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'frame_id)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 32) (cl:slot-value msg 'frame_id)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 40) (cl:slot-value msg 'frame_id)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 48) (cl:slot-value msg 'frame_id)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 56) (cl:slot-value msg 'frame_id)) ostream)
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'detections))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'detections))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ImageDetections>) istream)
  "Deserializes a message object of type '<ImageDetections>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'frame_id)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'frame_id)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'frame_id)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'frame_id)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 32) (cl:slot-value msg 'frame_id)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 40) (cl:slot-value msg 'frame_id)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 48) (cl:slot-value msg 'frame_id)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 56) (cl:slot-value msg 'frame_id)) (cl:read-byte istream))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'detections) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'detections)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'fla_msgs-msg:Detection))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ImageDetections>)))
  "Returns string type for a message object of type '<ImageDetections>"
  "fla_msgs/ImageDetections")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ImageDetections)))
  "Returns string type for a message object of type 'ImageDetections"
  "fla_msgs/ImageDetections")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ImageDetections>)))
  "Returns md5sum for a message object of type '<ImageDetections>"
  "0d46304a99ef564025aa09dae732b57e")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ImageDetections)))
  "Returns md5sum for a message object of type 'ImageDetections"
  "0d46304a99ef564025aa09dae732b57e")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ImageDetections>)))
  "Returns full string definition for message of type '<ImageDetections>"
  (cl:format cl:nil "Header header~%uint64 frame_id  # Image frame id~%Detection[] detections~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: fla_msgs/Detection~%uint32 class_id~%string class_name~%float32 confidence~%~%# (x_min, y_min), (x_max, y_max) define the bounding box of the detection in pixel coordinates~%float32 x_min ~%float32 y_min~%float32 x_max~%float32 y_max~%~%# ground truth unique identifier and position of the detected object (Simulation only) ~%uint32 object_id~%float32 x_pos~%float32 y_pos~%float32 z_pos~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ImageDetections)))
  "Returns full string definition for message of type 'ImageDetections"
  (cl:format cl:nil "Header header~%uint64 frame_id  # Image frame id~%Detection[] detections~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: fla_msgs/Detection~%uint32 class_id~%string class_name~%float32 confidence~%~%# (x_min, y_min), (x_max, y_max) define the bounding box of the detection in pixel coordinates~%float32 x_min ~%float32 y_min~%float32 x_max~%float32 y_max~%~%# ground truth unique identifier and position of the detected object (Simulation only) ~%uint32 object_id~%float32 x_pos~%float32 y_pos~%float32 z_pos~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ImageDetections>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     8
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'detections) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ImageDetections>))
  "Converts a ROS message object to a list"
  (cl:list 'ImageDetections
    (cl:cons ':header (header msg))
    (cl:cons ':frame_id (frame_id msg))
    (cl:cons ':detections (detections msg))
))
