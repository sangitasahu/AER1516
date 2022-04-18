; Auto-generated. Do not edit!


(cl:in-package convex_decomposer-msg)


;//! \htmlinclude CvxDecomp.msg.html

(cl:defclass <CvxDecomp> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (polyhedra
    :reader polyhedra
    :initarg :polyhedra
    :type (cl:vector convex_decomposer-msg:Polyhedron)
   :initform (cl:make-array 0 :element-type 'convex_decomposer-msg:Polyhedron :initial-element (cl:make-instance 'convex_decomposer-msg:Polyhedron))))
)

(cl:defclass CvxDecomp (<CvxDecomp>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <CvxDecomp>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'CvxDecomp)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name convex_decomposer-msg:<CvxDecomp> is deprecated: use convex_decomposer-msg:CvxDecomp instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <CvxDecomp>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader convex_decomposer-msg:header-val is deprecated.  Use convex_decomposer-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'polyhedra-val :lambda-list '(m))
(cl:defmethod polyhedra-val ((m <CvxDecomp>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader convex_decomposer-msg:polyhedra-val is deprecated.  Use convex_decomposer-msg:polyhedra instead.")
  (polyhedra m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <CvxDecomp>) ostream)
  "Serializes a message object of type '<CvxDecomp>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'polyhedra))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'polyhedra))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <CvxDecomp>) istream)
  "Deserializes a message object of type '<CvxDecomp>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'polyhedra) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'polyhedra)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'convex_decomposer-msg:Polyhedron))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<CvxDecomp>)))
  "Returns string type for a message object of type '<CvxDecomp>"
  "convex_decomposer/CvxDecomp")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'CvxDecomp)))
  "Returns string type for a message object of type 'CvxDecomp"
  "convex_decomposer/CvxDecomp")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<CvxDecomp>)))
  "Returns md5sum for a message object of type '<CvxDecomp>"
  "fa0b44be5161dc2f552ca9c3ef45157c")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'CvxDecomp)))
  "Returns md5sum for a message object of type 'CvxDecomp"
  "fa0b44be5161dc2f552ca9c3ef45157c")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<CvxDecomp>)))
  "Returns full string definition for message of type '<CvxDecomp>"
  (cl:format cl:nil "std_msgs/Header header~%convex_decomposer/Polyhedron[] polyhedra~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: convex_decomposer/Polyhedron~%shape_msgs/Plane[] planes~%~%================================================================================~%MSG: shape_msgs/Plane~%# Representation of a plane, using the plane equation ax + by + cz + d = 0~%~%# a := coef[0]~%# b := coef[1]~%# c := coef[2]~%# d := coef[3]~%~%float64[4] coef~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'CvxDecomp)))
  "Returns full string definition for message of type 'CvxDecomp"
  (cl:format cl:nil "std_msgs/Header header~%convex_decomposer/Polyhedron[] polyhedra~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: convex_decomposer/Polyhedron~%shape_msgs/Plane[] planes~%~%================================================================================~%MSG: shape_msgs/Plane~%# Representation of a plane, using the plane equation ax + by + cz + d = 0~%~%# a := coef[0]~%# b := coef[1]~%# c := coef[2]~%# d := coef[3]~%~%float64[4] coef~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <CvxDecomp>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'polyhedra) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <CvxDecomp>))
  "Converts a ROS message object to a list"
  (cl:list 'CvxDecomp
    (cl:cons ':header (header msg))
    (cl:cons ':polyhedra (polyhedra msg))
))
