; Auto-generated. Do not edit!


(cl:in-package convex_decomposer-msg)


;//! \htmlinclude Polyhedron.msg.html

(cl:defclass <Polyhedron> (roslisp-msg-protocol:ros-message)
  ((planes
    :reader planes
    :initarg :planes
    :type (cl:vector shape_msgs-msg:Plane)
   :initform (cl:make-array 0 :element-type 'shape_msgs-msg:Plane :initial-element (cl:make-instance 'shape_msgs-msg:Plane))))
)

(cl:defclass Polyhedron (<Polyhedron>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Polyhedron>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Polyhedron)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name convex_decomposer-msg:<Polyhedron> is deprecated: use convex_decomposer-msg:Polyhedron instead.")))

(cl:ensure-generic-function 'planes-val :lambda-list '(m))
(cl:defmethod planes-val ((m <Polyhedron>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader convex_decomposer-msg:planes-val is deprecated.  Use convex_decomposer-msg:planes instead.")
  (planes m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Polyhedron>) ostream)
  "Serializes a message object of type '<Polyhedron>"
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'planes))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'planes))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Polyhedron>) istream)
  "Deserializes a message object of type '<Polyhedron>"
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'planes) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'planes)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'shape_msgs-msg:Plane))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Polyhedron>)))
  "Returns string type for a message object of type '<Polyhedron>"
  "convex_decomposer/Polyhedron")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Polyhedron)))
  "Returns string type for a message object of type 'Polyhedron"
  "convex_decomposer/Polyhedron")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Polyhedron>)))
  "Returns md5sum for a message object of type '<Polyhedron>"
  "18bc595cd5fca2d6b49e654a7e19a442")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Polyhedron)))
  "Returns md5sum for a message object of type 'Polyhedron"
  "18bc595cd5fca2d6b49e654a7e19a442")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Polyhedron>)))
  "Returns full string definition for message of type '<Polyhedron>"
  (cl:format cl:nil "shape_msgs/Plane[] planes~%~%================================================================================~%MSG: shape_msgs/Plane~%# Representation of a plane, using the plane equation ax + by + cz + d = 0~%~%# a := coef[0]~%# b := coef[1]~%# c := coef[2]~%# d := coef[3]~%~%float64[4] coef~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Polyhedron)))
  "Returns full string definition for message of type 'Polyhedron"
  (cl:format cl:nil "shape_msgs/Plane[] planes~%~%================================================================================~%MSG: shape_msgs/Plane~%# Representation of a plane, using the plane equation ax + by + cz + d = 0~%~%# a := coef[0]~%# b := coef[1]~%# c := coef[2]~%# d := coef[3]~%~%float64[4] coef~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Polyhedron>))
  (cl:+ 0
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'planes) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Polyhedron>))
  "Converts a ROS message object to a list"
  (cl:list 'Polyhedron
    (cl:cons ':planes (planes msg))
))
