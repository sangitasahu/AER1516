; Auto-generated. Do not edit!


(cl:in-package map_simulator-msg)


;//! \htmlinclude Map3D.msg.html

(cl:defclass <Map3D> (roslisp-msg-protocol:ros-message)
  ((nodeCoordinates
    :reader nodeCoordinates
    :initarg :nodeCoordinates
    :type (cl:vector geometry_msgs-msg:Vector3)
   :initform (cl:make-array 0 :element-type 'geometry_msgs-msg:Vector3 :initial-element (cl:make-instance 'geometry_msgs-msg:Vector3)))
   (occupiedStatus
    :reader occupiedStatus
    :initarg :occupiedStatus
    :type (cl:vector cl:boolean)
   :initform (cl:make-array 0 :element-type 'cl:boolean :initial-element cl:nil))
   (nodeDistances
    :reader nodeDistances
    :initarg :nodeDistances
    :type (cl:vector cl:fixnum)
   :initform (cl:make-array 0 :element-type 'cl:fixnum :initial-element 0)))
)

(cl:defclass Map3D (<Map3D>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Map3D>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Map3D)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name map_simulator-msg:<Map3D> is deprecated: use map_simulator-msg:Map3D instead.")))

(cl:ensure-generic-function 'nodeCoordinates-val :lambda-list '(m))
(cl:defmethod nodeCoordinates-val ((m <Map3D>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader map_simulator-msg:nodeCoordinates-val is deprecated.  Use map_simulator-msg:nodeCoordinates instead.")
  (nodeCoordinates m))

(cl:ensure-generic-function 'occupiedStatus-val :lambda-list '(m))
(cl:defmethod occupiedStatus-val ((m <Map3D>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader map_simulator-msg:occupiedStatus-val is deprecated.  Use map_simulator-msg:occupiedStatus instead.")
  (occupiedStatus m))

(cl:ensure-generic-function 'nodeDistances-val :lambda-list '(m))
(cl:defmethod nodeDistances-val ((m <Map3D>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader map_simulator-msg:nodeDistances-val is deprecated.  Use map_simulator-msg:nodeDistances instead.")
  (nodeDistances m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Map3D>) ostream)
  "Serializes a message object of type '<Map3D>"
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'nodeCoordinates))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'nodeCoordinates))
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'occupiedStatus))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if ele 1 0)) ostream))
   (cl:slot-value msg 'occupiedStatus))
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'nodeDistances))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:write-byte (cl:ldb (cl:byte 8 0) ele) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) ele) ostream))
   (cl:slot-value msg 'nodeDistances))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Map3D>) istream)
  "Deserializes a message object of type '<Map3D>"
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'nodeCoordinates) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'nodeCoordinates)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'geometry_msgs-msg:Vector3))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'occupiedStatus) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'occupiedStatus)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:not (cl:zerop (cl:read-byte istream)))))))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'nodeDistances) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'nodeDistances)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:aref vals i)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:aref vals i)) (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Map3D>)))
  "Returns string type for a message object of type '<Map3D>"
  "map_simulator/Map3D")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Map3D)))
  "Returns string type for a message object of type 'Map3D"
  "map_simulator/Map3D")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Map3D>)))
  "Returns md5sum for a message object of type '<Map3D>"
  "ad6c8afc9d0105744f261886c76d5da8")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Map3D)))
  "Returns md5sum for a message object of type 'Map3D"
  "ad6c8afc9d0105744f261886c76d5da8")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Map3D>)))
  "Returns full string definition for message of type '<Map3D>"
  (cl:format cl:nil "geometry_msgs/Vector3[] nodeCoordinates~%bool[] occupiedStatus~%uint16[] nodeDistances~%~%~%================================================================================~%MSG: geometry_msgs/Vector3~%# This represents a vector in free space. ~%# It is only meant to represent a direction. Therefore, it does not~%# make sense to apply a translation to it (e.g., when applying a ~%# generic rigid transformation to a Vector3, tf2 will only apply the~%# rotation). If you want your data to be translatable too, use the~%# geometry_msgs/Point message instead.~%~%float64 x~%float64 y~%float64 z~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Map3D)))
  "Returns full string definition for message of type 'Map3D"
  (cl:format cl:nil "geometry_msgs/Vector3[] nodeCoordinates~%bool[] occupiedStatus~%uint16[] nodeDistances~%~%~%================================================================================~%MSG: geometry_msgs/Vector3~%# This represents a vector in free space. ~%# It is only meant to represent a direction. Therefore, it does not~%# make sense to apply a translation to it (e.g., when applying a ~%# generic rigid transformation to a Vector3, tf2 will only apply the~%# rotation). If you want your data to be translatable too, use the~%# geometry_msgs/Point message instead.~%~%float64 x~%float64 y~%float64 z~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Map3D>))
  (cl:+ 0
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'nodeCoordinates) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'occupiedStatus) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 1)))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'nodeDistances) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 2)))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Map3D>))
  "Converts a ROS message object to a list"
  (cl:list 'Map3D
    (cl:cons ':nodeCoordinates (nodeCoordinates msg))
    (cl:cons ':occupiedStatus (occupiedStatus msg))
    (cl:cons ':nodeDistances (nodeDistances msg))
))
