;; Auto-generated. Do not edit!


(when (boundp 'snapstack_msgs::AttitudeCommand)
  (if (not (find-package "SNAPSTACK_MSGS"))
    (make-package "SNAPSTACK_MSGS"))
  (shadow 'AttitudeCommand (find-package "SNAPSTACK_MSGS")))
(unless (find-package "SNAPSTACK_MSGS::ATTITUDECOMMAND")
  (make-package "SNAPSTACK_MSGS::ATTITUDECOMMAND"))

(in-package "ROS")
;;//! \htmlinclude AttitudeCommand.msg.html
(if (not (find-package "GEOMETRY_MSGS"))
  (ros::roseus-add-msgs "geometry_msgs"))
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass snapstack_msgs::AttitudeCommand
  :super ros::object
  :slots (_header _q _w _throttle _power ))

(defmethod snapstack_msgs::AttitudeCommand
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:q __q) (instance geometry_msgs::Quaternion :init))
    ((:w __w) (instance geometry_msgs::Vector3 :init))
    ((:throttle __throttle) 0.0)
    ((:power __power) nil)
    )
   (send-super :init)
   (setq _header __header)
   (setq _q __q)
   (setq _w __w)
   (setq _throttle (float __throttle))
   (setq _power __power)
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:q
   (&rest __q)
   (if (keywordp (car __q))
       (send* _q __q)
     (progn
       (if __q (setq _q (car __q)))
       _q)))
  (:w
   (&rest __w)
   (if (keywordp (car __w))
       (send* _w __w)
     (progn
       (if __w (setq _w (car __w)))
       _w)))
  (:throttle
   (&optional __throttle)
   (if __throttle (setq _throttle __throttle)) _throttle)
  (:power
   (&optional __power)
   (if __power (setq _power __power)) _power)
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; geometry_msgs/Quaternion _q
    (send _q :serialization-length)
    ;; geometry_msgs/Vector3 _w
    (send _w :serialization-length)
    ;; float64 _throttle
    8
    ;; bool _power
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; geometry_msgs/Quaternion _q
       (send _q :serialize s)
     ;; geometry_msgs/Vector3 _w
       (send _w :serialize s)
     ;; float64 _throttle
       (sys::poke _throttle (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; bool _power
       (if _power (write-byte -1 s) (write-byte 0 s))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; geometry_msgs/Quaternion _q
     (send _q :deserialize buf ptr-) (incf ptr- (send _q :serialization-length))
   ;; geometry_msgs/Vector3 _w
     (send _w :deserialize buf ptr-) (incf ptr- (send _w :serialization-length))
   ;; float64 _throttle
     (setq _throttle (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; bool _power
     (setq _power (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;;
   self)
  )

(setf (get snapstack_msgs::AttitudeCommand :md5sum-) "910a3f7454dc682455a31074c15a518f")
(setf (get snapstack_msgs::AttitudeCommand :datatype-) "snapstack_msgs/AttitudeCommand")
(setf (get snapstack_msgs::AttitudeCommand :definition-)
      "Header header
geometry_msgs/Quaternion q    # desired attitude
geometry_msgs/Vector3 w       # desired angular rates
float64 throttle # desired percent throttle sent to each motor
bool power # true if motors should be able to spin

================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data 
# in a particular coordinate frame.
# 
# sequence ID: consecutively increasing ID 
uint32 seq
#Two-integer timestamp that is expressed as:
# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
# time-handling sugar is provided by the client library
time stamp
#Frame this data is associated with
string frame_id

================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x
float64 y
float64 z
float64 w

================================================================================
MSG: geometry_msgs/Vector3
# This represents a vector in free space. 
# It is only meant to represent a direction. Therefore, it does not
# make sense to apply a translation to it (e.g., when applying a 
# generic rigid transformation to a Vector3, tf2 will only apply the
# rotation). If you want your data to be translatable too, use the
# geometry_msgs/Point message instead.

float64 x
float64 y
float64 z
")



(provide :snapstack_msgs/AttitudeCommand "910a3f7454dc682455a31074c15a518f")


