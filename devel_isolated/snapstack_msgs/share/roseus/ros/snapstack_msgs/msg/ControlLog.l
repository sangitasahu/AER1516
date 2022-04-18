;; Auto-generated. Do not edit!


(when (boundp 'snapstack_msgs::ControlLog)
  (if (not (find-package "SNAPSTACK_MSGS"))
    (make-package "SNAPSTACK_MSGS"))
  (shadow 'ControlLog (find-package "SNAPSTACK_MSGS")))
(unless (find-package "SNAPSTACK_MSGS::CONTROLLOG")
  (make-package "SNAPSTACK_MSGS::CONTROLLOG"))

(in-package "ROS")
;;//! \htmlinclude ControlLog.msg.html
(if (not (find-package "GEOMETRY_MSGS"))
  (ros::roseus-add-msgs "geometry_msgs"))
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass snapstack_msgs::ControlLog
  :super ros::object
  :slots (_header _p _p_ref _p_err _p_err_int _v _v_ref _v_err _a_ff _a_fb _j_ff _j_fb _q _q_ref _rpy _rpy_ref _w _w_ref _F_W _thrust _throttle _power ))

(defmethod snapstack_msgs::ControlLog
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:p __p) (instance geometry_msgs::Vector3 :init))
    ((:p_ref __p_ref) (instance geometry_msgs::Vector3 :init))
    ((:p_err __p_err) (instance geometry_msgs::Vector3 :init))
    ((:p_err_int __p_err_int) (instance geometry_msgs::Vector3 :init))
    ((:v __v) (instance geometry_msgs::Vector3 :init))
    ((:v_ref __v_ref) (instance geometry_msgs::Vector3 :init))
    ((:v_err __v_err) (instance geometry_msgs::Vector3 :init))
    ((:a_ff __a_ff) (instance geometry_msgs::Vector3 :init))
    ((:a_fb __a_fb) (instance geometry_msgs::Vector3 :init))
    ((:j_ff __j_ff) (instance geometry_msgs::Vector3 :init))
    ((:j_fb __j_fb) (instance geometry_msgs::Vector3 :init))
    ((:q __q) (instance geometry_msgs::Quaternion :init))
    ((:q_ref __q_ref) (instance geometry_msgs::Quaternion :init))
    ((:rpy __rpy) (instance geometry_msgs::Vector3 :init))
    ((:rpy_ref __rpy_ref) (instance geometry_msgs::Vector3 :init))
    ((:w __w) (instance geometry_msgs::Vector3 :init))
    ((:w_ref __w_ref) (instance geometry_msgs::Vector3 :init))
    ((:F_W __F_W) (instance geometry_msgs::Vector3 :init))
    ((:thrust __thrust) 0.0)
    ((:throttle __throttle) 0.0)
    ((:power __power) nil)
    )
   (send-super :init)
   (setq _header __header)
   (setq _p __p)
   (setq _p_ref __p_ref)
   (setq _p_err __p_err)
   (setq _p_err_int __p_err_int)
   (setq _v __v)
   (setq _v_ref __v_ref)
   (setq _v_err __v_err)
   (setq _a_ff __a_ff)
   (setq _a_fb __a_fb)
   (setq _j_ff __j_ff)
   (setq _j_fb __j_fb)
   (setq _q __q)
   (setq _q_ref __q_ref)
   (setq _rpy __rpy)
   (setq _rpy_ref __rpy_ref)
   (setq _w __w)
   (setq _w_ref __w_ref)
   (setq _F_W __F_W)
   (setq _thrust (float __thrust))
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
  (:p
   (&rest __p)
   (if (keywordp (car __p))
       (send* _p __p)
     (progn
       (if __p (setq _p (car __p)))
       _p)))
  (:p_ref
   (&rest __p_ref)
   (if (keywordp (car __p_ref))
       (send* _p_ref __p_ref)
     (progn
       (if __p_ref (setq _p_ref (car __p_ref)))
       _p_ref)))
  (:p_err
   (&rest __p_err)
   (if (keywordp (car __p_err))
       (send* _p_err __p_err)
     (progn
       (if __p_err (setq _p_err (car __p_err)))
       _p_err)))
  (:p_err_int
   (&rest __p_err_int)
   (if (keywordp (car __p_err_int))
       (send* _p_err_int __p_err_int)
     (progn
       (if __p_err_int (setq _p_err_int (car __p_err_int)))
       _p_err_int)))
  (:v
   (&rest __v)
   (if (keywordp (car __v))
       (send* _v __v)
     (progn
       (if __v (setq _v (car __v)))
       _v)))
  (:v_ref
   (&rest __v_ref)
   (if (keywordp (car __v_ref))
       (send* _v_ref __v_ref)
     (progn
       (if __v_ref (setq _v_ref (car __v_ref)))
       _v_ref)))
  (:v_err
   (&rest __v_err)
   (if (keywordp (car __v_err))
       (send* _v_err __v_err)
     (progn
       (if __v_err (setq _v_err (car __v_err)))
       _v_err)))
  (:a_ff
   (&rest __a_ff)
   (if (keywordp (car __a_ff))
       (send* _a_ff __a_ff)
     (progn
       (if __a_ff (setq _a_ff (car __a_ff)))
       _a_ff)))
  (:a_fb
   (&rest __a_fb)
   (if (keywordp (car __a_fb))
       (send* _a_fb __a_fb)
     (progn
       (if __a_fb (setq _a_fb (car __a_fb)))
       _a_fb)))
  (:j_ff
   (&rest __j_ff)
   (if (keywordp (car __j_ff))
       (send* _j_ff __j_ff)
     (progn
       (if __j_ff (setq _j_ff (car __j_ff)))
       _j_ff)))
  (:j_fb
   (&rest __j_fb)
   (if (keywordp (car __j_fb))
       (send* _j_fb __j_fb)
     (progn
       (if __j_fb (setq _j_fb (car __j_fb)))
       _j_fb)))
  (:q
   (&rest __q)
   (if (keywordp (car __q))
       (send* _q __q)
     (progn
       (if __q (setq _q (car __q)))
       _q)))
  (:q_ref
   (&rest __q_ref)
   (if (keywordp (car __q_ref))
       (send* _q_ref __q_ref)
     (progn
       (if __q_ref (setq _q_ref (car __q_ref)))
       _q_ref)))
  (:rpy
   (&rest __rpy)
   (if (keywordp (car __rpy))
       (send* _rpy __rpy)
     (progn
       (if __rpy (setq _rpy (car __rpy)))
       _rpy)))
  (:rpy_ref
   (&rest __rpy_ref)
   (if (keywordp (car __rpy_ref))
       (send* _rpy_ref __rpy_ref)
     (progn
       (if __rpy_ref (setq _rpy_ref (car __rpy_ref)))
       _rpy_ref)))
  (:w
   (&rest __w)
   (if (keywordp (car __w))
       (send* _w __w)
     (progn
       (if __w (setq _w (car __w)))
       _w)))
  (:w_ref
   (&rest __w_ref)
   (if (keywordp (car __w_ref))
       (send* _w_ref __w_ref)
     (progn
       (if __w_ref (setq _w_ref (car __w_ref)))
       _w_ref)))
  (:F_W
   (&rest __F_W)
   (if (keywordp (car __F_W))
       (send* _F_W __F_W)
     (progn
       (if __F_W (setq _F_W (car __F_W)))
       _F_W)))
  (:thrust
   (&optional __thrust)
   (if __thrust (setq _thrust __thrust)) _thrust)
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
    ;; geometry_msgs/Vector3 _p
    (send _p :serialization-length)
    ;; geometry_msgs/Vector3 _p_ref
    (send _p_ref :serialization-length)
    ;; geometry_msgs/Vector3 _p_err
    (send _p_err :serialization-length)
    ;; geometry_msgs/Vector3 _p_err_int
    (send _p_err_int :serialization-length)
    ;; geometry_msgs/Vector3 _v
    (send _v :serialization-length)
    ;; geometry_msgs/Vector3 _v_ref
    (send _v_ref :serialization-length)
    ;; geometry_msgs/Vector3 _v_err
    (send _v_err :serialization-length)
    ;; geometry_msgs/Vector3 _a_ff
    (send _a_ff :serialization-length)
    ;; geometry_msgs/Vector3 _a_fb
    (send _a_fb :serialization-length)
    ;; geometry_msgs/Vector3 _j_ff
    (send _j_ff :serialization-length)
    ;; geometry_msgs/Vector3 _j_fb
    (send _j_fb :serialization-length)
    ;; geometry_msgs/Quaternion _q
    (send _q :serialization-length)
    ;; geometry_msgs/Quaternion _q_ref
    (send _q_ref :serialization-length)
    ;; geometry_msgs/Vector3 _rpy
    (send _rpy :serialization-length)
    ;; geometry_msgs/Vector3 _rpy_ref
    (send _rpy_ref :serialization-length)
    ;; geometry_msgs/Vector3 _w
    (send _w :serialization-length)
    ;; geometry_msgs/Vector3 _w_ref
    (send _w_ref :serialization-length)
    ;; geometry_msgs/Vector3 _F_W
    (send _F_W :serialization-length)
    ;; float64 _thrust
    8
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
     ;; geometry_msgs/Vector3 _p
       (send _p :serialize s)
     ;; geometry_msgs/Vector3 _p_ref
       (send _p_ref :serialize s)
     ;; geometry_msgs/Vector3 _p_err
       (send _p_err :serialize s)
     ;; geometry_msgs/Vector3 _p_err_int
       (send _p_err_int :serialize s)
     ;; geometry_msgs/Vector3 _v
       (send _v :serialize s)
     ;; geometry_msgs/Vector3 _v_ref
       (send _v_ref :serialize s)
     ;; geometry_msgs/Vector3 _v_err
       (send _v_err :serialize s)
     ;; geometry_msgs/Vector3 _a_ff
       (send _a_ff :serialize s)
     ;; geometry_msgs/Vector3 _a_fb
       (send _a_fb :serialize s)
     ;; geometry_msgs/Vector3 _j_ff
       (send _j_ff :serialize s)
     ;; geometry_msgs/Vector3 _j_fb
       (send _j_fb :serialize s)
     ;; geometry_msgs/Quaternion _q
       (send _q :serialize s)
     ;; geometry_msgs/Quaternion _q_ref
       (send _q_ref :serialize s)
     ;; geometry_msgs/Vector3 _rpy
       (send _rpy :serialize s)
     ;; geometry_msgs/Vector3 _rpy_ref
       (send _rpy_ref :serialize s)
     ;; geometry_msgs/Vector3 _w
       (send _w :serialize s)
     ;; geometry_msgs/Vector3 _w_ref
       (send _w_ref :serialize s)
     ;; geometry_msgs/Vector3 _F_W
       (send _F_W :serialize s)
     ;; float64 _thrust
       (sys::poke _thrust (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
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
   ;; geometry_msgs/Vector3 _p
     (send _p :deserialize buf ptr-) (incf ptr- (send _p :serialization-length))
   ;; geometry_msgs/Vector3 _p_ref
     (send _p_ref :deserialize buf ptr-) (incf ptr- (send _p_ref :serialization-length))
   ;; geometry_msgs/Vector3 _p_err
     (send _p_err :deserialize buf ptr-) (incf ptr- (send _p_err :serialization-length))
   ;; geometry_msgs/Vector3 _p_err_int
     (send _p_err_int :deserialize buf ptr-) (incf ptr- (send _p_err_int :serialization-length))
   ;; geometry_msgs/Vector3 _v
     (send _v :deserialize buf ptr-) (incf ptr- (send _v :serialization-length))
   ;; geometry_msgs/Vector3 _v_ref
     (send _v_ref :deserialize buf ptr-) (incf ptr- (send _v_ref :serialization-length))
   ;; geometry_msgs/Vector3 _v_err
     (send _v_err :deserialize buf ptr-) (incf ptr- (send _v_err :serialization-length))
   ;; geometry_msgs/Vector3 _a_ff
     (send _a_ff :deserialize buf ptr-) (incf ptr- (send _a_ff :serialization-length))
   ;; geometry_msgs/Vector3 _a_fb
     (send _a_fb :deserialize buf ptr-) (incf ptr- (send _a_fb :serialization-length))
   ;; geometry_msgs/Vector3 _j_ff
     (send _j_ff :deserialize buf ptr-) (incf ptr- (send _j_ff :serialization-length))
   ;; geometry_msgs/Vector3 _j_fb
     (send _j_fb :deserialize buf ptr-) (incf ptr- (send _j_fb :serialization-length))
   ;; geometry_msgs/Quaternion _q
     (send _q :deserialize buf ptr-) (incf ptr- (send _q :serialization-length))
   ;; geometry_msgs/Quaternion _q_ref
     (send _q_ref :deserialize buf ptr-) (incf ptr- (send _q_ref :serialization-length))
   ;; geometry_msgs/Vector3 _rpy
     (send _rpy :deserialize buf ptr-) (incf ptr- (send _rpy :serialization-length))
   ;; geometry_msgs/Vector3 _rpy_ref
     (send _rpy_ref :deserialize buf ptr-) (incf ptr- (send _rpy_ref :serialization-length))
   ;; geometry_msgs/Vector3 _w
     (send _w :deserialize buf ptr-) (incf ptr- (send _w :serialization-length))
   ;; geometry_msgs/Vector3 _w_ref
     (send _w_ref :deserialize buf ptr-) (incf ptr- (send _w_ref :serialization-length))
   ;; geometry_msgs/Vector3 _F_W
     (send _F_W :deserialize buf ptr-) (incf ptr- (send _F_W :serialization-length))
   ;; float64 _thrust
     (setq _thrust (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _throttle
     (setq _throttle (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; bool _power
     (setq _power (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;;
   self)
  )

(setf (get snapstack_msgs::ControlLog :md5sum-) "0127ad6ed84894e6d10c273726d40503")
(setf (get snapstack_msgs::ControlLog :datatype-) "snapstack_msgs/ControlLog")
(setf (get snapstack_msgs::ControlLog :definition-)
      "# The outer loop trajectory tracker generates this msg for analysis / debugging

Header header

# position signals
geometry_msgs/Vector3 p
geometry_msgs/Vector3 p_ref
geometry_msgs/Vector3 p_err
geometry_msgs/Vector3 p_err_int

# velocity signals
geometry_msgs/Vector3 v
geometry_msgs/Vector3 v_ref
geometry_msgs/Vector3 v_err

# acceleration signals
geometry_msgs/Vector3 a_ff
geometry_msgs/Vector3 a_fb

# jerk signals
geometry_msgs/Vector3 j_ff
geometry_msgs/Vector3 j_fb

# attitude signals
geometry_msgs/Quaternion q
geometry_msgs/Quaternion q_ref
geometry_msgs/Vector3 rpy
geometry_msgs/Vector3 rpy_ref

# angular rate signals
geometry_msgs/Vector3 w
geometry_msgs/Vector3 w_ref

geometry_msgs/Vector3 F_W # Desired total force [N], expressed in world
float64 thrust   # total desired force [N]
float64 throttle # percent throttle sent to each motor

bool power # true if motors should be able to spin

# TODO: add outer (and inner?) parameters
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
================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x
float64 y
float64 z
float64 w

")



(provide :snapstack_msgs/ControlLog "0127ad6ed84894e6d10c273726d40503")


