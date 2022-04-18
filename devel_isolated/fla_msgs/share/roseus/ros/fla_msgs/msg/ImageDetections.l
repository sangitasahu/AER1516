;; Auto-generated. Do not edit!


(when (boundp 'fla_msgs::ImageDetections)
  (if (not (find-package "FLA_MSGS"))
    (make-package "FLA_MSGS"))
  (shadow 'ImageDetections (find-package "FLA_MSGS")))
(unless (find-package "FLA_MSGS::IMAGEDETECTIONS")
  (make-package "FLA_MSGS::IMAGEDETECTIONS"))

(in-package "ROS")
;;//! \htmlinclude ImageDetections.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass fla_msgs::ImageDetections
  :super ros::object
  :slots (_header _frame_id _detections ))

(defmethod fla_msgs::ImageDetections
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:frame_id __frame_id) 0)
    ((:detections __detections) (let (r) (dotimes (i 0) (push (instance fla_msgs::Detection :init) r)) r))
    )
   (send-super :init)
   (setq _header __header)
   (setq _frame_id (round __frame_id))
   (setq _detections __detections)
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:frame_id
   (&optional __frame_id)
   (if __frame_id (setq _frame_id __frame_id)) _frame_id)
  (:detections
   (&rest __detections)
   (if (keywordp (car __detections))
       (send* _detections __detections)
     (progn
       (if __detections (setq _detections (car __detections)))
       _detections)))
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; uint64 _frame_id
    8
    ;; fla_msgs/Detection[] _detections
    (apply #'+ (send-all _detections :serialization-length)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; uint64 _frame_id
#+(or :alpha :irix6 :x86_64)
       (progn (sys::poke _frame_id (send s :buffer) (send s :count) :long) (incf (stream-count s) 8))
#-(or :alpha :irix6 :x86_64)
       (cond ((and (class _frame_id) (= (length (_frame_id . bv)) 2)) ;; bignum
              (write-long (ash (elt (_frame_id . bv) 0) 0) s)
              (write-long (ash (elt (_frame_id . bv) 1) -1) s))
             ((and (class _frame_id) (= (length (_frame_id . bv)) 1)) ;; big1
              (write-long (elt (_frame_id . bv) 0) s)
              (write-long (if (>= _frame_id 0) 0 #xffffffff) s))
             (t                                         ;; integer
              (write-long _frame_id s)(write-long (if (>= _frame_id 0) 0 #xffffffff) s)))
     ;; fla_msgs/Detection[] _detections
     (write-long (length _detections) s)
     (dolist (elem _detections)
       (send elem :serialize s)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; uint64 _frame_id
     
#+(or :alpha :irix6 :x86_64)
      (setf _frame_id (prog1 (sys::peek buf ptr- :long) (incf ptr- 8)))
#-(or :alpha :irix6 :x86_64)
      (setf _frame_id (let ((b0 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4)))
                  (b1 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4))))
              (cond ((= b1 -1) b0)
                     ((and (= b1  0)
                           (<= lisp::most-negative-fixnum b0 lisp::most-positive-fixnum))
                      b0)
                    ((= b1  0) (make-instance bignum :size 1 :bv (integer-vector b0)))
                    (t (make-instance bignum :size 2 :bv (integer-vector b0 (ash b1 1)))))))
   ;; fla_msgs/Detection[] _detections
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _detections (let (r) (dotimes (i n) (push (instance fla_msgs::Detection :init) r)) r))
     (dolist (elem- _detections)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;;
   self)
  )

(setf (get fla_msgs::ImageDetections :md5sum-) "0d46304a99ef564025aa09dae732b57e")
(setf (get fla_msgs::ImageDetections :datatype-) "fla_msgs/ImageDetections")
(setf (get fla_msgs::ImageDetections :definition-)
      "Header header
uint64 frame_id  # Image frame id
Detection[] detections

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
MSG: fla_msgs/Detection
uint32 class_id
string class_name
float32 confidence

# (x_min, y_min), (x_max, y_max) define the bounding box of the detection in pixel coordinates
float32 x_min 
float32 y_min
float32 x_max
float32 y_max

# ground truth unique identifier and position of the detected object (Simulation only) 
uint32 object_id
float32 x_pos
float32 y_pos
float32 z_pos

")



(provide :fla_msgs/ImageDetections "0d46304a99ef564025aa09dae732b57e")


