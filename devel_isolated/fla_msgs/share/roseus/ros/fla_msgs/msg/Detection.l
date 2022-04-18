;; Auto-generated. Do not edit!


(when (boundp 'fla_msgs::Detection)
  (if (not (find-package "FLA_MSGS"))
    (make-package "FLA_MSGS"))
  (shadow 'Detection (find-package "FLA_MSGS")))
(unless (find-package "FLA_MSGS::DETECTION")
  (make-package "FLA_MSGS::DETECTION"))

(in-package "ROS")
;;//! \htmlinclude Detection.msg.html


(defclass fla_msgs::Detection
  :super ros::object
  :slots (_class_id _class_name _confidence _x_min _y_min _x_max _y_max _object_id _x_pos _y_pos _z_pos ))

(defmethod fla_msgs::Detection
  (:init
   (&key
    ((:class_id __class_id) 0)
    ((:class_name __class_name) "")
    ((:confidence __confidence) 0.0)
    ((:x_min __x_min) 0.0)
    ((:y_min __y_min) 0.0)
    ((:x_max __x_max) 0.0)
    ((:y_max __y_max) 0.0)
    ((:object_id __object_id) 0)
    ((:x_pos __x_pos) 0.0)
    ((:y_pos __y_pos) 0.0)
    ((:z_pos __z_pos) 0.0)
    )
   (send-super :init)
   (setq _class_id (round __class_id))
   (setq _class_name (string __class_name))
   (setq _confidence (float __confidence))
   (setq _x_min (float __x_min))
   (setq _y_min (float __y_min))
   (setq _x_max (float __x_max))
   (setq _y_max (float __y_max))
   (setq _object_id (round __object_id))
   (setq _x_pos (float __x_pos))
   (setq _y_pos (float __y_pos))
   (setq _z_pos (float __z_pos))
   self)
  (:class_id
   (&optional __class_id)
   (if __class_id (setq _class_id __class_id)) _class_id)
  (:class_name
   (&optional __class_name)
   (if __class_name (setq _class_name __class_name)) _class_name)
  (:confidence
   (&optional __confidence)
   (if __confidence (setq _confidence __confidence)) _confidence)
  (:x_min
   (&optional __x_min)
   (if __x_min (setq _x_min __x_min)) _x_min)
  (:y_min
   (&optional __y_min)
   (if __y_min (setq _y_min __y_min)) _y_min)
  (:x_max
   (&optional __x_max)
   (if __x_max (setq _x_max __x_max)) _x_max)
  (:y_max
   (&optional __y_max)
   (if __y_max (setq _y_max __y_max)) _y_max)
  (:object_id
   (&optional __object_id)
   (if __object_id (setq _object_id __object_id)) _object_id)
  (:x_pos
   (&optional __x_pos)
   (if __x_pos (setq _x_pos __x_pos)) _x_pos)
  (:y_pos
   (&optional __y_pos)
   (if __y_pos (setq _y_pos __y_pos)) _y_pos)
  (:z_pos
   (&optional __z_pos)
   (if __z_pos (setq _z_pos __z_pos)) _z_pos)
  (:serialization-length
   ()
   (+
    ;; uint32 _class_id
    4
    ;; string _class_name
    4 (length _class_name)
    ;; float32 _confidence
    4
    ;; float32 _x_min
    4
    ;; float32 _y_min
    4
    ;; float32 _x_max
    4
    ;; float32 _y_max
    4
    ;; uint32 _object_id
    4
    ;; float32 _x_pos
    4
    ;; float32 _y_pos
    4
    ;; float32 _z_pos
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; uint32 _class_id
       (write-long _class_id s)
     ;; string _class_name
       (write-long (length _class_name) s) (princ _class_name s)
     ;; float32 _confidence
       (sys::poke _confidence (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _x_min
       (sys::poke _x_min (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _y_min
       (sys::poke _y_min (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _x_max
       (sys::poke _x_max (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _y_max
       (sys::poke _y_max (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; uint32 _object_id
       (write-long _object_id s)
     ;; float32 _x_pos
       (sys::poke _x_pos (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _y_pos
       (sys::poke _y_pos (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _z_pos
       (sys::poke _z_pos (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; uint32 _class_id
     (setq _class_id (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; string _class_name
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _class_name (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; float32 _confidence
     (setq _confidence (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _x_min
     (setq _x_min (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _y_min
     (setq _y_min (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _x_max
     (setq _x_max (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _y_max
     (setq _y_max (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; uint32 _object_id
     (setq _object_id (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; float32 _x_pos
     (setq _x_pos (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _y_pos
     (setq _y_pos (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _z_pos
     (setq _z_pos (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get fla_msgs::Detection :md5sum-) "f4c53395944a41874a6ab9783c6e93b6")
(setf (get fla_msgs::Detection :datatype-) "fla_msgs/Detection")
(setf (get fla_msgs::Detection :definition-)
      "uint32 class_id
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



(provide :fla_msgs/Detection "f4c53395944a41874a6ab9783c6e93b6")


