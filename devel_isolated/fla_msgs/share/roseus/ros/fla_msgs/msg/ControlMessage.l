;; Auto-generated. Do not edit!


(when (boundp 'fla_msgs::ControlMessage)
  (if (not (find-package "FLA_MSGS"))
    (make-package "FLA_MSGS"))
  (shadow 'ControlMessage (find-package "FLA_MSGS")))
(unless (find-package "FLA_MSGS::CONTROLMESSAGE")
  (make-package "FLA_MSGS::CONTROLMESSAGE"))

(in-package "ROS")
;;//! \htmlinclude ControlMessage.msg.html


(defclass fla_msgs::ControlMessage
  :super ros::object
  :slots (_stamp _len _data ))

(defmethod fla_msgs::ControlMessage
  (:init
   (&key
    ((:stamp __stamp) (instance ros::time :init))
    ((:len __len) 0)
    ((:data __data) (make-array 32 :initial-element 0 :element-type :char))
    )
   (send-super :init)
   (setq _stamp __stamp)
   (setq _len (round __len))
   (setq _data __data)
   self)
  (:stamp
   (&optional __stamp)
   (if __stamp (setq _stamp __stamp)) _stamp)
  (:len
   (&optional __len)
   (if __len (setq _len __len)) _len)
  (:data
   (&optional __data)
   (if __data (setq _data __data)) _data)
  (:serialization-length
   ()
   (+
    ;; time _stamp
    8
    ;; uint32 _len
    4
    ;; uint8[32] _data
    (* 1    32)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; time _stamp
       (write-long (send _stamp :sec) s) (write-long (send _stamp :nsec) s)
     ;; uint32 _len
       (write-long _len s)
     ;; uint8[32] _data
     (princ _data s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; time _stamp
     (send _stamp :sec (sys::peek buf ptr- :integer)) (incf ptr- 4)  (send _stamp :nsec (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; uint32 _len
     (setq _len (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; uint8[32] _data
   (setq _data (make-array 32 :element-type :char))
   (replace _data buf :start2 ptr-) (incf ptr- 32)
   ;;
   self)
  )

(setf (get fla_msgs::ControlMessage :md5sum-) "61a82e2f5b8cbb1d6b3046fb9bb5d79f")
(setf (get fla_msgs::ControlMessage :datatype-) "fla_msgs/ControlMessage")
(setf (get fla_msgs::ControlMessage :definition-)
      "time stamp
uint32 len
uint8[32] data

")



(provide :fla_msgs/ControlMessage "61a82e2f5b8cbb1d6b3046fb9bb5d79f")


