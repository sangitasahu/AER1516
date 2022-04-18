;; Auto-generated. Do not edit!


(when (boundp 'fla_msgs::NodeList)
  (if (not (find-package "FLA_MSGS"))
    (make-package "FLA_MSGS"))
  (shadow 'NodeList (find-package "FLA_MSGS")))
(unless (find-package "FLA_MSGS::NODELIST")
  (make-package "FLA_MSGS::NODELIST"))

(in-package "ROS")
;;//! \htmlinclude NodeList.msg.html


(defclass fla_msgs::NodeList
  :super ros::object
  :slots (_stamp _nodelog ))

(defmethod fla_msgs::NodeList
  (:init
   (&key
    ((:stamp __stamp) (instance ros::time :init))
    ((:nodelog __nodelog) (let (r) (dotimes (i 0) (push (instance fla_msgs::NodeStatus :init) r)) r))
    )
   (send-super :init)
   (setq _stamp __stamp)
   (setq _nodelog __nodelog)
   self)
  (:stamp
   (&optional __stamp)
   (if __stamp (setq _stamp __stamp)) _stamp)
  (:nodelog
   (&rest __nodelog)
   (if (keywordp (car __nodelog))
       (send* _nodelog __nodelog)
     (progn
       (if __nodelog (setq _nodelog (car __nodelog)))
       _nodelog)))
  (:serialization-length
   ()
   (+
    ;; time _stamp
    8
    ;; fla_msgs/NodeStatus[] _nodelog
    (apply #'+ (send-all _nodelog :serialization-length)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; time _stamp
       (write-long (send _stamp :sec) s) (write-long (send _stamp :nsec) s)
     ;; fla_msgs/NodeStatus[] _nodelog
     (write-long (length _nodelog) s)
     (dolist (elem _nodelog)
       (send elem :serialize s)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; time _stamp
     (send _stamp :sec (sys::peek buf ptr- :integer)) (incf ptr- 4)  (send _stamp :nsec (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; fla_msgs/NodeStatus[] _nodelog
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _nodelog (let (r) (dotimes (i n) (push (instance fla_msgs::NodeStatus :init) r)) r))
     (dolist (elem- _nodelog)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;;
   self)
  )

(setf (get fla_msgs::NodeList :md5sum-) "ef5328a9c9c28a94588f0215d3bce4ca")
(setf (get fla_msgs::NodeList :datatype-) "fla_msgs/NodeList")
(setf (get fla_msgs::NodeList :definition-)
      "time stamp
NodeStatus[] nodelog

================================================================================
MSG: fla_msgs/NodeStatus
uint8 INACTIVE=1
uint8 STARTING=2
uint8 INIT=3
uint8 READY=4
uint8 ALARM=5
uint8 FAIL=6
uint8 SILENT=7
uint32 id
uint32 pid
uint8 status
int32 arg
float64 load

")



(provide :fla_msgs/NodeList "ef5328a9c9c28a94588f0215d3bce4ca")


