;; Auto-generated. Do not edit!


(when (boundp 'fla_msgs::NodeStatus)
  (if (not (find-package "FLA_MSGS"))
    (make-package "FLA_MSGS"))
  (shadow 'NodeStatus (find-package "FLA_MSGS")))
(unless (find-package "FLA_MSGS::NODESTATUS")
  (make-package "FLA_MSGS::NODESTATUS"))

(in-package "ROS")
;;//! \htmlinclude NodeStatus.msg.html


(intern "*INACTIVE*" (find-package "FLA_MSGS::NODESTATUS"))
(shadow '*INACTIVE* (find-package "FLA_MSGS::NODESTATUS"))
(defconstant fla_msgs::NodeStatus::*INACTIVE* 1)
(intern "*STARTING*" (find-package "FLA_MSGS::NODESTATUS"))
(shadow '*STARTING* (find-package "FLA_MSGS::NODESTATUS"))
(defconstant fla_msgs::NodeStatus::*STARTING* 2)
(intern "*INIT*" (find-package "FLA_MSGS::NODESTATUS"))
(shadow '*INIT* (find-package "FLA_MSGS::NODESTATUS"))
(defconstant fla_msgs::NodeStatus::*INIT* 3)
(intern "*READY*" (find-package "FLA_MSGS::NODESTATUS"))
(shadow '*READY* (find-package "FLA_MSGS::NODESTATUS"))
(defconstant fla_msgs::NodeStatus::*READY* 4)
(intern "*ALARM*" (find-package "FLA_MSGS::NODESTATUS"))
(shadow '*ALARM* (find-package "FLA_MSGS::NODESTATUS"))
(defconstant fla_msgs::NodeStatus::*ALARM* 5)
(intern "*FAIL*" (find-package "FLA_MSGS::NODESTATUS"))
(shadow '*FAIL* (find-package "FLA_MSGS::NODESTATUS"))
(defconstant fla_msgs::NodeStatus::*FAIL* 6)
(intern "*SILENT*" (find-package "FLA_MSGS::NODESTATUS"))
(shadow '*SILENT* (find-package "FLA_MSGS::NODESTATUS"))
(defconstant fla_msgs::NodeStatus::*SILENT* 7)
(defclass fla_msgs::NodeStatus
  :super ros::object
  :slots (_id _pid _status _arg _load ))

(defmethod fla_msgs::NodeStatus
  (:init
   (&key
    ((:id __id) 0)
    ((:pid __pid) 0)
    ((:status __status) 0)
    ((:arg __arg) 0)
    ((:load __load) 0.0)
    )
   (send-super :init)
   (setq _id (round __id))
   (setq _pid (round __pid))
   (setq _status (round __status))
   (setq _arg (round __arg))
   (setq _load (float __load))
   self)
  (:id
   (&optional __id)
   (if __id (setq _id __id)) _id)
  (:pid
   (&optional __pid)
   (if __pid (setq _pid __pid)) _pid)
  (:status
   (&optional __status)
   (if __status (setq _status __status)) _status)
  (:arg
   (&optional __arg)
   (if __arg (setq _arg __arg)) _arg)
  (:load
   (&optional __load)
   (if __load (setq _load __load)) _load)
  (:serialization-length
   ()
   (+
    ;; uint32 _id
    4
    ;; uint32 _pid
    4
    ;; uint8 _status
    1
    ;; int32 _arg
    4
    ;; float64 _load
    8
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; uint32 _id
       (write-long _id s)
     ;; uint32 _pid
       (write-long _pid s)
     ;; uint8 _status
       (write-byte _status s)
     ;; int32 _arg
       (write-long _arg s)
     ;; float64 _load
       (sys::poke _load (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; uint32 _id
     (setq _id (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; uint32 _pid
     (setq _pid (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; uint8 _status
     (setq _status (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; int32 _arg
     (setq _arg (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; float64 _load
     (setq _load (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;;
   self)
  )

(setf (get fla_msgs::NodeStatus :md5sum-) "bc0e9b8f7a292a18c6b50b43586a86d8")
(setf (get fla_msgs::NodeStatus :datatype-) "fla_msgs/NodeStatus")
(setf (get fla_msgs::NodeStatus :definition-)
      "uint8 INACTIVE=1
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



(provide :fla_msgs/NodeStatus "bc0e9b8f7a292a18c6b50b43586a86d8")


