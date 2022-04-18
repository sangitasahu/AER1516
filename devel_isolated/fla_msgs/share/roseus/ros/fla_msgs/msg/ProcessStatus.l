;; Auto-generated. Do not edit!


(when (boundp 'fla_msgs::ProcessStatus)
  (if (not (find-package "FLA_MSGS"))
    (make-package "FLA_MSGS"))
  (shadow 'ProcessStatus (find-package "FLA_MSGS")))
(unless (find-package "FLA_MSGS::PROCESSSTATUS")
  (make-package "FLA_MSGS::PROCESSSTATUS"))

(in-package "ROS")
;;//! \htmlinclude ProcessStatus.msg.html


(intern "*INIT*" (find-package "FLA_MSGS::PROCESSSTATUS"))
(shadow '*INIT* (find-package "FLA_MSGS::PROCESSSTATUS"))
(defconstant fla_msgs::ProcessStatus::*INIT* 3)
(intern "*READY*" (find-package "FLA_MSGS::PROCESSSTATUS"))
(shadow '*READY* (find-package "FLA_MSGS::PROCESSSTATUS"))
(defconstant fla_msgs::ProcessStatus::*READY* 4)
(intern "*ALARM*" (find-package "FLA_MSGS::PROCESSSTATUS"))
(shadow '*ALARM* (find-package "FLA_MSGS::PROCESSSTATUS"))
(defconstant fla_msgs::ProcessStatus::*ALARM* 5)
(intern "*FAIL*" (find-package "FLA_MSGS::PROCESSSTATUS"))
(shadow '*FAIL* (find-package "FLA_MSGS::PROCESSSTATUS"))
(defconstant fla_msgs::ProcessStatus::*FAIL* 6)
(defclass fla_msgs::ProcessStatus
  :super ros::object
  :slots (_id _pid _status _arg ))

(defmethod fla_msgs::ProcessStatus
  (:init
   (&key
    ((:id __id) 0)
    ((:pid __pid) 0)
    ((:status __status) 0)
    ((:arg __arg) 0)
    )
   (send-super :init)
   (setq _id (round __id))
   (setq _pid (round __pid))
   (setq _status (round __status))
   (setq _arg (round __arg))
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
   ;;
   self)
  )

(setf (get fla_msgs::ProcessStatus :md5sum-) "af0cc9ae0197397f393fc2c50073a7a2")
(setf (get fla_msgs::ProcessStatus :datatype-) "fla_msgs/ProcessStatus")
(setf (get fla_msgs::ProcessStatus :definition-)
      "uint8 INIT=3
uint8 READY=4
uint8 ALARM=5
uint8 FAIL=6
uint32 id
uint32 pid
uint8 status
int32 arg

")



(provide :fla_msgs/ProcessStatus "af0cc9ae0197397f393fc2c50073a7a2")


