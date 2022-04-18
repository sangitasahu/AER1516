;; Auto-generated. Do not edit!


(when (boundp 'fla_msgs::FlightCommand)
  (if (not (find-package "FLA_MSGS"))
    (make-package "FLA_MSGS"))
  (shadow 'FlightCommand (find-package "FLA_MSGS")))
(unless (find-package "FLA_MSGS::FLIGHTCOMMAND")
  (make-package "FLA_MSGS::FLIGHTCOMMAND"))

(in-package "ROS")
;;//! \htmlinclude FlightCommand.msg.html


(intern "*CMD_TAKEOFF*" (find-package "FLA_MSGS::FLIGHTCOMMAND"))
(shadow '*CMD_TAKEOFF* (find-package "FLA_MSGS::FLIGHTCOMMAND"))
(defconstant fla_msgs::FlightCommand::*CMD_TAKEOFF* 0)
(intern "*CMD_GO*" (find-package "FLA_MSGS::FLIGHTCOMMAND"))
(shadow '*CMD_GO* (find-package "FLA_MSGS::FLIGHTCOMMAND"))
(defconstant fla_msgs::FlightCommand::*CMD_GO* 1)
(intern "*CMD_LAND*" (find-package "FLA_MSGS::FLIGHTCOMMAND"))
(shadow '*CMD_LAND* (find-package "FLA_MSGS::FLIGHTCOMMAND"))
(defconstant fla_msgs::FlightCommand::*CMD_LAND* 2)
(intern "*CMD_HOVER*" (find-package "FLA_MSGS::FLIGHTCOMMAND"))
(shadow '*CMD_HOVER* (find-package "FLA_MSGS::FLIGHTCOMMAND"))
(defconstant fla_msgs::FlightCommand::*CMD_HOVER* 3)
(intern "*CMD_INIT*" (find-package "FLA_MSGS::FLIGHTCOMMAND"))
(shadow '*CMD_INIT* (find-package "FLA_MSGS::FLIGHTCOMMAND"))
(defconstant fla_msgs::FlightCommand::*CMD_INIT* 4)
(intern "*CMD_KILL*" (find-package "FLA_MSGS::FLIGHTCOMMAND"))
(shadow '*CMD_KILL* (find-package "FLA_MSGS::FLIGHTCOMMAND"))
(defconstant fla_msgs::FlightCommand::*CMD_KILL* 5)
(intern "*CMD_START_SEARCH*" (find-package "FLA_MSGS::FLIGHTCOMMAND"))
(shadow '*CMD_START_SEARCH* (find-package "FLA_MSGS::FLIGHTCOMMAND"))
(defconstant fla_msgs::FlightCommand::*CMD_START_SEARCH* 6)
(defclass fla_msgs::FlightCommand
  :super ros::object
  :slots (_command ))

(defmethod fla_msgs::FlightCommand
  (:init
   (&key
    ((:command __command) 0)
    )
   (send-super :init)
   (setq _command (round __command))
   self)
  (:command
   (&optional __command)
   (if __command (setq _command __command)) _command)
  (:serialization-length
   ()
   (+
    ;; uint8 _command
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; uint8 _command
       (write-byte _command s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; uint8 _command
     (setq _command (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;;
   self)
  )

(setf (get fla_msgs::FlightCommand :md5sum-) "3fa8261f600a67acfe8d66a49672fd7b")
(setf (get fla_msgs::FlightCommand :datatype-) "fla_msgs/FlightCommand")
(setf (get fla_msgs::FlightCommand :definition-)
      "uint8 CMD_TAKEOFF=0
uint8 CMD_GO=1
uint8 CMD_LAND=2
uint8 CMD_HOVER=3
uint8 CMD_INIT=4
uint8 CMD_KILL=5
uint8 CMD_START_SEARCH=6
uint8 command

")



(provide :fla_msgs/FlightCommand "3fa8261f600a67acfe8d66a49672fd7b")


