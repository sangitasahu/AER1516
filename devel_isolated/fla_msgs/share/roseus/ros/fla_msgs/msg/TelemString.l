;; Auto-generated. Do not edit!


(when (boundp 'fla_msgs::TelemString)
  (if (not (find-package "FLA_MSGS"))
    (make-package "FLA_MSGS"))
  (shadow 'TelemString (find-package "FLA_MSGS")))
(unless (find-package "FLA_MSGS::TELEMSTRING")
  (make-package "FLA_MSGS::TELEMSTRING"))

(in-package "ROS")
;;//! \htmlinclude TelemString.msg.html


(defclass fla_msgs::TelemString
  :super ros::object
  :slots (_nodeid _text ))

(defmethod fla_msgs::TelemString
  (:init
   (&key
    ((:nodeid __nodeid) 0)
    ((:text __text) "")
    )
   (send-super :init)
   (setq _nodeid (round __nodeid))
   (setq _text (string __text))
   self)
  (:nodeid
   (&optional __nodeid)
   (if __nodeid (setq _nodeid __nodeid)) _nodeid)
  (:text
   (&optional __text)
   (if __text (setq _text __text)) _text)
  (:serialization-length
   ()
   (+
    ;; uint32 _nodeid
    4
    ;; string _text
    4 (length _text)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; uint32 _nodeid
       (write-long _nodeid s)
     ;; string _text
       (write-long (length _text) s) (princ _text s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; uint32 _nodeid
     (setq _nodeid (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; string _text
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _text (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;;
   self)
  )

(setf (get fla_msgs::TelemString :md5sum-) "90c5a36c3a388c55ce89c05cc31213e3")
(setf (get fla_msgs::TelemString :datatype-) "fla_msgs/TelemString")
(setf (get fla_msgs::TelemString :definition-)
      "uint32 nodeid
string text

")



(provide :fla_msgs/TelemString "90c5a36c3a388c55ce89c05cc31213e3")


