;; Auto-generated. Do not edit!


(when (boundp 'map_simulator::Map3D)
  (if (not (find-package "MAP_SIMULATOR"))
    (make-package "MAP_SIMULATOR"))
  (shadow 'Map3D (find-package "MAP_SIMULATOR")))
(unless (find-package "MAP_SIMULATOR::MAP3D")
  (make-package "MAP_SIMULATOR::MAP3D"))

(in-package "ROS")
;;//! \htmlinclude Map3D.msg.html
(if (not (find-package "GEOMETRY_MSGS"))
  (ros::roseus-add-msgs "geometry_msgs"))


(defclass map_simulator::Map3D
  :super ros::object
  :slots (_nodeCoordinates _occupiedStatus _nodeDistances ))

(defmethod map_simulator::Map3D
  (:init
   (&key
    ((:nodeCoordinates __nodeCoordinates) (let (r) (dotimes (i 0) (push (instance geometry_msgs::Vector3 :init) r)) r))
    ((:occupiedStatus __occupiedStatus) (let (r) (dotimes (i 0) (push nil r)) r))
    ((:nodeDistances __nodeDistances) (make-array 0 :initial-element 0 :element-type :integer))
    )
   (send-super :init)
   (setq _nodeCoordinates __nodeCoordinates)
   (setq _occupiedStatus __occupiedStatus)
   (setq _nodeDistances __nodeDistances)
   self)
  (:nodeCoordinates
   (&rest __nodeCoordinates)
   (if (keywordp (car __nodeCoordinates))
       (send* _nodeCoordinates __nodeCoordinates)
     (progn
       (if __nodeCoordinates (setq _nodeCoordinates (car __nodeCoordinates)))
       _nodeCoordinates)))
  (:occupiedStatus
   (&optional __occupiedStatus)
   (if __occupiedStatus (setq _occupiedStatus __occupiedStatus)) _occupiedStatus)
  (:nodeDistances
   (&optional __nodeDistances)
   (if __nodeDistances (setq _nodeDistances __nodeDistances)) _nodeDistances)
  (:serialization-length
   ()
   (+
    ;; geometry_msgs/Vector3[] _nodeCoordinates
    (apply #'+ (send-all _nodeCoordinates :serialization-length)) 4
    ;; bool[] _occupiedStatus
    (* 1    (length _occupiedStatus)) 4
    ;; uint16[] _nodeDistances
    (* 2    (length _nodeDistances)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; geometry_msgs/Vector3[] _nodeCoordinates
     (write-long (length _nodeCoordinates) s)
     (dolist (elem _nodeCoordinates)
       (send elem :serialize s)
       )
     ;; bool[] _occupiedStatus
     (write-long (length _occupiedStatus) s)
     (dotimes (i (length _occupiedStatus))
       (if (elt _occupiedStatus i) (write-byte -1 s) (write-byte 0 s))
       )
     ;; uint16[] _nodeDistances
     (write-long (length _nodeDistances) s)
     (dotimes (i (length _nodeDistances))
       (write-word (elt _nodeDistances i) s)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; geometry_msgs/Vector3[] _nodeCoordinates
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _nodeCoordinates (let (r) (dotimes (i n) (push (instance geometry_msgs::Vector3 :init) r)) r))
     (dolist (elem- _nodeCoordinates)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;; bool[] _occupiedStatus
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _occupiedStatus (make-list n))
     (dotimes (i n)
     (setf (elt _occupiedStatus i) (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
     ))
   ;; uint16[] _nodeDistances
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _nodeDistances (instantiate integer-vector n))
     (dotimes (i n)
     (setf (elt _nodeDistances i) (sys::peek buf ptr- :short)) (incf ptr- 2)
     ))
   ;;
   self)
  )

(setf (get map_simulator::Map3D :md5sum-) "ad6c8afc9d0105744f261886c76d5da8")
(setf (get map_simulator::Map3D :datatype-) "map_simulator/Map3D")
(setf (get map_simulator::Map3D :definition-)
      "geometry_msgs/Vector3[] nodeCoordinates
bool[] occupiedStatus
uint16[] nodeDistances


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



(provide :map_simulator/Map3D "ad6c8afc9d0105744f261886c76d5da8")


