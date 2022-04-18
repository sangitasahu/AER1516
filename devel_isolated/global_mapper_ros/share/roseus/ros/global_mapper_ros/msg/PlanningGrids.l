;; Auto-generated. Do not edit!


(when (boundp 'global_mapper_ros::PlanningGrids)
  (if (not (find-package "GLOBAL_MAPPER_ROS"))
    (make-package "GLOBAL_MAPPER_ROS"))
  (shadow 'PlanningGrids (find-package "GLOBAL_MAPPER_ROS")))
(unless (find-package "GLOBAL_MAPPER_ROS::PLANNINGGRIDS")
  (make-package "GLOBAL_MAPPER_ROS::PLANNINGGRIDS"))

(in-package "ROS")
;;//! \htmlinclude PlanningGrids.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass global_mapper_ros::PlanningGrids
  :super ros::object
  :slots (_header _projected_goal _origin _grid_dimensions _resolution _occupancy_data _distance_data _cost_data _occupied_threshold _dmax ))

(defmethod global_mapper_ros::PlanningGrids
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:projected_goal __projected_goal) (make-array 3 :initial-element 0.0 :element-type :float))
    ((:origin __origin) (make-array 3 :initial-element 0.0 :element-type :float))
    ((:grid_dimensions __grid_dimensions) (make-array 3 :initial-element 0 :element-type :integer))
    ((:resolution __resolution) 0.0)
    ((:occupancy_data __occupancy_data) (make-array 0 :initial-element 0.0 :element-type :float))
    ((:distance_data __distance_data) (make-array 0 :initial-element 0 :element-type :integer))
    ((:cost_data __cost_data) (make-array 0 :initial-element 0 :element-type :integer))
    ((:occupied_threshold __occupied_threshold) 0.0)
    ((:dmax __dmax) 0)
    )
   (send-super :init)
   (setq _header __header)
   (setq _projected_goal __projected_goal)
   (setq _origin __origin)
   (setq _grid_dimensions __grid_dimensions)
   (setq _resolution (float __resolution))
   (setq _occupancy_data __occupancy_data)
   (setq _distance_data __distance_data)
   (setq _cost_data __cost_data)
   (setq _occupied_threshold (float __occupied_threshold))
   (setq _dmax (round __dmax))
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:projected_goal
   (&optional __projected_goal)
   (if __projected_goal (setq _projected_goal __projected_goal)) _projected_goal)
  (:origin
   (&optional __origin)
   (if __origin (setq _origin __origin)) _origin)
  (:grid_dimensions
   (&optional __grid_dimensions)
   (if __grid_dimensions (setq _grid_dimensions __grid_dimensions)) _grid_dimensions)
  (:resolution
   (&optional __resolution)
   (if __resolution (setq _resolution __resolution)) _resolution)
  (:occupancy_data
   (&optional __occupancy_data)
   (if __occupancy_data (setq _occupancy_data __occupancy_data)) _occupancy_data)
  (:distance_data
   (&optional __distance_data)
   (if __distance_data (setq _distance_data __distance_data)) _distance_data)
  (:cost_data
   (&optional __cost_data)
   (if __cost_data (setq _cost_data __cost_data)) _cost_data)
  (:occupied_threshold
   (&optional __occupied_threshold)
   (if __occupied_threshold (setq _occupied_threshold __occupied_threshold)) _occupied_threshold)
  (:dmax
   (&optional __dmax)
   (if __dmax (setq _dmax __dmax)) _dmax)
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; float64[3] _projected_goal
    (* 8    3)
    ;; float64[3] _origin
    (* 8    3)
    ;; int32[3] _grid_dimensions
    (* 4    3)
    ;; float64 _resolution
    8
    ;; float32[] _occupancy_data
    (* 4    (length _occupancy_data)) 4
    ;; int32[] _distance_data
    (* 4    (length _distance_data)) 4
    ;; int32[] _cost_data
    (* 4    (length _cost_data)) 4
    ;; float32 _occupied_threshold
    4
    ;; int32 _dmax
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; float64[3] _projected_goal
     (dotimes (i 3)
       (sys::poke (elt _projected_goal i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;; float64[3] _origin
     (dotimes (i 3)
       (sys::poke (elt _origin i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;; int32[3] _grid_dimensions
     (dotimes (i 3)
       (write-long (elt _grid_dimensions i) s)
       )
     ;; float64 _resolution
       (sys::poke _resolution (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float32[] _occupancy_data
     (write-long (length _occupancy_data) s)
     (dotimes (i (length _occupancy_data))
       (sys::poke (elt _occupancy_data i) (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
       )
     ;; int32[] _distance_data
     (write-long (length _distance_data) s)
     (dotimes (i (length _distance_data))
       (write-long (elt _distance_data i) s)
       )
     ;; int32[] _cost_data
     (write-long (length _cost_data) s)
     (dotimes (i (length _cost_data))
       (write-long (elt _cost_data i) s)
       )
     ;; float32 _occupied_threshold
       (sys::poke _occupied_threshold (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; int32 _dmax
       (write-long _dmax s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; float64[3] _projected_goal
   (dotimes (i (length _projected_goal))
     (setf (elt _projected_goal i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     )
   ;; float64[3] _origin
   (dotimes (i (length _origin))
     (setf (elt _origin i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     )
   ;; int32[3] _grid_dimensions
   (dotimes (i (length _grid_dimensions))
     (setf (elt _grid_dimensions i) (sys::peek buf ptr- :integer)) (incf ptr- 4)
     )
   ;; float64 _resolution
     (setq _resolution (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float32[] _occupancy_data
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _occupancy_data (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _occupancy_data i) (sys::peek buf ptr- :float)) (incf ptr- 4)
     ))
   ;; int32[] _distance_data
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _distance_data (instantiate integer-vector n))
     (dotimes (i n)
     (setf (elt _distance_data i) (sys::peek buf ptr- :integer)) (incf ptr- 4)
     ))
   ;; int32[] _cost_data
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _cost_data (instantiate integer-vector n))
     (dotimes (i n)
     (setf (elt _cost_data i) (sys::peek buf ptr- :integer)) (incf ptr- 4)
     ))
   ;; float32 _occupied_threshold
     (setq _occupied_threshold (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; int32 _dmax
     (setq _dmax (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get global_mapper_ros::PlanningGrids :md5sum-) "86653f2401f7e451293296585b426430")
(setf (get global_mapper_ros::PlanningGrids :datatype-) "global_mapper_ros/PlanningGrids")
(setf (get global_mapper_ros::PlanningGrids :definition-)
      "Header header

float64[3] projected_goal  # current position of projected goal within the grid
float64[3] origin  # center of map in global frame [m]
int32[3] grid_dimensions  # size of map [voxels]
float64 resolution  # voxel size [m]

float32[] occupancy_data
int32[] distance_data
int32[] cost_data

float32 occupied_threshold #occupancy values above this are considered 'occupied'
int32 dmax  #max squared distance reported in distance_grid

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

")



(provide :global_mapper_ros/PlanningGrids "86653f2401f7e451293296585b426430")


