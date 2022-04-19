Note that the forest.bag is recorded using the Gazebo time. Therefore, to visualize it in Rviz (along other markers from other nodes), follow these steps:


roslaunch acl_sim sim.launch quad:=SQ01s world_name:="empty.world"  #We won't use Gazebo, but this solves some time issues with Rviz


Launch rviz


rosrun tf static_transform_publisher 0.0 0.0 0.0 0.0 0.0 0.0 1.0 map world 10


rosbag play iros_office.bag



#And now run other nodes that publish other stuff