#!/usr/bin/env python
#This script enables you to run a bunch of simulations of the random forest worlds. 
#Command: python runSimForest.py 0; python runSimForest.py 1; python runSimForest.py 2; python runSimForest.py 3; python runSimForest.py 4; python runSimForest.py 5; python runSimForest.py 6; python runSimForest.py 7; python runSimForest.py 8; python runSimForest.py 9;      

#The total distance is saved in distance.txt


import rospy
import subprocess
import signal
import os
import time
import sys

from geometry_msgs.msg import PoseStamped, PointStamped, Vector3
from acl_msgs.msg import QuadFlightMode, QuadGoal, TermGoal, State
import numpy as np

finish=False;
distance=0;
xlast=0;
ylast=0;
zlast=0;
i=0

def callback(data):
     global finish
     global distance
     global xlast
     global ylast
     global zlast
     global x
     global y
     global z
     global i

     print(data.pos.x)
     x=data.pos.x;
     y=data.pos.y;
     z=data.pos.z;
     
     i=i+1
     if (i%20==0):
     	increment = np.array([xlast-x,ylast-y,zlast-z])
     	distance=distance+np.linalg.norm(increment)
     	xlast=x
        ylast=y
        zlast=z
     
     if ((x<=51 and x>=49) and (y<=51 and y>=49)):
     	finish=True

     print("Distance PYTHON=", distance)


    
def listener(seed):
    global finish
    global x
    global y
    global z
    global distance

    #gazebo = subprocess.Popen("roscore",shell=True )

    #time.sleep(3)
    rospy.init_node('listener')
    rospy.Subscriber("/SQ01s/state", State, callback)

    # spin() simply keeps python from exiting until this node is stopped
    for a in range(0, 1):

        seed=seed[1]


        time.sleep(3)

        gazebo = subprocess.Popen("roslaunch acl_sim sim.launch quad:=SQ01s world_name:=density01seed"+repr(seed)+".world",shell=True )
        time.sleep(15)

        quad_sim = subprocess.Popen("roslaunch quad_sim quad_sim.launch gazebo:=true veh:=SQ num:=01 z:=0.1",shell=True )
        time.sleep(5)

        mapper = subprocess.Popen("roslaunch global_mapper_ros global_mapper_node.launch quad:=SQ01s",shell=True )
        time.sleep(5)

        gui=subprocess.Popen("roslaunch acl_demos cvx_fsm.launch quad:=SQ01s vicon_room:=1",shell=True )
        time.sleep(8)

        cvx = subprocess.Popen("roslaunch cvx cvx.launch quad:=SQ01s",shell=True )
        time.sleep(4)


        pub_goal=subprocess.Popen("rostopic pub /move_base_simple/goal geometry_msgs/PoseStamped \'{header: {stamp: now, frame_id: \"world\"}, pose: {position: {x: 0, y: 0, z: 1}, orientation: {w: 1.0}}}\'",shell=True )
        time.sleep(2)
        pub_start_signal=subprocess.Popen("rostopic pub /SQ01s/flightmode acl_msgs/QuadFlightMode \'{mode: 4}\'",shell=True );
        time.sleep(8)
        #os.system ("rosnode kill /SQ01s/global_mapper_ros") #Kill the mapper (to avoid noise in the takeoff)
        #time.sleep(10)
       # mapper = subprocess.Popen("roslaunch global_mapper_ros global_mapper_node.launch quad:=SQ01s",shell=True )
       # time.sleep(10)
        pub_goal=subprocess.Popen("rostopic pub /move_base_simple/goal geometry_msgs/PoseStamped \'{header: {stamp: now, frame_id: \"world\"}, pose: {position: {x: 50, y: 50, z: 1}, orientation: {w: 1.0}}}\'",shell=True )
        distance=0

        start = time.time()

        time.sleep(5)
       # while (not((x<=51 and x>=49) and (y<=51 and y>=49)) and time.time()-start<=60*15):
        while (not((x<=51 and x>=49) and (y<=51 and y>=49)) and time.time()-start<=60*15):
        	#print ("Python x=",x)
        	#print ("Python y=",y)
        	pass
    
        file = open('distances.txt','a') 
        file.write(repr(seed)+", "+repr(x)+", "+repr(y)+", "+repr(z)+", "+repr(distance)+", "+repr(time.time()-start) );
        file.write("\n") 
        file.close()


        time.sleep(3)

        os.killpg(os.getpgid(gazebo.pid), signal.SIGTERM)
        os.killpg(os.getpgid(quad_sim.pid), signal.SIGTERM)
        os.killpg(os.getpgid(mapper.pid), signal.SIGTERM)
        os.killpg(os.getpgid(gui.pid), signal.SIGTERM)
        os.killpg(os.getpgid(cvx.pid), signal.SIGTERM)
        os.killpg(os.getpgid(pub_goal.pid), signal.SIGTERM)


        pub_goal.terminate()
        pub_goal.kill()
        gui.kill()
        gui.terminate()
        os.system ("rosnode kill /SQ01s/global_mapper_ros")
        os.system ("rosnode kill /SQ01s/cntrl /SQ01s/gazebo_relay /SQ01s/quad_sim /SQ01s/spawn_robot /SQ01s/vicon_relay /behavior_selector /camera2body /vicon2world")
        #os.system ("rosnode kill -a")
        time.sleep(10)
        #os.system ("killall -9 gazebo & killall -9 gzserver  & killall -9 gzclient & killall -9 roscore & killall -9 rosmaster")
        os.system ("killall -9 gazebo & killall -9 gzserver  & killall -9 gzclient")
        os.system ("rosnode kill /SQ01s/cntrl /SQ01s/cvx /SQ01s/fsm /SQ01s/gazebo_relay /SQ01s/global_mapper_ros /SQ01s/quad_sim /SQ01s/vicon_relay /behavior_selector /camera2body /gazebo /rqt_gui /rviz /vicon2world")

        x=0
        y=0
        z=0
        time.sleep(10)
        print("bucle terminado")

    #rospy.spin()
    print("Saliendo de esclavo arriba")




if True:
    listener(sys.argv)
    print("Saliendo de esclavo")
    #sys.exit("Saliendo de esclavo")
    print("Antes de return")
    raise SystemExit
    print("Despes de return")




