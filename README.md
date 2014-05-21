nextage_ros_seqplay_util
========================

ROS package of SequencePlayer utility for NEXTAGE OPEN

Build
-----

~~~ sh
$ cd /path/to/catkin_ws/src
$ git clone git@github.com:youtalk/nextage_ros_seqplay_util.git
$ cd ..
$ catkin_make
~~~


Execution
---------
**terminal #1**
~~~sh
$ roscore
~~~

**terminal #2**
~~~sh
$ rtmlaunch nextage_ros_bridge nextage_ros_bridge_simulation.launch
~~~

**terminal #3**
~~~sh
$ rosrun nextage_ros_seqplay_util seqplay_util_server.py
~~~

**terminal #4**
~~~sh
$ rosservice list /nextage_ros_seqplay_util
/nextage_ros_seqplay_util/get_loggers
/nextage_ros_seqplay_util/goInitial
/nextage_ros_seqplay_util/goOffPose
/nextage_ros_seqplay_util/setTargetPoseRelative
/nextage_ros_seqplay_util/set_logger_level
$ rosservice call /nextage_ros_seqplay_util/goInitial 2.0
$ rosservice call /nextage_ros_seqplay_util/setTargetPoseRelative "larm" "[0, 0, 0.1]" "[0, 0, 0]" 2
$ rosservice call /nextage_ros_seqplay_util/goOffPose 2.0
~~~
