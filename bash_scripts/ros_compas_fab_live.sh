#!/bin/bash

cd $HOME
source ur_ws/devel/setup.bash
gnome-terminal -- /bin/sh -c "roscore; exec bash" &
sleep 3
gnome-terminal -- /bin/sh -c "source ~/ur_ws/devel/setup.bash; roslaunch rosbridge_server rosbridge_websocket.launch; exec bash" &
sleep 3
gnome-terminal -- /bin/sh -c "source ~/ur_ws/devel/setup.bash; roslaunch file_server file_server.launch; exec bash" &
sleep 3
gnome-terminal -- /bin/sh -c "source ~/ur_ws/devel/setup.bash; roslaunch ur_robot_driver ur10e_bringup.launch robot_ip:=192.168.1.25 kinematics_config:='/home/angel/my_robot_calibration.yaml'; exec bash" &
sleep 3
gnome-terminal -- /bin/sh -c "source ~/ur_ws/devel/setup.bash; roslaunch ur10_e_moveit_config  ur10_e_moveit_planning_execution.launch limited:=true; exec bash" &
sleep 3
gnome-terminal -- /bin/sh -c "source ~/ur_ws/devel/setup.bash; roslaunch ur10_e_moveit_config moveit_rviz.launch config:=true; exec bash" &
sleep 3
gnome-terminal -- /bin/sh -c "source ~/ur_ws/devel/setup.bash; roslaunch cv_basics cv_basics_py.launch" &

exit 0
