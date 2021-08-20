#!/bin/bash

cd $HOME
source ur_ws/devel/setup.bash
gnome-terminal -- /bin/sh -c "roscore; exec bash" &
sleep 3
gnome-terminal -- /bin/sh -c "source ~/ur_ws/devel/setup.bash; roslaunch rosbridge_server rosbridge_websocket.launch; exec bash" &
sleep 3
gnome-terminal -- /bin/sh -c "source ~/ur_ws/devel/setup.bash; roslaunch file_server file_server.launch; exec bash" &
sleep 3
gnome-terminal -- /bin/sh -c "source ~/ur_ws/devel/setup.bash; roslaunch ur10_e_moveit_config demo.launch; exec bash" &
sleep 3
gnome-terminal -- /bin/sh -c "source ~/ur_ws/devel/setup.bash; roslaunch cv_basics cv_basics_py.launch" &

exit 0
