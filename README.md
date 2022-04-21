# CG2111A
Make sure these lines are at the bottom of your ~/.bashrc in your laptop

    export ROS_MASTER_URI=http://(Ip Address of Pi):11311
    export ROS_HOSTNAME=(Ip Address of Laptop)
    source /opt/ros/noetic/setup.bash
    source ~/Desktop/catkin_ws/devel/setup.bash

These lines need to be added at the bottom of ~/.bashrc for RPi

    export ROS_MASTER_URI=http://(Ip Address of Pi):11311
    export ROS_HOSTNAME=(Ip Address of Pi)
