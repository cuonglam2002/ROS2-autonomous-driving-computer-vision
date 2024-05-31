# !/bin/bash

# Lấy PID của các nodes ROS 2
# node_pids=$(ps aux | grep ros2 | grep -v grep | awk '{print $2}')

# # Dừng các nodes bằng cách sử dụng PID
# for pid in $node_pids; do
#     kill $pid
# done
# ros2 daemon stop
# ros2 daemon start
# ros2 daemon status