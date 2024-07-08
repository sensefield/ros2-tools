#!/bin/bash

PARENT_FRAME=$1
CHILD_FRAME=$2

ros2 bag play ${HOME}/Downloads/rosbag2_2024_05_30-15_16_02_0.db3 -l --clock --remap /tf:=/a /tf_static:=/b &
ros2 run tf2_ros static_transform_publisher 0.0 0.0 1.0 0.0 0.0 0.00 ${PARENT_FRAME} ${CHILD_FRAME} &
rviz2 -d calibration.rviz &
python3 gui.py --frame_id ${PARENT_FRAME} --child_frame_id ${CHILD_FRAME} &
ros2 run tf2_ros tf2_echo ${PARENT_FRAME} ${CHILD_FRAME}
