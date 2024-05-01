#!/bin/bash

DASHCAM_VIDEO_FILE="$1"
ROS_BAG_FILE="$2"
OFFSET="$3"

source catkin_ws/devel/setup.bash
roscore &
rosrun rqt_mypkg rqt_mypkg &
rqt_pid=$!


sleep 3

width=684
height=360

rosbag play -u 15 -q -s $OFFSET $ROS_BAG_FILE &

ffmpeg -f x11grab -framerate 30 -video_size ${width}x${height} -window_id 0x2c00006 -y -t 10 -i 0.0 dashboard.mp4

ffmpeg -i $DASHCAM_VIDEO_FILE -i dashboard.mp4 -y -filter_complex "[0:v]scale=640:360[v0];[1:v]scale=640:360[v1];[v0][v1]vstack=inputs=2[v]" -map "[v]" result.mp4

  
kill $rqt_pid