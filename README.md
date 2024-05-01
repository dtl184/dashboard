# dashboard

This repository contains source code necessary to run my dashboard tool. The tool takes a dashcam video, ROS bag file, and records the data playing back on a custom RQt dashboard. It then stitches the dashboard under the dashcam video.

Dependencies: ROS-noetic, Python 3 (I used 3.8.10), RQt (usually installed alongside ROS), FFmpeg

# How to use:

1. Clone this repository to your (Linux) machine and navigate to it
2. Run the following command:

    `bash dashboard.bash /path/to/your/dashcam_video.mp4 /path/to/your/bag_file.bag your_offset_in_seconds`

3. Wait for the command to finish, making sure not to change the size of the RQt plugin window.

Once the tool finishes working, the final result will be in the same directory as `result.mp4`. Important: move this file to another directory before running the tool again. It will automatically overwrite the result file each time it is run. 
