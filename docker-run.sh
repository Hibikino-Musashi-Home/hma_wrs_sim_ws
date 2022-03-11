docker run --gpus all \
           -it \
           --net host \
           -e ROS_MASTER_URI=$ROS_MASTER_URI \
           -e ROS_IP=`hostname -I | cut -d' ' -f1` \
           -v ~/ros_ws/hma/hma_wrs_sim_ws/src:/home/ros_ws/hma/hma_wrs_sim_ws/src hma_wrs_sim_ws:latest \
           /bin/bash
