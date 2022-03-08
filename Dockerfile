FROM ghcr.io/haganelego/hma_ros_docker/hma_ros:melodic-rc21

SHELL [ "/bin/bash", "-c" ]

ENV HOME /rcap21
ENV ROS_MASTER_URI http://simulator:11311

# install hsr package
# TODO

# create workspace folder
RUN mkdir -p $HOME/ros_ws/hma/hma_wrs_sim_ws
RUN mkdir -p $HOME/ros_ws/hma/cv_bridge_ws

# build cv_bridge, image_geometry
RUN source /opt/ros/melodic/setup.bash && \
    cd $HOME/ros_ws/hma/cv_bridge_ws && \
    mkdir install && \
    mkdir src && \
    catkin config -DPYTHON_EXECUTABLE=/usr/bin/python3 -DPYTHON_INCLUDE_DIR=/usr/include/python3.6m -DPYTHON_LIBRARY=/usr/lib/x86_64-linux-gnu/libpython3.6m.so && \
    catkin config --install && \
    git clone https://github.com/ros-perception/vision_opencv.git src/vision_opencv && \
    cd src/vision_opencv/ && \
    git checkout 1.13.0 && \
    cd ../../ && \
    catkin build cv_bridge && \
    catkin build image_geometry 

# copy algorithm
ADD . $HOME/ros_ws/hma/hma_wrs_sim_ws

# install dependencies defined in package.xml
RUN rosdep update

# compile and install our algorithm
RUN cd $HOME/ros_ws/hma/hma_wrs_sim_ws && source /opt/ros/$ROS_DISTRO/setup.bash && catkin build

# download yolact weight
RUN cd ~/ros_ws/hma/hma_wrs_sim_ws/src/01_common/hma_yolact_pkgs/hma_yolact_pkg/io/ && \
    wget TODO -O weights.zip && \
    unzip -o weights

CMD source $HOME/ros_ws/hma/catkin_workspace/install/setup.bash --extend && source $HOME/ros_ws/hma/hma_wrs_sim_ws/devel/setup.bash && sh ~/ros_ws/hma/hma_wrs_sim_ws/technical_challenge.sh

