<div align="center">
  <img src="resources/logo.png" />
  <div align="center">
    <!-- <b><font size="6">Hibikino-Musashi@Home</font></b><br> -->
    <a href="https://www.brain.kyutech.ac.jp/~hma/">
      <b><font size="5">&#x1f5a5; Website</font></b>
    </a>
    &nbsp;|&nbsp;
    <a href="https://hibikino-musashi-home.github.io/hma_wrs_sim_ws/">
      <b><font size="5">&#x1f4d6; Documentation</font></b><br>
    </a>
  </div><br>
  <img src="resources/mapping.png" width="500"/>&nbsp;&nbsp;
  <img src="resources/yolact.gif" width="234"/>
</div>
<br>

# Introduction
This repository is the development workspace for the [WRS Simulator](https://github.com/hsr-project/tmc_wrs_docker) at Hibikino-Musashi@Home.

It contains the source code that won 2nd place at [RoboCup 2021 Worldwide](https://athome.robocup.org/rc2021/) and 1st place at [RoboCup Asia-Pacific 2021](https://robocupap.org/robocuphome/).

It also contains a part of the 1st place source code at the [World Robot Summit 2020 Partner Robot Challenge (Real Space)](https://wrs.nedo.go.jp/wrs2020/challenge/service/partner.html).

## Test environment
This workspace was tested on Ubuntu 18.04, ROS Melodic, RTX 3090, CUDA 11.2, and cuDNN 8.

## Execution video
[![](https://img.youtube.com/vi/DRKtSZEgaNQ/0.jpg)](https://www.youtube.com/watch?v=DRKtSZEgaNQ)

# Installation
Please perform the docker installation described in the [WRS Simulator](https://github.com/hsr-project/tmc_wrs_docker) in advance.

## Clone repository
```
$ mkdir -p ~/ros_ws/hma && cd ~/ros_ws/hma
$ git clone --recursive https://github.com/Hibikino-Musashi-Home/hma_wrs_sim_ws.git
```

## Download weights for object recognition 
```
$ cd ~/ros_ws/hma/hma_wrs_sim_ws
$ sh get-weights.sh
```

# Usage
## 1. Starting the simulator
### i. with GPU
```
$ DISPLAY=:0 xhost si:localuser:root
$ cd ~/ros_ws/hma/hma_wrs_sim_ws/src/04_sim_docker/hsrb_robocup_dspl_docker
$ docker-compose -f docker-compose.nvidia.yml up
```

### ii. without GPU
```
$ DISPLAY=:0 xhost si:localuser:root
$ cd ~/ros_ws/hma/hma_wrs_sim_ws/src/04_sim_docker/hsrb_robocup_dspl_docker
$ docker-compose -f docker-compose.yml up
```

After starting the simulator, access to [http://localhost:3000/](http://localhost:3000/) and press the Start button (▷) on the simulator.

## 2. Executing the program with docker
Start a new terminal and execute the following commands.
```
$ cd ~/ros_ws/hma/hma_wrs_sim_ws/src/04_sim_docker/hsrb_robocup_dspl_docker/ && source set-rosmaster.sh
$ cd ~/ros_ws/hma/hma_wrs_sim_ws
$ docker build . -t hma_wrs_sim_ws
$ sh docker-run.sh
$ source /opt/ros/melodic/setup.bash && cd ~/ros_ws/hma/hma_wrs_sim_ws && catkin build
$ source ~/ros_ws/hma/cv_bridge_ws/install/setup.bash --extend && source ~/ros_ws/hma/hma_wrs_sim_ws/devel/setup.bash
$ sh ~/ros_ws/hma/hma_wrs_sim_ws/scripts/exec_all.sh
```
### Notes
- Changing the source code placed in ~/ros_ws/hma/hma_wrs_sim_ws/src on the local computer also changes the source code in the docker container.

- If you exit from a docker container by typing ctrl + p, ctrl + q, you can re-enter the container with `$ docker attach [container id]`.
The container id can check using `$ docker ps`.

# License
This software is released under the BSD 3-Clause Clear License, see [LICENSE](https://github.com/Hibikino-Musashi-Home/hma_wrs_sim_ws/blob/master/LICENSE).