<div align="center">
  <img src="resources/hma_logo.png" width="300"/>
  <div align="center">
    <a href="https://www.brain.kyutech.ac.jp/~hma/">
      <b><font size="5">Hibikino-Musashi@Home website</font></b>
    </a>
  </div><br><br>
  <img src="resources/mapping.png" width="500"/>&nbsp;&nbsp;
  <img src="resources/yolact.gif" width="234"/>
</div>
<br>

# Introduction
This repository is the development workspace for the [WRS Simulator](https://github.com/hsr-project/tmc_wrs_docker) at Hibikino-Musashi@Home.

It contains the source code that won 2nd place at [RoboCup 2021 Worldwide](https://athome.robocup.org/rc2021/) and 1st place at [RoboCup Asia-Pacific 2021](https://robocupap.org/robocuphome/).

It also contains a part of the 1st place source code at the [World Robot Summit 2020 Partner Robot Challenge (Real Space)](https://wrs.nedo.go.jp/wrs2020/challenge/service/partner.html).

## Execution video
<!-- TODO: 撮り直す -->
[![](https://img.youtube.com/vi/orbCnOmkeAo/0.jpg)](https://youtu.be/orbCnOmkeAo?t=4895)

# Installation
Please perform the docker installation described in the [WRS Simulator](https://github.com/hsr-project/tmc_wrs_docker) in advance.

## Clone repository and Build workspace
```
$ mkdir -p ~/ros_ws/hma && cd ~/ros_ws/hma
$ git clone --recursive https://github.com/Hibikino-Musashi-Home/hma_wrs_sim_ws.git
$ catkin build
```

## Download weights for object recognition 
```
$ cd ~/ros_ws/hma/hma_wrs_sim_ws
$ sh get_weights.sh
```

# Usage
## Terminal 1: Starting the simulator
```
cd ~/ros_ws/hma/hma_wrs_sim_ws/src/04_sim_docker/hsrb_robocup_dspl_docker
docker-compose -f docker-compose.nvidia.yml up
```

## Terminal 2: Executing the program in the local environment
```
$ cd ~/ros_ws/hma/hma_wrs_sim_ws
$ sh scripts/exec_all.sh
```

If you want to use docker, execute the following command.
## Terminal 2: Executing the program using docker
<!-- TODO: 動いてない -->
```
$ cd ~/ros_ws/hma/hma_wrs_sim_ws
$ docker build . -t hma_wrs_sim_ws
$ docker run --gpus all -it hma_wrs_sim_ws /bin/bash
```

# License
This software is released under the BSD 3-Clause Clear License, see [LICENSE](https://github.com/Hibikino-Musashi-Home/hma_wrs_sim_ws/blob/master/LICENSE).