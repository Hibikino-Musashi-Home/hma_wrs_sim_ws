# hma_hsr_cu_pkg

The package for the Go and Get It task (Task 2 in the WRS).

For details of the rules, please [see](https://cdn.robocup.org/athome/wp/2021/06/2021_rulebook.pdf).

## Quick start
Please start the WRS Simulator before inputting the following command.
```
$ roslaunch hma_hsr_gg_pkg gg.launch
```
For debug:
```
$ roslaunch hma_hsr_gg_pkg gg_debug.launch
```

## Nodes
### [sm_gg_node](docs/sm_gg_node.md)
A state machine for executing the task.

### [detect_drivable_area_server](docs/detect_drivable_area_server.md)
Detects the drivable area.
