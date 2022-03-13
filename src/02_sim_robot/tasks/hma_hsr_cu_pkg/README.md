# hma_hsr_cu_pkg

The package for the Clean Up task (Task 1 in the WRS).

For details of the rules, please [see](https://cdn.robocup.org/athome/wp/2021/06/2021_rulebook.pdf).

## Quick start
Please start the WRS Simulator before inputting the following command.
```
$ roslaunch hma_hsr_cu_pkg cu.launch
```
For debug:
```
$ roslaunch hma_hsr_cu_pkg cu_debug.launch
```

## Nodes
### [sm_cu_node](docs/sm_cu_node.md)
A state machine for executing the task.

### [estimate_orientation_node](docs/estimate_orientation_node.md)
Estimates cutlery and marker orientation.
