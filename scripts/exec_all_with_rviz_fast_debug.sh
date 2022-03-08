roslaunch hma_hsr_cu_pkg cu_debug.launch time:=1.0 open_drawer:=false
wait
sleep 5
roslaunch hma_hsr_gg_pkg gg_debug.launch