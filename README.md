# BeatCat_car
---
Porting Planning and Control Algorithms Based on Autoware

从apollo ,autoware等开源项目中抽取自动驾驶框架的部分，实现低耦合的过程

#### set up
```
desc : waypoint_loader desc sample
cmd  : roslaunch waypoint_maker waypoint_loader.launch

desc : waypoint_saver desc sample
cmd  : roslaunch waypoint_maker waypoint_saver.launch

desc : waypoint_clicker desc sample
cmd  : rosrun waypoint_maker waypoint_clicker

desc : create waypoints from some source points
cmd  : roslaunch waypoint_maker waypoint_creator.launch
```


route planner
waypoint planner
path follow

autoware输出
vehicle_cmd_msg_t = autoware_msgs::VehicleCmd;

