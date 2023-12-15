### CmdVelToVehicleCmd for ROS
这是一个用于ROS (Robot Operating System)的转换节点，它将geometry_msgs/Twist消息转换为autoware_msgs/VehicleCmd消息。

### 依赖项
- ROS (测试在ROS Melodic和Noetic上)
- geometry_msgs
- autoware_msgs
### 文件解释
- CmdVelToVehicleCmd.cpp: 这个文件包含一个CmdVelToVehicleCmd类，该类在ROS中作为一个节点运行，订阅/cmd_vel主题，用于获取Twist类型的命令速度。它还发布/vehicle_cmd主题，用于发布VehicleCmd类型的命令。在cmdVelCallback函数中，它接收Twist类型的速度命令，将其转换为VehicleCmd类型的命令，并发布到/vehicle_cmd主题。

```bash
class CmdVelToVehicleCmd {
public:
    CmdVelToVehicleCmd() {
        // 初始化订阅者，订阅/cmd_vel话题，消息类型为Twist
        sub = nh.subscribe("/cmd_vel", 1, &CmdVelToVehicleCmd::cmdVelCallback, this);

        // 初始化发布者，发布到/vehicle_cmd话题，消息类型为VehicleCmd
        pub = nh.advertise<autoware_msgs::VehicleCmd>("/vehicle_cmd", 1);
    }

    // ... (省略了部分代码)

    void cmdVelCallback(const geometry_msgs::Twist::ConstPtr& msg) {
        // 创建VehicleCmd消息
        autoware_msgs::VehicleCmd vehicle_cmd;

        // 创建ControlCommand消息
        autoware_msgs::ControlCommand ctrl_cmd;
        double speed = msg->linear.x;
        //if(msg->linear.x < 0){
        //speed = 0;
        //}
        // 将Twist消息转化为ControlCommand消息
        ctrl_cmd.linear_velocity = speed;
        // ctrl_cmd.linear_acceleration = msg->linear.y;
        ctrl_cmd.steering_angle = msg->angular.z;

        // 将ControlCommand消息设置为VehicleCmd消息的一部分
        vehicle_cmd.ctrl_cmd = ctrl_cmd;

        // 发布VehicleCmd消息
        pub.publish(vehicle_cmd);
    }
}; ... (省略了部分代码)
```
### 如何使用
1. 首先启动你的ROS核心。
```bash 
roscore
```
2. 编译并运行该节点。
```bash
rosrun <your_package> cmd_vel_to_vehicle_cmd_node
``` 
### 注意
- 此程序假定你已经有一个发布到/cmd_vel主题的节点，通常这是你的机器人的遥控器。此外，你需要确保你的参数设置正确，以适应你的机器人的特定环境和需求。
- 请注意，你需要将<your_package>替换为你的实际ROS包名。