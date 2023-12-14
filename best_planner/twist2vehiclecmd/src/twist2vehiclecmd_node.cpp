#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <autoware_msgs/VehicleCmd.h>
#include <autoware_msgs/ControlCommand.h>

class CmdVelToVehicleCmd {
public:
    CmdVelToVehicleCmd() {
        // 初始化订阅者，订阅/cmd_vel话题，消息类型为Twist
        sub = nh.subscribe("/cmd_vel", 1, &CmdVelToVehicleCmd::cmdVelCallback, this);

        // 初始化发布者，发布到/vehicle_cmd话题，消息类型为VehicleCmd
        pub = nh.advertise<autoware_msgs::VehicleCmd>("/vehicle_cmd", 1);
    }

private:
    ros::NodeHandle nh;
    ros::Subscriber sub;
    ros::Publisher pub;

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
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "cmd_vel_to_vehicle_cmd_node");
    CmdVelToVehicleCmd cmd_vel_to_vehicle_cmd;
    ros::spin();
    return 0;
}
