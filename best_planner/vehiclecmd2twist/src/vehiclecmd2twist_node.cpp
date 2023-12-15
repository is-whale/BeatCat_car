#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <autoware_msgs/VehicleCmd.h>
#include <autoware_msgs/ControlCommand.h>

class CmdVelToVehicleCmd
{
public:
    CmdVelToVehicleCmd()
    {

        // 初始化订阅者，订阅/cmd_vel话题，消息类型为Twist
        sub = nh.subscribe("/cmd_vel", 1, &CmdVelToVehicleCmd::cmdVelCallback, this);
        vehicle_cmd_sub = nh.subscribe("/vehicle_cmd", 1, &CmdVelToVehicleCmd::vehicle_cmd_vel_callback, this);

        // 初始化发布者，发布到/vehicle_cmd话题,消息类型为VehicleCmd

        pub = nh.advertise<autoware_msgs::VehicleCmd>("/vehicle_cmd", 1);
        cmd_vel_pub = nh.advertise<geometry_msgs::TwistStamped>("/cmd_vel_2",1);
    }

private:
    ros::NodeHandle nh;
    ros::Subscriber sub;
    ros::Subscriber vehicle_cmd_sub; ///< vehicle_cmd
    ros::Publisher pub;
    ros::Publisher cmd_vel_pub; ///< cmd_vel

    void vehicle_cmd_vel_callback(const autoware_msgs::VehicleCmd::ConstPtr &msg)
    {
        autoware_msgs::VehicleCmd vehicle_cmd_for_pub;
        geometry_msgs::TwistStamped cmd_vel_for_pub;
        ///< twist command
        cmd_vel_for_pub.twist = msg->twist_cmd.twist; ///< 速度赋值
        cmd_vel_for_pub.header = msg->header;

        ///< vehice command
        vehicle_cmd_for_pub.header = msg->header;
        vehicle_cmd_for_pub.ctrl_cmd = msg->ctrl_cmd;
        pub.publish(vehicle_cmd_for_pub);
        cmd_vel_pub.publish(cmd_vel_for_pub);
    }

    void cmdVelCallback(const geometry_msgs::Twist::ConstPtr &msg)
    {
        // 创建VehicleCmd消息
        autoware_msgs::VehicleCmd vehicle_cmd;

        // 创建ControlCommand消息
        autoware_msgs::ControlCommand ctrl_cmd;
        double speed = msg->linear.x;
        // if(msg->linear.x < 0){
        // speed = 0;
        // }
        //  将Twist消息转化为ControlCommand消息
        ctrl_cmd.linear_velocity = speed;
        // ctrl_cmd.linear_acceleration = msg->linear.y;
        ctrl_cmd.steering_angle = msg->angular.z;

        // 将ControlCommand消息设置为VehicleCmd消息的一部分
        vehicle_cmd.ctrl_cmd = ctrl_cmd;

        // 发布VehicleCmd消息
        pub.publish(vehicle_cmd);
    }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "vehiclecmd2twist_node");
    CmdVelToVehicleCmd cmd_vel_to_vehicle_cmd;
    ros::spin();
    return 0;
}
