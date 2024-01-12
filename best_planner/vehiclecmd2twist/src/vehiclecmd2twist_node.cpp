#include <ros/ros.h>                      // ROS库
#include <geometry_msgs/Twist.h>          //Twist消息类型
#include <autoware_msgs/VehicleCmd.h>     //VehicleCmd消息类型
#include <autoware_msgs/ControlCommand.h> //ControlCommand消息类型

#define USE_STAMPED 0

/*
TODO: 兼容不同的输出类型，如：Twist,TwistStamped等
暂时选择Twist类型
*/

class CmdVelToVehicleCmd
{
public:
    CmdVelToVehicleCmd()
    {
        // 初始化订阅者，订阅/vehicle_cmd话题，消息类型为TwistStamped
        vehicle_cmd_sub = nh.subscribe("/vehicle_cmd", 1, &CmdVelToVehicleCmd::vehicle_cmd_vel_callback, this);

        // 初始化发布者，发布到/cmd_vel话题,消息类型为VehicleCmd
        if (USE_STAMPED)
        {
            // 初始化订阅者，订阅/cmd_vel话题，消息类型为Twist
            cmd_vel_pub = nh.advertise<geometry_msgs::TwistStamped>("/cmd_vel", 1);
        }
        else
        {
            cmd_vel_without_stamped_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
        }
    }

private:
    ros::NodeHandle nh;              // ROS节点句柄
    ros::Subscriber sub;             // 订阅者句柄
    ros::Subscriber vehicle_cmd_sub; ///< vehicle_cmd订阅者句柄
    ros::Publisher pub;              // 发布者句柄

    ros::Publisher cmd_vel_pub;                 ///< cmd_vel发布者句柄
    ros::Publisher cmd_vel_without_stamped_pub; ///< cmd_vel发布者句柄

    void vehicle_cmd_vel_callback(const autoware_msgs::VehicleCmd::ConstPtr &msg)
    {
        autoware_msgs::VehicleCmd vehicle_cmd_for_pub;
        geometry_msgs::TwistStamped cmd_vel_for_pub;
        geometry_msgs::TwistPtr test;
        geometry_msgs::Twist cmd_vel_without_stamped;
        ///< twist command
        cmd_vel_for_pub.twist = msg->twist_cmd.twist; ///< 速度赋值
        cmd_vel_for_pub.header = msg->header;

        // for gazebo
        ///< twist without stamped
        cmd_vel_without_stamped.linear = msg->twist_cmd.twist.linear;   ///< 速度赋值
        cmd_vel_without_stamped.angular = msg->twist_cmd.twist.angular; ///< 速度赋值
        cmd_vel_without_stamped_pub.publish(cmd_vel_without_stamped);
        // end

        ///< vehice command
        vehicle_cmd_for_pub.header = msg->header;
        vehicle_cmd_for_pub.ctrl_cmd = msg->ctrl_cmd;
        // cmd_vel_pub.publish(cmd_vel_for_pub);
    }

    // void cmdVelCallback(const geometry_msgs::Twist::ConstPtr &msg)
    // {
    //     // 创建VehicleCmd消息
    //     autoware_msgs::VehicleCmd vehicle_cmd;
    //     // 创建ControlCommand消息
    //     autoware_msgs::ControlCommand ctrl_cmd;
    //     double speed = msg->linear.x;
    //     // if(msg->linear.x < 0){
    //     // speed = 0;
    //     // }
    //     // 将Twist消息转化为ControlCommand消息
    //     ctrl_cmd.linear_velocity = speed;
    //     // ctrl_cmd.linear_acceleration = msg->linear.y;
    //     ctrl_cmd.steering_angle = msg->angular.z;
    //     // 将ControlCommand消息设置为VehicleCmd消息的一部分
    //     vehicle_cmd.ctrl_cmd = ctrl_cmd;
    //     // 发布VehicleCmd消息
    //     pub.publish(vehicle_cmd);
    // }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "vehiclecmd2twist_node"); // 初始化ROS节点
    CmdVelToVehicleCmd cmd_vel_to_vehicle_cmd;      // 创建CmdVelToVehicleCmd对象
    ros::spin();                                    // 主循环，处理ROS消息
    return 0;
}