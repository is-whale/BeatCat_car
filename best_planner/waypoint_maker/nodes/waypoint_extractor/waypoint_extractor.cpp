/*
 * Copyright 2015-2019 Autoware Foundation. All rights reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
#include <ros/ros.h>                 // 导入ROS头文件
#include <tf/transform_datatypes.h>  // 导入tf库的转换数据类型头文件
#include <autoware_msgs/LaneArray.h> // 导入autoware_msgs/LaneArray头文件
#include <iostream>                  // 导入标准输入输出流库
#include <fstream>                   // 导入文件流库
#include <vector>                    // 导入vector库

namespace waypoint_maker
{
  // 构造函数
  class WaypointExtractor
  {
  private:
    ros::NodeHandle nh_, private_nh_;   // 创建ros节点句柄私有成员变量
    ros::Subscriber larray_sub_;        // 创建订阅者私有成员变量
    std::string lane_csv_, lane_topic_; // 创建字符串私有成员变量
    autoware_msgs::LaneArray lane_;     // 创建lane数组私有成员变量

  public:
    WaypointExtractor() : private_nh_("~") // 构造函数初始化
    {
      init();
    }

    // 析构函数
    ~WaypointExtractor()
    {
      deinit();
    }

    // 公开函数，将速度从米/秒转换为千米/小时
    double mps2kmph(double velocity_mps)
    {
      return (velocity_mps * 60 * 60) / 1000;
    }

    // 公开函数，将文件路径添加后缀
    const std::string addFileSuffix(std::string file_path, std::string suffix)
    {
      std::string output_file_path, tmp;               // 创建字符串私有成员变量
      std::string directory_path, filename, extension; // 创建字符串私有成员变量

      tmp = file_path;
      const std::string::size_type idx_slash(tmp.find_last_of("/")); // 使用最后一个斜杠的下标
      if (idx_slash != std::string::npos)
      {
        tmp.erase(0, idx_slash); // 清除从0到斜杠下标的字符串
      }
      const std::string::size_type idx_dot(tmp.find_last_of("."));
      const std::string::size_type idx_dot_allpath(file_path.find_last_of("."));
      if (idx_dot != std::string::npos && idx_dot != tmp.size() - 1)
      {
        file_path.erase(idx_dot_allpath, file_path.size() - 1); // 清除从路径中最后一个点到最后一个字符串的长度的字符串
      }
      file_path += suffix + ".csv"; // 添加后缀和csv
      return file_path;
    }

    // 公开函数，初始化
    void init()
    {
      private_nh_.param<std::string>("lane_csv", lane_csv_, "/tmp/driving_lane.csv");     // 从参数服务器获取csv文件路径
      private_nh_.param<std::string>("lane_topic", lane_topic_, "/lane_waypoints_array"); // 从参数服务器获取主题名称
      // 设置发布者
      larray_sub_ = nh_.subscribe(lane_topic_, 1, &WaypointExtractor::LaneArrayCallback, this); // 创建订阅者
    }

    // 公开函数，去初始化
    void deinit()
    {
      if (lane_.lanes.empty())
      {
        return;
      }
      std::vector<std::string> dst_multi_file_path(lane_.lanes.size(), lane_csv_); // 创建字符串数组私有成员变量
      if (lane_.lanes.size() > 1)
      {
        for (auto &el : dst_multi_file_path)
        {
          el = addFileSuffix(el, std::to_string(&el - &dst_multi_file_path[0])); // 为文件路径添加索引后缀
        }
      }
      saveLaneArray(dst_multi_file_path, lane_); // 保存lane数组到csv文件
    }

    // 公开函数，处理lane数组回调函数
    void LaneArrayCallback(const autoware_msgs::LaneArray::ConstPtr &larray)
    {
      if (larray->lanes.empty())
      {
        return;
      }
      lane_ = *larray; // 转储lane数组
    }

    // 公开函数，保存lane数组到csv文件
    void saveLaneArray(const std::vector<std::string> &paths, const autoware_msgs::LaneArray &lane_array)
    {
      for (const auto &file_path : paths)
      {
        const unsigned long idx = &file_path - &paths[0];                                                   // 使用文件路径得到索引
        std::ofstream ofs(file_path.c_str());                                                               // 创建文件流对象
        ofs << "x,y,z,yaw,velocity,change_flag,steering_flag,accel_flag,stop_flag,event_flag" << std::endl; // 写入文件头
        for (const auto &el : lane_array.lanes[idx].waypoints)
        {
          const geometry_msgs::Point p = el.pose.pose.position;    // 获取位置
          const double yaw = tf::getYaw(el.pose.pose.orientation); // 获取航向角
          const double vel = mps2kmph(el.twist.twist.linear.x);    // 转换速度单位
          const int states[] =
              {
                  el.change_flag, el.wpstate.steering_state, el.wpstate.accel_state,
                  el.wpstate.stop_state, el.wpstate.event_state};             // 定义状态数组
          ofs << std::fixed << std::setprecision(4);                          // 设置输出浮点数格式
          ofs << p.x << "," << p.y << "," << p.z << "," << yaw << "," << vel; // 输出位置、航向角和速度
          for (int i = 0; i < 5; ofs << "," << states[i++])
          {
          }                 // 输出状态
          ofs << std::endl; // 输出换行符
        }
      }
    }
  };

} // namespace waypoint_maker

int main(int argc, char **argv)
{
  ros::init(argc, argv, "waypoint_extractor"); // 初始化ROS节点
  waypoint_maker::WaypointExtractor we;        // 创建waypoint_extractor对象
  ros::spin();                                 // ROS事件循环
  return 0;
}