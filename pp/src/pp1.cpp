#include <ros/ros.h>
#include <algorithm>
#include <cassert>
#include <cmath>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>

#include <geometry_msgs/Twist.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseWithCovariance.h>
#include <geometry_msgs/TwistWithCovariance.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include "geometry_msgs/PoseStamped.h"

#include "../include/cpprobotics_types.h"
// #include "../include/cubic_spline.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "std_msgs/Int32.h"

#define PREVIEW_DIS 3 // 预瞄距离

// #define Ld 1.868  //轴距
#define Ld 0.65 // 轴距

using namespace std;
using namespace cpprobotics;

ros::Publisher purepersuit_;
ros::Publisher path_pub_;
ros::Publisher posepub_;
nav_msgs::Path path;
int sign = 1;

float carVelocity = 0;
float preview_dis = 0;
float k = 0.1;
int bback = 1;

// 计算四元数转换到欧拉角
std::array<float, 3> calQuaternionToEuler(const float x, const float y,
                                          const float z, const float w)
{
  std::array<float, 3> calRPY = {(0, 0, 0)};
  // roll = atan2(2(wx+yz),1-2(x*x+y*y))
  calRPY[0] = atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y));
  // pitch = arcsin(2(wy-zx))
  calRPY[1] = asin(2 * (w * y - z * x));
  // yaw = atan2(2(wx+yz),1-2(y*y+z*z))
  calRPY[2] = atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z));

  return calRPY;
}

cpprobotics::Vec_f r_x_;
cpprobotics::Vec_f r_y_;

int pointNum = 0; // 保存路径点的个数
int targetIndex = pointNum - 1;
/*方案二*/
vector<float> bestPoints_ = {0.0};

// 计算发送给模型车的转角
void poseCallback(const geometry_msgs::Pose &currentWaypoint)
{
  // 保存当前信息
  auto currentPositionX = currentWaypoint.position.x;
  auto currentPositionY = currentWaypoint.position.y;
  auto currentPositionZ = 0.0;

  auto currentQuaternionX = currentWaypoint.orientation.x;
  auto currentQuaternionY = currentWaypoint.orientation.y;
  auto currentQuaternionZ = currentWaypoint.orientation.z;
  auto currentQuaternionW = currentWaypoint.orientation.w;

  // 四元数转换
  std::array<float, 3> calRPY =
      calQuaternionToEuler(currentQuaternionX, currentQuaternionY,
                           currentQuaternionZ, currentQuaternionW);
  geometry_msgs::Twist zero_vel;
  zero_vel.linear.x = 0;
  zero_vel.angular.z = 0;

  // 无路径不输出
  if (r_x_.empty())
  {
    purepersuit_.publish(zero_vel);
    return;
  }

  /**************************************************************************************************/
  // 方案二:通过计算当前坐标和目标轨迹距离，找到一个最小距离的索引号
  int index;
  vector<float> bestPoints_;
  for (int i = 0; i < pointNum; i++)
  {
    // float lad = 0.0;
    float path_x = r_x_[i];
    float path_y = r_y_[i];
    // 遍历所有路径点和当前位置的距离，保存到数组中
    float lad = sqrt(pow(path_x - currentPositionX, 2) +
                     pow(path_y - currentPositionY, 2));

    bestPoints_.push_back(lad);
  }
  // 找到数组中最小距离
  auto smallest = min_element(bestPoints_.begin(), bestPoints_.end());
  // 找到最小距离的索引位置
  index = distance(bestPoints_.begin(), smallest);
  //  ROS_INFO("%d",index);
  int temp_index;
  for (int i = index; i < pointNum; i++)
  {
    // 遍历路径点和预瞄点的距离，从最小横向位置的索引开始
    float dis =
        sqrt(pow(r_y_[index] - r_y_[i], 2) + pow(r_x_[index] - r_x_[i], 2));
    //  ROS_INFO("%d",index);
    // 判断跟预瞄点的距离
    if (dis < preview_dis)
    {
      temp_index = i;
    }
    else
    {
      break;
    }
  }
  std::cout << "预瞄点index:" << temp_index << endl;
  std::cout << "预瞄距离:" << preview_dis << endl;
  index = temp_index;

  /**************************************************************************************************/
  // ROS_INFO("test1");
  // ROS_INFO("%d",index);
  // double r1 = r_y_[index] - currentPositionY;
  // ROS_INFO("test1.01");
  // double r2 = r_x_[index] - currentPositionX;
  // ROS_INFO("%f,%f",r1,r2);
  float alpha =
      atan2(r_y_[index] - currentPositionY, r_x_[index] - currentPositionX);

  alpha = alpha - calRPY[2];
  // ROS_INFO("test1.1");
  // 当前点和目标点的距离Id
  float dl = sqrt(pow(r_y_[index] - currentPositionY, 2) +
                  pow(r_x_[index] - currentPositionX, 2));
  // ROS_INFO("test1.2");
  // 发布小车运动指令及运动轨迹
  float rl = sqrt(pow(r_y_[pointNum - 1] - currentPositionY, 2) +
                  pow(r_x_[pointNum - 1] - currentPositionX, 2));
  ROS_INFO("DL: %f", dl);
  ROS_INFO("RL: %f", rl);
  if (rl > 0.2)
  {
    //   ROS_INFO("test2");
    float theta = atan(2 * Ld * sin(alpha) / dl);
    geometry_msgs::Twist vel_msg;
    // vel_msg.linear.x = sign*3;
    vel_msg.linear.x = sign * 0.5;
    vel_msg.angular.z = theta;
    purepersuit_.publish(vel_msg);
    // 发布小车运动轨迹
    geometry_msgs::PoseStamped this_pose_stamped;
    this_pose_stamped.pose.position.x = currentPositionX;
    this_pose_stamped.pose.position.y = currentPositionY;

    geometry_msgs::Quaternion goal_quat = tf::createQuaternionMsgFromYaw(theta);
    this_pose_stamped.pose.orientation.x = currentQuaternionX;
    this_pose_stamped.pose.orientation.y = currentQuaternionY;
    this_pose_stamped.pose.orientation.z = currentQuaternionZ;
    this_pose_stamped.pose.orientation.w = currentQuaternionW;

    this_pose_stamped.header.stamp = ros::Time::now();

    this_pose_stamped.header.frame_id = "world";
    // ROS_INFO("test3");
    path.poses.push_back(this_pose_stamped);
    // ROS_INFO("test4");
  }
  else
  {
    //   ROS_INFO("test1.3");
    geometry_msgs::Twist vel_msg;
    vel_msg.linear.x = 0;
    vel_msg.angular.z = 0;
    // ROS_INFO("test1.4");
    purepersuit_.publish(vel_msg);
    // ROS_INFO("test1.5");
  }
  path_pub_.publish(path);
}

void velocityCall(const geometry_msgs::Twist &carWaypoint)
{
  // carVelocity = carWaypoint.linear.x;
  carVelocity = carWaypoint.linear.x;
  preview_dis = k * carVelocity + PREVIEW_DIS;
}

void pointCallback(const nav_msgs::Path &msg)
{
  // geometry_msgs/PoseStamped[] poses
  //    ROS_INFO("got a plan!");
  int pn = msg.poses.size();
  if (pn != pointNum)
  {
    pointNum = pn;
    r_x_.clear();
    r_y_.clear();
    for (int i = 0; i < pointNum; i++)
    {
      r_x_.push_back(msg.poses[i].pose.position.x);
      r_y_.push_back(msg.poses[i].pose.position.y);
    }
    ROS_INFO("current path len: %d", pointNum);
    // auto a = msg.poses[0].pose.position.x;

    //  ROS_INFO("point %d:%f,%f",i,msg.poses[i].pose.position.x,msg.poses[i].pose.position.y);
  }
  else if (!msg.poses.empty() && !r_x_.empty())
  {
    if (r_x_[0] != msg.poses[0].pose.position.x)
    {
      pointNum = pn;
      r_x_.clear();
      r_y_.clear();
      for (int i = 0; i < pointNum; i++)
      {
        r_x_.push_back(msg.poses[i].pose.position.x);
        r_y_.push_back(msg.poses[i].pose.position.y);
      }
    }
  }
}

/**
 * @brief 获取车身姿态信息
  */
void odomCallback(const nav_msgs::Odometry &odominfo)
{
  static tf2_ros::Buffer buf;
  static tf2_ros::TransformListener tl(buf);
  // geometry_msgs::TransformStamped tfm = buf.lookupTransform("odom","map",ros::Time(0));
  geometry_msgs::Pose curr_pose = odominfo.pose.pose;
  geometry_msgs::Twist curr_vel = odominfo.twist.twist;
  geometry_msgs::PoseStamped currpose;
  geometry_msgs::TwistStamped currtwist;
  geometry_msgs::PoseStamped newpose;
  geometry_msgs::TwistStamped newtwist;
  currpose.header.frame_id = "odom";
  currpose.pose = curr_pose;
  currtwist.header.frame_id = "odom";
  currtwist.twist = curr_vel;

  try
  {
    newpose = buf.transform(currpose, "map");
    // newtwist = ;
    // ROS_INFO("TRANSFORM SUCC!");
    posepub_.publish(newpose);
  }
  catch (const std::exception &e)
  {
    ROS_INFO("error %s", e.what());
  }

  // newtwist = buf.transform(currtwist,"map");
  poseCallback(newpose.pose);
  // velocityCall(newtwist.twist);
  // poseCallback(curr_pose);
  velocityCall(curr_vel);
  // ROS_INFO("getting one odom info!");
}
void signCallback(const std_msgs::Int32 msg)
{
  sign = msg.data;
}
int main(int argc, char **argv)
{
  // 创建节点
  ros::init(argc, argv, "pure_pursuit");
  ROS_INFO("succ!");

  // 创建节点句柄
  ros::NodeHandle n;
  // tf

  // 创建Publisher，发送经过pure_pursuit计算后的转角及速度
  purepersuit_ = n.advertise<geometry_msgs::Twist>("/smart/cmd_vel2", 20);

  path_pub_ = n.advertise<nav_msgs::Path>("rvizpath", 100, true);
  // ros::Rate loop_rate(10);
  posepub_ = n.advertise<geometry_msgs::PoseStamped>("transpose", 10);
  path.header.frame_id = "map";
  // 设置时间戳
  path.header.stamp = ros::Time::now();
  geometry_msgs::PoseStamped pose;
  pose.header.stamp = ros::Time::now();
  // 设置参考系move_base/odom/footprint
  pose.header.frame_id = "odom";

  // ros::Subscriber splinePath = n.subscribe("/move_base/TebLocalPlannerROS/local_plan", 20, pointCallback);
  ros::Subscriber splinePath = n.subscribe("/stable_local", 20, pointCallback);

  ros::Subscriber odomMsgs = n.subscribe("/odom", 20, odomCallback);
  ros::Subscriber signn = n.subscribe("/sign", 20, signCallback);
  ros::spin();
  return 0;
}
