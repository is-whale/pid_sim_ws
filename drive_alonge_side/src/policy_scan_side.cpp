#include <ros/ros.h>
#include <vector>
#include <nav_msgs/OccupancyGrid.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Vector3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_broadcaster.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

ros::Publisher edge_pub;
MoveBaseClient* ac;  // 全局变量，用于保存 MoveBaseClient 对象指针

std::vector<std::pair<int, int>> bresenham_line(int x0, int y0, int x1, int y1) {
    std::vector<std::pair<int, int>> line_points;
    bool steep = abs(y1 - y0) > abs(x1 - x0);
    if (steep) {
        std::swap(x0, y0);
        std::swap(x1, y1);
    }
    if (x0 > x1) {
        std::swap(x0, x1);
        std::swap(y0, y1);
    }
    int dx = x1 - x0;
    int dy = abs(y1 - y0);
    int error = dx / 2;
    int ystep = (y0 < y1) ? 1 : -1;
    int y = y0;
    for (int x = x0; x <= x1; x++) {
        if (steep) {
            line_points.push_back(std::make_pair(y, x));
        } else {
            line_points.push_back(std::make_pair(x, y));
        }
        error -= dy;
        if (error < 0) {
            y += ystep;
            error += dx;
        }
    }
    return line_points;
}

void costmapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg)
{
    nav_msgs::OccupancyGrid edges;
    edges.header = msg->header;
    edges.info = msg->info;
    edges.data.resize(msg->data.size(), 0);
    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener(tfBuffer);
    double tolerance = 0.11, radius = 1.1; // 这两个写成动态参数

    std::vector<geometry_msgs::Point> edge_points;

    for (int y = 1; y < msg->info.height - 1; y++) {
        for (int x = 1; x < msg->info.width - 1; x++) {
            // Compute the index in the flattened grid
            int i = y*msg->info.width + x;

            // If this cell is occupied and has at least one neighboring cell that is free,
            // then it is an edge cell
            if (msg->data[i] > 0 &&
                (msg->data[i - 1] == 0 || msg->data[i + 1] == 0 ||
                 msg->data[i - msg->info.width] == 0 || msg->data[i + msg->info.width] == 0))
            {
                double global_x = msg->info.origin.position.x + x * msg->info.resolution;
                double global_y = msg->info.origin.position.y + y * msg->info.resolution;
                geometry_msgs::PointStamped point_in_costmap;
                point_in_costmap.header.frame_id = msg->header.frame_id;
                point_in_costmap.header.stamp = ros::Time(0);
                point_in_costmap.point.x = global_x;
                point_in_costmap.point.y = global_y;
                point_in_costmap.point.z = 0.0;
                try{
                    // The transform will become available at some point later than the timestamp of the point, so we need to wait for it
                    tfBuffer.canTransform("base_link", "map", ros::Time(0), ros::Duration(1.0));
                    geometry_msgs::PointStamped point_in_base_link;
                    tfBuffer.transform(point_in_costmap, point_in_base_link, "base_link");
                    // ROS_INFO("Point in 'base_link' frame: x=%f, y=%f, z=%f", point_in_base_link.point.x, point_in_base_link.point.y, point_in_base_link.point.z);
                    if (point_in_base_link.point.x > 0.5 && point_in_base_link.point.y <= 0.5) //筛选出机器人右前方的障碍物信息
                    {
                        edges.data[i] = 100;
                        geometry_msgs::Point pt;
                        pt.x = x;
                        pt.y = y;
                        edge_points.push_back(pt);

                        double sum_dx = 0.0;
                        double sum_dy = 0.0;
                        for (const auto& neighbor : edge_points) {
                            if (std::hypot(pt.x - neighbor.x, pt.y - neighbor.y) <= 1.0) {
                                sum_dx += neighbor.x - pt.x;
                                sum_dy += neighbor.y - pt.y;
                            }
                        }

                        // Compute the direction
                        double angle = atan2(sum_dy, sum_dx);
                        ROS_INFO_STREAM("angle: "<<angle);
                        tf2::Quaternion quaternion;
                        quaternion.setRPY(0, 0, -angle);

                        // Get the direction as a quaternion
                        geometry_msgs::Quaternion direction_quat;
                        tf2::convert(quaternion, direction_quat);

                        double distance_squared = point_in_base_link.point.x * point_in_base_link.point.x + point_in_base_link.point.y * point_in_base_link.point.y;
                        // ROS_INFO_STREAM(abs(distance_squared - radius * radius));
                        if (abs(distance_squared - radius * radius) < tolerance)
                        {
                            // transform base_link origin point to map
                            geometry_msgs::PointStamped origin_point_in_base_link;
                            origin_point_in_base_link.header.frame_id = msg->header.frame_id;
                            origin_point_in_base_link.header.stamp = ros::Time(0);
                            origin_point_in_base_link.point.x = 0.0;
                            origin_point_in_base_link.point.y = 0.0;
                            origin_point_in_base_link.point.z = 0.0;
                            tfBuffer.canTransform("map", "base_link", ros::Time(0), ros::Duration(1.0));
                            geometry_msgs::PointStamped origin_point_in_costmap;
                            tfBuffer.transform(origin_point_in_base_link, origin_point_in_costmap, "map");

                            // 计算base_link原点对应的 x，y index值
                            int origin_x = static_cast<int>((origin_point_in_costmap.point.x - msg->info.origin.position.x) / msg->info.resolution);
                            int origin_y = static_cast<int>((origin_point_in_costmap.point.y - msg->info.origin.position.y) / msg->info.resolution);

                            // 通过bresenham_line算法确定车与选取目标点之间连线上的点，根据连线上是否有占据，判断是否是跨墙点
                            std::vector<std::pair<int, int>> line_points = bresenham_line(origin_x, origin_y, x, y);
                            for (auto it = line_points.begin(); it != line_points.end(); ++it) {
                                ROS_INFO_STREAM("x: "<<it->first<<" , "<<"y: "<<it->second);
                                int index = it->second * msg->info.width + it->first;
                                if(msg->data[index] > 0){
                                    continue;
                                }
                                else{
                                    move_base_msgs::MoveBaseGoal goal;
                                    // Set up the frame parameters
                                    goal.target_pose.header.frame_id = "map";
                                    goal.target_pose.header.stamp = ros::Time::now();

                                    // Define a position and orientation for the robot to reach
                                    goal.target_pose.pose.position.x = global_x;
                                    goal.target_pose.pose.position.y = global_y;
                                    goal.target_pose.pose.position.z = 0.0;

                                    // // ToDo: 通过每个点的切线方向来给orientation赋值，但还需要考虑朝向问题
                                    // goal.target_pose.pose.orientation.x = 0.0;
                                    // goal.target_pose.pose.orientation.y = 0.0;
                                    // goal.target_pose.pose.orientation.z = 0.7155289882270001;
                                    // goal.target_pose.pose.orientation.w = 0.6985830423126842;

                                    // Set the orientation from the tangent direction
                                    goal.target_pose.pose.orientation = direction_quat;

                                    // Send the goal position and orientation for the robot to reach
                                    ROS_INFO("Sending goal");
                                    ac->sendGoal(goal);
                                }
                            }
                        }
                        
                    }
                }
                catch (tf2::TransformException &ex) {
                    ROS_WARN("%s",ex.what());
                }
                
            }
        }
    }

    edge_pub.publish(edges);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "edge_detection");

    ros::NodeHandle nh;
    // 创建 MoveBaseClient 对象
    ac = new MoveBaseClient("move_base", true);

    // wait for the action server to come up
    while(!ac->waitForServer(ros::Duration(5.0))){
        ROS_INFO("Waiting for the move_base action server to come up");
    }

    ros::Subscriber costmap_sub = nh.subscribe("move_base/local_costmap/costmap", 1, costmapCallback);
    edge_pub = nh.advertise<nav_msgs::OccupancyGrid>("edges", 1);

    ros::spin();
    delete ac;  // 释放 MoveBaseClient 对象

    return 0;
}
