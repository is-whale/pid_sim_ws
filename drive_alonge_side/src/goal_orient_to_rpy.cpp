#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/LinearMath/Matrix3x3.h>

void goalCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    // Extract the quaternion
    tf2::Quaternion q(
        msg->pose.orientation.x,
        msg->pose.orientation.y,
        msg->pose.orientation.z,
        msg->pose.orientation.w);

    // Convert to roll, pitch, yaw
    tf2::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);

    // Now you have the euler angles
    ROS_INFO("Euler Angles: Roll=%f, Pitch=%f, Yaw=%f", roll, pitch, yaw);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "quaternion_to_euler");

    ros::NodeHandle nh;

    ros::Subscriber goal_sub = nh.subscribe("/move_base_simple/goal", 1, goalCallback);

    ros::spin();

    return 0;
}
