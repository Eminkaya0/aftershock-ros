#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>

class PathFollower {
public:
    PathFollower() {
        ros::NodeHandle nh;
        path_sub_ = nh.subscribe("/planned_path", 10, &PathFollower::pathCallback, this);
        cmd_pub_ = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
    }

    void pathCallback(const nav_msgs::Path::ConstPtr& msg) {
        ROS_INFO("Path received with %zu waypoints", msg->poses.size());
    }

private:
    ros::Subscriber path_sub_;
    ros::Publisher cmd_pub_;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "path_follower_node");
    PathFollower follower;
    ros::spin();
    return 0;
}