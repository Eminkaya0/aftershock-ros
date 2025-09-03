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
/*
// DOSYA: path_follower_node.cpp

#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h> // Mevcut konumu almak için
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <cmath>

class PathFollower {
public:
    PathFollower() : nh_("~"), tf_listener_(tf_buffer_) {
        // Parametreler
        nh_.param<std::string>("global_frame", global_frame_, "world");
        nh_.param<std::string>("robot_base_frame", robot_base_frame_, "firefly/base_link");
        nh_.param<double>("lookahead_distance", lookahead_distance_, 1.0); // İleriye ne kadar bakacağı (metre)
        nh_.param<double>("linear_speed", linear_speed_, 0.8);      // Maksimum ileri hız (m/s)
        nh_.param<double>("goal_tolerance", goal_tolerance_, 0.3);      // Hedefe ne kadar yaklaşınca duracağı (metre)

        path_sub_ = nh_.subscribe("/planned_path", 1, &PathFollower::pathCallback, this);
        cmd_pub_ = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
        
        // Yolu takip etmek için düzenli bir döngü
        timer_ = nh_.createTimer(ros::Duration(0.1), &PathFollower::controlLoop, this);

        current_path_ = nullptr;
        target_waypoint_index_ = 0;
        ROS_INFO("Path Follower Node başlatıldı.");
    }

private:
    ros::NodeHandle nh_;
    ros::Subscriber path_sub_;
    ros::Publisher cmd_pub_;
    ros::Timer timer_;
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;
    
    nav_msgs::Path::ConstPtr current_path_;
    int target_waypoint_index_;
    
    // Parametreler
    std::string global_frame_, robot_base_frame_;
    double lookahead_distance_;
    double linear_speed_;
    double goal_tolerance_;

    void pathCallback(const nav_msgs::Path::ConstPtr& msg) {
        if (msg->poses.empty()) {
            ROS_WARN("Boş bir yol alındı, takip durduruluyor.");
            current_path_ = nullptr;
            return;
        }
        ROS_INFO("Yeni yol alındı. %zu waypoint içeriyor.", msg->poses.size());
        current_path_ = msg;
        target_waypoint_index_ = 0; // Takibe en baştan başla
    }

    void controlLoop(const ros::TimerEvent&) {
        if (!current_path_ || target_waypoint_index_ >= current_path_->poses.size()) {
            // Takip edilecek yol yoksa veya yol bittiyse dur.
            stopMotion();
            return;
        }

        // 1. Dronun mevcut konumunu al
        geometry_msgs::PoseStamped current_pose;
        try {
            geometry_msgs::TransformStamped transform = tf_buffer_.lookupTransform(
                global_frame_, robot_base_frame_, ros::Time(0), ros::Duration(0.2));
            current_pose.header.frame_id = global_frame_;
            current_pose.pose.position.x = transform.transform.translation.x;
            current_pose.pose.position.y = transform.transform.translation.y;
            current_pose.pose.position.z = transform.transform.translation.z;
        } catch (tf2::TransformException &ex) {
            ROS_WARN("Konum alınamadı: %s", ex.what());
            stopMotion();
            return;
        }

        // 2. Hedef waypoint'i bul ve güncelle
        geometry_msgs::PoseStamped& target_waypoint = current_path_->poses[target_waypoint_index_];
        double distance_to_target = std::sqrt(
            std::pow(target_waypoint.pose.position.x - current_pose.pose.position.x, 2) +
            std::pow(target_waypoint.pose.position.y - current_pose.pose.position.y, 2) +
            std::pow(target_waypoint.pose.position.z - current_pose.pose.position.z, 2));

        // Hedefe yeterince yaklaştıysak, bir sonrakine geç
        if (distance_to_target < lookahead_distance_) {
            target_waypoint_index_++;
            if (target_waypoint_index_ >= current_path_->poses.size()) {
                ROS_INFO("Yol tamamlandı!");
                stopMotion();
                current_path_ = nullptr; // Yolu temizle
                return;
            }
        }
        
        // 3. Hız komutunu hesapla
        geometry_msgs::Twist cmd;
        double dx = target_waypoint.pose.position.x - current_pose.pose.position.x;
        double dy = target_waypoint.pose.position.y - current_pose.pose.position.y;
        double dz = target_waypoint.pose.position.z - current_pose.pose.position.z;

        // Yön vektörünü normalize et ve hızı uygula
        double magnitude = std::sqrt(dx*dx + dy*dy + dz*dz);
        if (magnitude > 0.01) {
            cmd.linear.x = (dx / magnitude) * linear_speed_;
            cmd.linear.y = (dy / magnitude) * linear_speed_;
            cmd.linear.z = (dz / magnitude) * linear_speed_;
        }
        
        // Z eksenindeki dönüşü (yaw) de ayarlamak daha gelişmiş bir kontrolcü gerektirir.
        // Şimdilik sadece pozisyonu takip ediyoruz.
        cmd.angular.z = 0; // Veya bir sonraki waypoint'e bakacak şekilde ayarla

        // 4. Komutu yayınla
        cmd_pub_.publish(cmd);
    }
    
    void stopMotion() {
        geometry_msgs::Twist stop_cmd;
        // Her şeyi sıfırla
        stop_cmd.linear.x = 0;
        stop_cmd.linear.y = 0;
        stop_cmd.linear.z = 0;
        stop_cmd.angular.x = 0;
        stop_cmd.angular.y = 0;
        stop_cmd.angular.z = 0;
        cmd_pub_.publish(stop_cmd);
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "path_follower_node");
    PathFollower follower;
    ros::spin();
    return 0;
}
*/