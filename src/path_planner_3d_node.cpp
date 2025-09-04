// DOSYA: path_planner_3d_node.cpp - İYİLEŞTİRİLMİŞ VERSİYON
#include <ros/ros.h>
#include <octomap_msgs/Octomap.h>
#include <octomap_msgs/conversions.h>
#include <octomap/octomap.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>
#include <tf2_ros/transform_listener.h>
#include <visualization_msgs/Marker.h>

// OMPL Kütüphaneleri
#include <ompl/base/SpaceInformation.h>
#include <ompl/base/spaces/SE3StateSpace.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>
#include <ompl/geometric/planners/rrt/RRTstar.h>
#include <ompl/geometric/planners/prm/PRMstar.h>  // YENİ: Daha optimal planlayıcı
#include <ompl/geometric/PathSimplifier.h>
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/base/objectives/PathLengthOptimizationObjective.h>  // YENİ: Optimizasyon hedefi

namespace ob = ompl::base;
namespace og = ompl::geometric;

class PathPlanner3D {
public:
    PathPlanner3D() : nh_("~"), tf_listener_(tf_buffer_) {
        // Parametreler
        nh_.param<std::string>("global_frame", global_frame_, "world");
        nh_.param<std::string>("robot_base_frame", robot_base_frame_, "firefly/base_link");
        nh_.param<double>("planning_timeout", planning_timeout_, 5.0);
        nh_.param<double>("robot_radius", robot_radius_, 0.6);
        nh_.param<double>("safety_margin", safety_margin_, 0.4);
        nh_.param<int>("interpolation_points", interpolation_points_, 200);
        nh_.param<double>("goal_tolerance", goal_tolerance_, 0.1);
        nh_.param<std::string>("planner_type", planner_type_, "RRTstar"); // Varsayılan olarak RRT* kullan
        
        // YENİ: Path pürüzsüzleştirme parametreleri
        nh_.param<double>("smoothing_time", smoothing_time_, 1.0);
        nh_.param<int>("shortcut_attempts", shortcut_attempts_, 50);
        nh_.param<double>("max_segment_length", max_segment_length_, 0.5);
        nh_.param<double>("collision_check_resolution", collision_check_resolution_, 0.05);
        
        // YENİ: Dinamik sınırları oku
        nh_.param<double>("bounds/x_min", x_min_, -25.0);
        nh_.param<double>("bounds/x_max", x_max_, 25.0);
        nh_.param<double>("bounds/y_min", y_min_, -25.0);
        nh_.param<double>("bounds/y_max", y_max_, 25.0);
        nh_.param<double>("bounds/z_min", z_min_, 0.5);
        nh_.param<double>("bounds/z_max", z_max_, 20.0);
        
        // Hedef takip değişkenlerini başlat
        has_previous_goal_ = false;

        // Subscribers & Publishers
        goal_sub_ = nh_.subscribe("/exploration_goal", 1, &PathPlanner3D::goalCallback, this);
        octomap_sub_ = nh_.subscribe("/octomap_full", 1, &PathPlanner3D::octomapCallback, this);
        path_pub_ = nh_.advertise<nav_msgs::Path>("/planned_path", 1, true);
        marker_pub_ = nh_.advertise<visualization_msgs::Marker>("/path_marker", 1, true);
        
        // YENİ: Debug marker publisher
        debug_marker_pub_ = nh_.advertise<visualization_msgs::Marker>("/path_debug_marker", 1, true);

        setupOMPL();
        ROS_INFO("3D Path Planner Node başlatıldı (İyileştirilmiş Versiyon)");
    }

private:
    ros::NodeHandle nh_;
    ros::Subscriber goal_sub_, octomap_sub_;
    ros::Publisher path_pub_, marker_pub_, debug_marker_pub_;
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;

    std::shared_ptr<octomap::OcTree> octree_;

    std::string global_frame_, robot_base_frame_;
    double planning_timeout_;
    double robot_radius_;
    double safety_margin_;
    int interpolation_points_;
    
    // YENİ parametreler
    double smoothing_time_;
    int shortcut_attempts_;
    double max_segment_length_;
    double collision_check_resolution_;
    double x_min_, x_max_, y_min_, y_max_, z_min_, z_max_;
    
    geometry_msgs::PoseStamped last_goal_;
    bool has_previous_goal_;
    double goal_tolerance_;
    std::string planner_type_;

    ob::SpaceInformationPtr space_information_;

    void octomapCallback(const octomap_msgs::Octomap::ConstPtr& msg) {
        octomap::AbstractOcTree* tree;
        if (msg->binary) {
            tree = octomap_msgs::binaryMsgToMap(*msg);
        } else {
            tree = octomap_msgs::fullMsgToMap(*msg);
        }
        
        if (tree) {
            octree_ = std::shared_ptr<octomap::OcTree>(dynamic_cast<octomap::OcTree*>(tree));
            ROS_INFO("OctoMap başarıyla alındı. Leaf sayısı: %zu", octree_->getNumLeafNodes());
        } else {
            ROS_ERROR("OctoMap mesaji dönüştürülemedi");
        }
    }

    // İYİLEŞTİRİLMİŞ: Daha hassas çarpışma kontrolü
    bool isStateValid(const ob::State *state) {
        if (!octree_) {
            ROS_DEBUG("isStateValid: Octree bulunamadı");
            return false;
        }

        const auto *se3_state = state->as<ob::SE3StateSpace::StateType>();
        const double x = se3_state->getX();
        const double y = se3_state->getY();
        const double z = se3_state->getZ();
        
        // Toplam kontrol yarıçapı
        double total_radius = robot_radius_ + safety_margin_;
        
        // İYİLEŞTİRİLMİŞ: Daha fazla kontrol noktası (küresel dağılım)
        std::vector<std::array<double, 3>> check_points = {
            {0, 0, 0},  // merkez
            // X-Y-Z eksenleri
            {total_radius, 0, 0}, {-total_radius, 0, 0},
            {0, total_radius, 0}, {0, -total_radius, 0},
            {0, 0, total_radius}, {0, 0, -total_radius},
            // Köşegenler (daha iyi kapsama için)
            {total_radius*0.7, total_radius*0.7, 0},
            {-total_radius*0.7, total_radius*0.7, 0},
            {total_radius*0.7, -total_radius*0.7, 0},
            {-total_radius*0.7, -total_radius*0.7, 0},
            // Üst ve alt köşegenler
            {total_radius*0.5, 0, total_radius*0.5},
            {-total_radius*0.5, 0, total_radius*0.5},
            {0, total_radius*0.5, total_radius*0.5},
            {0, -total_radius*0.5, total_radius*0.5}
        };
        
        for (const auto& offset : check_points) {
            octomap::point3d query_point(x + offset[0], y + offset[1], z + offset[2]);
            
            // Harita sınırları kontrolü
            octomap::OcTreeKey key;
            if (!octree_->coordToKeyChecked(query_point, key)) {
                continue;  // Harita dışındaki noktaları atla
            }
            
            octomap::OcTreeNode* node = octree_->search(query_point);
            if (node && octree_->isNodeOccupied(node)) {
                ROS_DEBUG("isStateValid: Çarpışma tespit edildi [%.2f, %.2f, %.2f]", 
                         x + offset[0], y + offset[1], z + offset[2]);
                return false;
            }
        }
        
        return true;
    }

    void setupOMPL() {
        // 1. State Space tanımlama
        auto space = std::make_shared<ob::SE3StateSpace>();

        // 2. Dinamik sınırları kullan
        ob::RealVectorBounds bounds(3);
        bounds.setLow(0, x_min_); bounds.setHigh(0, x_max_);
        bounds.setLow(1, y_min_); bounds.setHigh(1, y_max_);
        bounds.setLow(2, z_min_); bounds.setHigh(2, z_max_);
        space->setBounds(bounds);

        // 3. Space Information kurulumu
        space_information_ = std::make_shared<ob::SpaceInformation>(space);
        space_information_->setStateValidityChecker([this](const ob::State *state) { 
            return isStateValid(state); 
        });
        
        // İYİLEŞTİRİLMİŞ: Daha dengeli çarpışma kontrolü
        space_information_->setStateValidityCheckingResolution(collision_check_resolution_);
        
        space_information_->setup();
        
        ROS_INFO("OMPL kurulumu tamamlandı. Sınırlar: X[%.1f, %.1f] Y[%.1f, %.1f] Z[%.1f, %.1f]",
                 x_min_, x_max_, y_min_, y_max_, z_min_, z_max_);
    }

    void goalCallback(const geometry_msgs::PoseStamped::ConstPtr& goal_msg) {
        // Hedef değişiklik kontrolü
        if (has_previous_goal_) {
            double dx = goal_msg->pose.position.x - last_goal_.pose.position.x;
            double dy = goal_msg->pose.position.y - last_goal_.pose.position.y;
            double dz = goal_msg->pose.position.z - last_goal_.pose.position.z;
            double distance = sqrt(dx*dx + dy*dy + dz*dz);
            
            if (distance < goal_tolerance_) {
                ROS_DEBUG("Hedef değişmedi, planlama atlandı");
                return;
            }
        }

        ROS_INFO("Yeni hedef alındı: [%.2f, %.2f, %.2f]", 
                 goal_msg->pose.position.x, goal_msg->pose.position.y, goal_msg->pose.position.z);
        
        if (!octree_ || !space_information_) {
            ROS_WARN("Harita veya OMPL kurulumu hazır değil");
            return;
        }

        // Robot'un mevcut konumunu al
        geometry_msgs::TransformStamped transform_stamped;
        try {
            transform_stamped = tf_buffer_.lookupTransform(global_frame_, robot_base_frame_, 
                                                          ros::Time(0), ros::Duration(1.0));
        } catch (tf2::TransformException &ex) {
            ROS_WARN("Başlangıç konumu alınamadı: %s", ex.what());
            return;
        }

        // Başlangıç ve hedef durumlarını tanımla
        ob::ScopedState<> start(space_information_);
        start->as<ob::SE3StateSpace::StateType>()->setXYZ(
            transform_stamped.transform.translation.x,
            transform_stamped.transform.translation.y,
            transform_stamped.transform.translation.z);
        start->as<ob::SE3StateSpace::StateType>()->rotation().setIdentity();

        ob::ScopedState<> goal(space_information_);
        goal->as<ob::SE3StateSpace::StateType>()->setXYZ(
            goal_msg->pose.position.x,
            goal_msg->pose.position.y,
            goal_msg->pose.position.z);
        goal->as<ob::SE3StateSpace::StateType>()->rotation().setIdentity();

        // Durum kontrolü
        bool start_valid = space_information_->isValid(start.get());
        bool goal_valid = space_information_->isValid(goal.get());
        
        if (!start_valid || !goal_valid) {
            ROS_ERROR("Başlangıç veya hedef konumu geçersiz!");
            return;
        }

        // İYİLEŞTİRİLMİŞ: Simple Setup ve optimizasyon hedefi
        auto ss = std::make_shared<og::SimpleSetup>(space_information_);
        ss->setStartAndGoalStates(start, goal);
        
        // YENİ: Optimizasyon hedefi ekle (daha kısa yollar için)
        auto objective = std::make_shared<ob::PathLengthOptimizationObjective>(space_information_);
        objective->setCostThreshold(ob::Cost(std::numeric_limits<double>::infinity()));
        ss->setOptimizationObjective(objective);
        
        // Planlayıcı seçimi
        if (planner_type_ == "RRTstar") {
            auto rrt_star = std::make_shared<og::RRTstar>(space_information_);
            rrt_star->setRange(2.0);  // YENİ: Daha büyük adımlar için
            ss->setPlanner(rrt_star);
            ROS_INFO("RRT* planlayıcısı kullanılıyor");
        } else if (planner_type_ == "PRMstar") {
            auto prm_star = std::make_shared<og::PRMstar>(space_information_);
            ss->setPlanner(prm_star);
            ROS_INFO("PRM* planlayıcısı kullanılıyor");
        } else {
            auto rrt_connect = std::make_shared<og::RRTConnect>(space_information_);
            rrt_connect->setRange(2.0);  // YENİ: Daha büyük adımlar
            ss->setPlanner(rrt_connect);
            ROS_INFO("RRTConnect planlayıcısı kullanılıyor");
        }

        // Planlama
        ROS_INFO("3D yol planlaması başlatıldı...");
        ob::PlannerStatus solved = ss->solve(planning_timeout_);

        if (solved) {
            ROS_INFO("Ham yol bulundu! Şimdi optimize ediliyor...");
            
            og::PathGeometric path = ss->getSolutionPath();
            
            // İYİLEŞTİRİLMİŞ: Gelişmiş path optimizasyonu
            optimizePath(path);
            
            publishPath(path);
            
            // Başarılı planlama sonrası hedefi kaydet
            last_goal_ = *goal_msg;
            has_previous_goal_ = true;
            
            ROS_INFO("Path optimizasyonu tamamlandı. Nokta sayısı: %zu", 
                     path.getStateCount());
        } else {
            ROS_WARN("Belirtilen sürede yol bulunamadı");
        }
    }
    
    // YENİ: Gelişmiş path optimizasyon fonksiyonu
    void optimizePath(og::PathGeometric& path) {
        auto simplifier = std::make_shared<og::PathSimplifier>(space_information_);
        
        // 1. Önce kısayollar bul (gereksiz noktaları kaldır)
        ROS_INFO("Kısayollar aranıyor...");
        simplifier->shortcutPath(path, shortcut_attempts_, 0, 0.33, 0.01);
        
        // 2. Path'i basitleştir
        ROS_INFO("Path basitleştiriliyor...");
        simplifier->simplify(path, smoothing_time_);
        
        // 3. B-Spline ile pürüzsüzleştir (2 kez uygula daha iyi sonuç için)
        ROS_INFO("B-Spline pürüzsüzleştirme uygulanıyor...");
        simplifier->smoothBSpline(path, 3);  // 3 iterasyon
        
        // 4. Tekrar kısayol araması (pürüzsüzleştirme sonrası)
        simplifier->shortcutPath(path, shortcut_attempts_/2, 0, 0.33, 0.01);
        
        // 5. Son kez pürüzsüzleştir
        simplifier->smoothBSpline(path, 2);
        
        // 6. İnterpolasyon (yumuşak geçişler için)
        path.interpolate(interpolation_points_);
        
        // 7. YENİ: Maksimum segment uzunluğuna göre böl
        subdividePathSegments(path);
    }
    
    // YENİ: Path segmentlerini maksimum uzunluğa göre böl
    void subdividePathSegments(og::PathGeometric& path) {
        og::PathGeometric new_path(space_information_);
        
        for (size_t i = 0; i < path.getStateCount() - 1; ++i) {
            const auto* s1 = path.getState(i)->as<ob::SE3StateSpace::StateType>();
            const auto* s2 = path.getState(i+1)->as<ob::SE3StateSpace::StateType>();
            
            double dx = s2->getX() - s1->getX();
            double dy = s2->getY() - s1->getY();
            double dz = s2->getZ() - s1->getZ();
            double dist = sqrt(dx*dx + dy*dy + dz*dz);
            
            new_path.append(path.getState(i));
            
            // Segment çok uzunsa ara noktalar ekle
            if (dist > max_segment_length_) {
                int n_segments = ceil(dist / max_segment_length_);
                for (int j = 1; j < n_segments; ++j) {
                    double t = double(j) / n_segments;
                    
                    ob::ScopedState<> inter_state(space_information_);
                    auto* inter = inter_state->as<ob::SE3StateSpace::StateType>();
                    inter->setXYZ(
                        s1->getX() + t * dx,
                        s1->getY() + t * dy,
                        s1->getZ() + t * dz
                    );
                    inter->rotation().setIdentity();
                    
                    if (space_information_->isValid(inter_state.get())) {
                        new_path.append(inter_state.get());
                    }
                }
            }
        }
        
        // Son noktayı ekle
        new_path.append(path.getState(path.getStateCount() - 1));
        
        path = new_path;
    }

    void publishPath(const og::PathGeometric& path) {
        nav_msgs::Path ros_path;
        ros_path.header.stamp = ros::Time::now();
        ros_path.header.frame_id = global_frame_;
        
        visualization_msgs::Marker path_marker;
        path_marker.header = ros_path.header;
        path_marker.ns = "planned_path";
        path_marker.id = 0;
        path_marker.type = visualization_msgs::Marker::LINE_STRIP;
        path_marker.action = visualization_msgs::Marker::ADD;
        path_marker.pose.orientation.w = 1.0;
        path_marker.scale.x = 0.05;  // Daha ince çizgi
        path_marker.color.a = 1.0;
        path_marker.color.r = 0.0;
        path_marker.color.g = 0.8;
        path_marker.color.b = 0.2;
        
        // YENİ: Debug marker (waypoint'leri göster)
        visualization_msgs::Marker waypoint_marker;
        waypoint_marker.header = ros_path.header;
        waypoint_marker.ns = "waypoints";
        waypoint_marker.id = 1;
        waypoint_marker.type = visualization_msgs::Marker::SPHERE_LIST;
        waypoint_marker.action = visualization_msgs::Marker::ADD;
        waypoint_marker.pose.orientation.w = 1.0;
        waypoint_marker.scale.x = waypoint_marker.scale.y = waypoint_marker.scale.z = 0.1;
        waypoint_marker.color.r = 1.0;
        waypoint_marker.color.g = 0.0;
        waypoint_marker.color.b = 0.0;
        waypoint_marker.color.a = 0.7;

        for (std::size_t i = 0; i < path.getStateCount(); ++i) {
            const auto* state = path.getState(i)->as<ob::SE3StateSpace::StateType>();
            double x = state->getX(), y = state->getY(), z = state->getZ();
            
            geometry_msgs::PoseStamped pose;
            pose.header = ros_path.header;
            pose.pose.position.x = x;
            pose.pose.position.y = y;
            pose.pose.position.z = z;
            pose.pose.orientation.w = 1.0;  // Rotasyon identity
            ros_path.poses.push_back(pose);
            
            geometry_msgs::Point p;
            p.x = x; p.y = y; p.z = z;
            path_marker.points.push_back(p);
            
            // Her 5. noktayı waypoint olarak işaretle
            if (i % 5 == 0) {
                waypoint_marker.points.push_back(p);
            }
        }

        path_pub_.publish(ros_path);
        marker_pub_.publish(path_marker);
        debug_marker_pub_.publish(waypoint_marker);
        
        ROS_INFO("Path yayınlandı: %zu nokta", ros_path.poses.size());
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "path_planner_3d_node");
    PathPlanner3D planner;
    ros::spin();
    return 0;
}