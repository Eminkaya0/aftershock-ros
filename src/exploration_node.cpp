#include <ros/ros.h>
#include <octomap_msgs/Octomap.h>
#include <octomap_msgs/conversions.h>
#include <octomap/octomap.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Point.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <visualization_msgs/MarkerArray.h>
#include <std_msgs/ColorRGBA.h>
#include <vector>
#include <cmath>
#include <algorithm>
#include <queue>
#include <map>
#include <set>

// Point3d karşılaştırıcı
struct Point3dComparator {
    bool operator()(const octomap::point3d& a, const octomap::point3d& b) const {
        if (a.x() != b.x()) return a.x() < b.x();
        if (a.y() != b.y()) return a.y() < b.y();
        return a.z() < b.z();
    }
};

// Frontier kümesi için bilgi yapısı
struct FrontierCluster {
    std::vector<octomap::point3d> points;
    octomap::point3d centroid;
    double exploration_value;  // Keşif değeri (ne kadar değerli bir hedef)
    double size;               // Küme büyüklüğü
    bool is_corner;            // Köşe mi?
    double explored_ratio;     // Çevresindeki keşfedilmiş alan oranı
};

class ExplorationNode {
public:
    ExplorationNode() : nh_("~"), tf_listener_(tf_buffer_) {
        // Sınır parametrelerini al
        nh_.param<double>("min_x", min_x_, -11.5);
        nh_.param<double>("max_x", max_x_, -1.0);
        nh_.param<double>("min_y", min_y_, -5.0);
        nh_.param<double>("max_y", max_y_, 5.0);
        nh_.param<double>("min_z", min_z_, 0.0);
        nh_.param<double>("max_z", max_z_, 1.7);  // YENİ: 1.7 metre yükseklik sınırı
        
        // Exploration parametreleri
        nh_.param<std::string>("octomap_topic", octomap_topic_, "/octomap_full");
        nh_.param<std::string>("goal_topic", goal_topic_, "/exploration_goal");
        nh_.param<std::string>("global_frame", global_frame_, "world");
        nh_.param<std::string>("robot_base_frame", robot_base_frame_, "base_link");
        nh_.param<double>("update_frequency", update_frequency_, 0.5);
        nh_.param<int>("min_cluster_size", min_cluster_size_, 20);  // Artırıldı
        
        // YENİ parametreler
        nh_.param<double>("cluster_radius", cluster_radius_, 0.8);  // Kümeleme yarıçapı
        nh_.param<double>("min_exploration_value", min_exploration_value_, 0.1);  // Minimum keşif değeri (gevşetildi)
        nh_.param<double>("corner_penalty", corner_penalty_, 0.1);  // Köşe cezası (azaltıldı)
        nh_.param<double>("safe_distance_from_frontier", safe_distance_, 1.0);  // Frontier'dan güvenli mesafe
        nh_.param<double>("exploration_radius", exploration_radius_, 2.0);  // Keşfedilmişlik kontrol yarıçapı
        nh_.param<double>("min_unexplored_ratio", min_unexplored_ratio_, 0.2);  // Minimum keşfedilmemiş alan oranı (gevşetildi)
        
        // Son hedef takibi
        has_previous_goal_ = false;
        goal_change_threshold_ = 2.0;  // 2 metre değişiklik eşiği

        // Subscriber ve Publisher'ları başlat
        octomap_sub_ = nh_.subscribe(octomap_topic_, 1, &ExplorationNode::octomapCallback, this);
        goal_pub_ = nh_.advertise<geometry_msgs::PoseStamped>(goal_topic_, 1);
        marker_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("/exploration_markers", 10);
        frontier_marker_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("/frontier_clusters", 10);
        
        // Ana işlem döngüsü için Timer
        timer_ = nh_.createTimer(ros::Duration(1.0 / update_frequency_), &ExplorationNode::processMapCallback, this);

        ROS_INFO("İyileştirilmiş Apartment Exploration Node başlatıldı");
        ROS_INFO("Yükseklik sınırı: %.2f metre", max_z_);
        ROS_INFO("Minimum küme boyutu: %d", min_cluster_size_);
    }

private:
    // Üye değişkenler
    double min_x_, max_x_, min_y_, max_y_, min_z_, max_z_;
    double cluster_radius_;
    double min_exploration_value_;
    double corner_penalty_;
    double safe_distance_;
    double exploration_radius_;
    double min_unexplored_ratio_;
    double goal_change_threshold_;
    
    geometry_msgs::Point last_goal_;
    bool has_previous_goal_;
    
    ros::NodeHandle nh_;
    ros::Subscriber octomap_sub_;
    ros::Publisher goal_pub_, marker_pub_, frontier_marker_pub_;
    ros::Timer timer_;
    
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;
    
    std::unique_ptr<octomap::OcTree> octree_;
    bool map_received_ = false;
    
    std::string octomap_topic_, goal_topic_, global_frame_, robot_base_frame_;
    double update_frequency_;
    int min_cluster_size_;

    void octomapCallback(const octomap_msgs::Octomap::ConstPtr& msg) {
        octomap::AbstractOcTree* tree = octomap_msgs::msgToMap(*msg);
        if (tree) {
            octree_.reset(dynamic_cast<octomap::OcTree*>(tree));
            map_received_ = true;
            ROS_INFO_ONCE("İlk OctoMap haritası başarıyla alındı");
        } else {
            ROS_ERROR("OctoMap mesajı dönüştürülemedi");
        }
    }

    void processMapCallback(const ros::TimerEvent& event) {
        if (!map_received_ || !octree_) {
            ROS_WARN_THROTTLE(5, "Henüz harita alınmadı, işlem bekleniyor...");
            return;
        }

        // Robot konumunu al
        geometry_msgs::TransformStamped robot_transform;
        try {
            robot_transform = tf_buffer_.lookupTransform(global_frame_, robot_base_frame_, ros::Time(0));
        } catch (tf2::TransformException &ex) {
            ROS_WARN("Robot konumu alınamadı: %s", ex.what());
            return;
        }
        
        octomap::point3d robot_pos(robot_transform.transform.translation.x,
                                   robot_transform.transform.translation.y,
                                   robot_transform.transform.translation.z);

        // 1. Frontier hücrelerini bul
        std::vector<octomap::point3d> frontier_cells = findFrontierCells();
        if (frontier_cells.empty()) {
            ROS_INFO_THROTTLE(10, "Keşfedilecek yeni sınır hücresi bulunamadı");
            return;
        }
        
        // 2. Gelişmiş kümeleme yap
        std::vector<FrontierCluster> clusters = advancedClusterFrontiers(frontier_cells);
        
        // 3. Kümeleri değerlendir ve filtrele
        evaluateAndFilterClusters(clusters, robot_pos);
        
        // 4. En iyi hedefi seç ve yayınla
        if (!clusters.empty()) {
            selectAndPublishBestGoal(clusters, robot_pos);
        } else {
            ROS_INFO("Uygun hedef kümesi bulunamadı");
        }
        
        // 5. Görselleştirmeleri yayınla
        visualizeClusters(clusters);
    }

    std::vector<octomap::point3d> findFrontierCells() {
        std::vector<octomap::point3d> frontiers;
        
        for (octomap::OcTree::leaf_iterator it = octree_->begin_leafs(), end = octree_->end_leafs(); 
             it != end; ++it) {
            
            // Sadece boş hücrelerle ilgilen
            if (octree_->isNodeOccupied(*it)) continue;
            
            octomap::point3d p = it.getCoordinate();
            
            // SINIR KONTROLÜ - Özellikle Z ekseni (1.7m sınırı)
            if (p.x() < min_x_ || p.x() > max_x_ ||
                p.y() < min_y_ || p.y() > max_y_ ||
                p.z() < min_z_ || p.z() > max_z_) {
                continue;
            }
            
            // Frontier olup olmadığını kontrol et
            if (isFrontierCell(p)) {
                frontiers.push_back(p);
            }
        }
        
        ROS_INFO_THROTTLE(5, "%zu adet frontier hücresi bulundu (z < %.2fm)", frontiers.size(), max_z_);
        return frontiers;
    }
    
    bool isFrontierCell(const octomap::point3d& point) {
        // 6-komşuluk kontrolü
        std::vector<octomap::point3d> neighbors = {
            octomap::point3d(point.x() + octree_->getResolution(), point.y(), point.z()),
            octomap::point3d(point.x() - octree_->getResolution(), point.y(), point.z()),
            octomap::point3d(point.x(), point.y() + octree_->getResolution(), point.z()),
            octomap::point3d(point.x(), point.y() - octree_->getResolution(), point.z()),
            octomap::point3d(point.x(), point.y(), point.z() + octree_->getResolution()),
            octomap::point3d(point.x(), point.y(), point.z() - octree_->getResolution())
        };
        
        for (const auto& neighbor : neighbors) {
            octomap::OcTreeNode* node = octree_->search(neighbor);
            if (node == nullptr) {  // Bilinmeyen alan
                return true;
            }
        }
        return false;
    }
    
    // YENİ: Gelişmiş kümeleme algoritması
    std::vector<FrontierCluster> advancedClusterFrontiers(const std::vector<octomap::point3d>& frontiers) {
        std::vector<FrontierCluster> clusters;
        std::set<octomap::point3d, Point3dComparator> unvisited(frontiers.begin(), frontiers.end());
        
        while (!unvisited.empty()) {
            FrontierCluster cluster;
            std::queue<octomap::point3d> queue;
            
            // İlk noktayı al
            auto it = unvisited.begin();
            queue.push(*it);
            unvisited.erase(it);
            
            // BFS ile kümeyi genişlet
            while (!queue.empty()) {
                octomap::point3d current = queue.front();
                queue.pop();
                cluster.points.push_back(current);
                
                // Yakındaki frontier'ları bul ve ekle
                auto nearby_it = unvisited.begin();
                while (nearby_it != unvisited.end()) {
                    if (current.distance(*nearby_it) < cluster_radius_) {
                        queue.push(*nearby_it);
                        nearby_it = unvisited.erase(nearby_it);
                    } else {
                        ++nearby_it;
                    }
                }
            }
            
            // Küme boyutu kontrolü
            if (cluster.points.size() >= min_cluster_size_) {
                calculateClusterProperties(cluster);
                clusters.push_back(cluster);
            }
        }
        
        ROS_INFO("%zu adet frontier kümesi oluşturuldu", clusters.size());
        return clusters;
    }
    
    void calculateClusterProperties(FrontierCluster& cluster) {
        // Centroid hesapla
        octomap::point3d sum(0, 0, 0);
        for (const auto& point : cluster.points) {
            sum += point;
        }
        cluster.centroid = sum * (1.0 / cluster.points.size());
        cluster.size = cluster.points.size();
        
        // Köşe kontrolü - etrafındaki dolu hücre dağılımına bak
        cluster.is_corner = isCornerCluster(cluster.centroid);
        
        // Keşfedilmişlik oranı hesapla
        cluster.explored_ratio = calculateExploredRatio(cluster.centroid);
    }
    
    bool isCornerCluster(const octomap::point3d& center) {
        // 8 yönde engel kontrolü yap
        int occupied_directions = 0;
        std::vector<double> angles = {0, 45, 90, 135, 180, 225, 270, 315};
        
        for (double angle : angles) {
            double rad = angle * M_PI / 180.0;
            octomap::point3d check_point(
                center.x() + 0.5 * cos(rad),
                center.y() + 0.5 * sin(rad),
                center.z()
            );
            
            octomap::OcTreeNode* node = octree_->search(check_point);
            if (node && octree_->isNodeOccupied(node)) {
                occupied_directions++;
            }
        }
        
        // 3'ten fazla yönde engel varsa köşe sayılır
        return occupied_directions >= 3;
    }
    
    double calculateExploredRatio(const octomap::point3d& center) {
        int total_cells = 0;
        int explored_cells = 0;
        
        // Belirli yarıçapta kontrol et (performans için adımları büyüttük)
        for (double x = -exploration_radius_; x <= exploration_radius_; x += 0.5) {
            for (double y = -exploration_radius_; y <= exploration_radius_; y += 0.5) {
                for (double z = -0.5; z <= 0.5; z += 0.5) {
                    octomap::point3d check_point(center.x() + x, center.y() + y, center.z() + z);
                    
                    // Sınırlar içinde mi?
                    if (check_point.x() < min_x_ || check_point.x() > max_x_ ||
                        check_point.y() < min_y_ || check_point.y() > max_y_ ||
                        check_point.z() < min_z_ || check_point.z() > max_z_) {
                        continue;
                    }
                    
                    total_cells++;
                    octomap::OcTreeNode* node = octree_->search(check_point);
                    if (node != nullptr) {  // Keşfedilmiş (boş veya dolu)
                        explored_cells++;
                    }
                }
            }
        }
        
        return (total_cells > 0) ? (double)explored_cells / total_cells : 0.0;
    }
    
    void evaluateAndFilterClusters(std::vector<FrontierCluster>& clusters, const octomap::point3d& robot_pos) {
        auto it = clusters.begin();
        while (it != clusters.end()) {
            // Keşif değeri hesapla
            double distance_factor = 1.0 / (1.0 + it->centroid.distance(robot_pos));
            double size_factor = std::min(1.0, it->size / 100.0);
            double unexplored_factor = 1.0 - it->explored_ratio;
            
            it->exploration_value = distance_factor * 0.3 + 
                                   size_factor * 0.3 + 
                                   unexplored_factor * 0.4;
            
            // Köşe cezası uygula
            if (it->is_corner) {
                it->exploration_value *= (1.0 - corner_penalty_);
            }
            
            // Filtreleme kriterleri
            bool should_remove = false;
            
            // 1. Çok fazla keşfedilmiş alan varsa
            if (it->explored_ratio > (1.0 - min_unexplored_ratio_)) {
                ROS_INFO("Küme kaldırıldı: Çok fazla keşfedilmiş (%.2f > %.2f)", it->explored_ratio, (1.0 - min_unexplored_ratio_));
                should_remove = true;
            }
            
            // 2. Köşede ve değeri düşükse
            if (it->is_corner && it->exploration_value < min_exploration_value_) {
                ROS_INFO("Küme kaldırıldı: Köşe ve düşük değer (%.3f < %.3f)", it->exploration_value, min_exploration_value_);
                should_remove = true;
            }
            
            // 3. Genel değer eşiği
            if (it->exploration_value < min_exploration_value_ * 0.5) {
                ROS_INFO("Küme kaldırıldı: Çok düşük keşif değeri (%.3f < %.3f)", it->exploration_value, min_exploration_value_ * 0.5);
                should_remove = true;
            }
            
            if (should_remove) {
                it = clusters.erase(it);
            } else {
                ++it;
            }
        }
        
        // Keşif değerine göre sırala (yüksekten düşüğe)
        std::sort(clusters.begin(), clusters.end(), 
                  [](const FrontierCluster& a, const FrontierCluster& b) {
                      return a.exploration_value > b.exploration_value;
                  });
        
        ROS_INFO("Filtreleme sonrası %zu adet küme kaldı", clusters.size());
    }
    
    void selectAndPublishBestGoal(const std::vector<FrontierCluster>& clusters, 
                                  const octomap::point3d& robot_pos) {
        // En iyi kümeyi seç (zaten sıralı)
        const FrontierCluster& best_cluster = clusters.front();
        
        // Güvenli hedef noktası bul
        octomap::point3d safe_goal = findSafeGoalPoint(best_cluster);
        
        // Aynı hedefe tekrar gitmeyi önle
        if (has_previous_goal_) {
            double distance = sqrt(pow(safe_goal.x() - last_goal_.x, 2) +
                                 pow(safe_goal.y() - last_goal_.y, 2) +
                                 pow(safe_goal.z() - last_goal_.z, 2));
            
            if (distance < goal_change_threshold_) {
                ROS_INFO("Hedef değişmedi, aynı hedef korunuyor");
                return;
            }
        }
        
        // Hedefi yayınla
        geometry_msgs::PoseStamped goal_pose;
        goal_pose.header.stamp = ros::Time::now();
        goal_pose.header.frame_id = global_frame_;
        goal_pose.pose.position.x = safe_goal.x();
        goal_pose.pose.position.y = safe_goal.y();
        goal_pose.pose.position.z = safe_goal.z();
        goal_pose.pose.orientation.w = 1.0;
        
        goal_pub_.publish(goal_pose);
        
        // Son hedefi kaydet
        last_goal_.x = safe_goal.x();
        last_goal_.y = safe_goal.y();
        last_goal_.z = safe_goal.z();
        has_previous_goal_ = true;
        
        ROS_INFO("Hedef yayınlandı: [%.2f, %.2f, %.2f] (Değer: %.3f, Köşe: %s, Keşfedilmiş: %.1f%%)",
                 safe_goal.x(), safe_goal.y(), safe_goal.z(),
                 best_cluster.exploration_value,
                 best_cluster.is_corner ? "EVET" : "HAYIR",
                 best_cluster.explored_ratio * 100);
    }
    
    octomap::point3d findSafeGoalPoint(const FrontierCluster& cluster) {
        // Cluster centroid'inden güvenli mesafede nokta bul
        octomap::point3d best_point = cluster.centroid;
        double best_score = -1.0;
        
        // Farklı mesafelerde dene
        for (double dist = safe_distance_; dist <= safe_distance_ + 1.0; dist += 0.2) {
            // 8 yönde ara
            for (int angle = 0; angle < 360; angle += 30) {
                double rad = angle * M_PI / 180.0;
                octomap::point3d candidate(
                    cluster.centroid.x() + dist * cos(rad),
                    cluster.centroid.y() + dist * sin(rad),
                    cluster.centroid.z()
                );
                
                // Güvenlik kontrolü
                if (!isPointSafe(candidate)) continue;
                
                // Skorlama: boş alan miktarı + frontier'a yakınlık
                double score = evaluateSafePoint(candidate, cluster.centroid);
                if (score > best_score) {
                    best_score = score;
                    best_point = candidate;
                }
            }
        }
        
        return best_point;
    }
    
    bool isPointSafe(const octomap::point3d& point) {
        // Nokta etrafında yeterli boş alan var mı?
        double check_radius = 0.5;
        for (double dx = -check_radius; dx <= check_radius; dx += 0.1) {
            for (double dy = -check_radius; dy <= check_radius; dy += 0.1) {
                for (double dz = -0.2; dz <= 0.2; dz += 0.1) {
                    octomap::point3d check(point.x() + dx, point.y() + dy, point.z() + dz);
                    octomap::OcTreeNode* node = octree_->search(check);
                    if (node && octree_->isNodeOccupied(node)) {
                        return false;
                    }
                }
            }
        }
        return true;
    }
    
    double evaluateSafePoint(const octomap::point3d& point, const octomap::point3d& frontier_center) {
        double free_space_score = 0.0;
        double check_radius = 0.8;
        int free_count = 0, total_count = 0;
        
        for (double dx = -check_radius; dx <= check_radius; dx += 0.2) {
            for (double dy = -check_radius; dy <= check_radius; dy += 0.2) {
                octomap::point3d check(point.x() + dx, point.y() + dy, point.z());
                octomap::OcTreeNode* node = octree_->search(check);
                total_count++;
                if (node && !octree_->isNodeOccupied(node)) {
                    free_count++;
                }
            }
        }
        
        free_space_score = (double)free_count / total_count;
        double distance_penalty = point.distance(frontier_center) / 3.0;
        
        return free_space_score - distance_penalty;
    }
    
    void visualizeClusters(const std::vector<FrontierCluster>& clusters) {
        visualization_msgs::MarkerArray marker_array;
        int marker_id = 0;
        
        ROS_INFO("Görselleştirme: %zu cluster marker yayınlanıyor", clusters.size());
        
        // Her küme için marker oluştur
        for (size_t i = 0; i < clusters.size(); ++i) {
            const auto& cluster = clusters[i];
            
            // Küme noktaları için marker
            visualization_msgs::Marker points_marker;
            points_marker.header.frame_id = global_frame_;
            points_marker.header.stamp = ros::Time::now();
            points_marker.ns = "frontier_points";
            points_marker.id = marker_id++;
            points_marker.type = visualization_msgs::Marker::CUBE_LIST;
            points_marker.action = visualization_msgs::Marker::ADD;
            points_marker.scale.x = octree_->getResolution();
            points_marker.scale.y = octree_->getResolution();
            points_marker.scale.z = octree_->getResolution();
            
            // Renk: İlk küme yeşil, köşeler kırmızı, diğerleri sarı
            if (i == 0) {
                points_marker.color.r = 0.0;
                points_marker.color.g = 1.0;
                points_marker.color.b = 0.0;
            } else if (cluster.is_corner) {
                points_marker.color.r = 1.0;
                points_marker.color.g = 0.0;
                points_marker.color.b = 0.0;
            } else {
                points_marker.color.r = 1.0;
                points_marker.color.g = 1.0;
                points_marker.color.b = 0.0;
            }
            points_marker.color.a = 0.5;
            
            for (const auto& point : cluster.points) {
                geometry_msgs::Point p;
                p.x = point.x();
                p.y = point.y();
                p.z = point.z();
                points_marker.points.push_back(p);
            }
            marker_array.markers.push_back(points_marker);
            
            // Centroid için sphere marker
            visualization_msgs::Marker centroid_marker;
            centroid_marker.header.frame_id = global_frame_;
            centroid_marker.header.stamp = ros::Time::now();
            centroid_marker.ns = "frontier_centroids";
            centroid_marker.id = marker_id++;
            centroid_marker.type = visualization_msgs::Marker::SPHERE;
            centroid_marker.action = visualization_msgs::Marker::ADD;
            centroid_marker.pose.position.x = cluster.centroid.x();
            centroid_marker.pose.position.y = cluster.centroid.y();
            centroid_marker.pose.position.z = cluster.centroid.z();
            centroid_marker.scale.x = 0.3;
            centroid_marker.scale.y = 0.3;
            centroid_marker.scale.z = 0.3;
            centroid_marker.color = points_marker.color;
            centroid_marker.color.a = 1.0;
            marker_array.markers.push_back(centroid_marker);
            
            // Değer metni
            visualization_msgs::Marker text_marker;
            text_marker.header = centroid_marker.header;
            text_marker.ns = "frontier_text";
            text_marker.id = marker_id++;
            text_marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
            text_marker.action = visualization_msgs::Marker::ADD;
            text_marker.pose.position = centroid_marker.pose.position;
            text_marker.pose.position.z += 0.5;
            text_marker.scale.z = 0.2;
            text_marker.color.r = text_marker.color.g = text_marker.color.b = 1.0;
            text_marker.color.a = 1.0;
            text_marker.text = "V:" + std::to_string(cluster.exploration_value).substr(0, 4) +
                              " S:" + std::to_string(cluster.size) +
                              (cluster.is_corner ? " [C]" : "");
            marker_array.markers.push_back(text_marker);
        }
        
        frontier_marker_pub_.publish(marker_array);
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "apartment_exploration_node");
    ExplorationNode node;
    ros::spin();
    return 0;
}