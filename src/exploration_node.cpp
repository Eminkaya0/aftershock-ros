#include <ros/ros.h>
#include <octomap_msgs/Octomap.h>
#include <octomap_msgs/conversions.h>
#include <octomap/octomap.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Point.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <visualization_msgs/MarkerArray.h>

#include <vector>
#include <queue>
#include <cmath>
#include <algorithm>
#include <map>

// octomap::point3d için bir karsilastirma operatoru lazim, map içerisinde anahtar olarak kullanabilmek için.
struct Point3dComparator {
    bool operator()(const octomap::point3d& a, const octomap::point3d& b) const {
        if (a.x() != b.x()) return a.x() < b.x();
        if (a.y() != b.y()) return a.y() < b.y();
        return a.z() < b.z();
    }
};

class ExplorationNode {
public:
    ExplorationNode() : nh_("~"), tf_listener_(tf_buffer_) {
        

        // ...
        // Sinir parametrelerini al
        nh_.param<double>("min_x", min_x_, -11.5);
        nh_.param<double>("max_x", max_x_, -1.0);
        nh_.param<double>("min_y", min_y_, -5.0);
        nh_.param<double>("max_y", max_y_, 5.0);
        nh_.param<double>("min_z", min_z_, 0.0);
        nh_.param<double>("max_z", max_z_, 12.8);
    


        // Parametreleri al
        nh_.param<std::string>("octomap_topic", octomap_topic_, "/octomap_full");
        nh_.param<std::string>("goal_topic", goal_topic_, "/exploration_goal");
        nh_.param<std::string>("global_frame", global_frame_, "world");
        nh_.param<std::string>("robot_base_frame", robot_base_frame_, "base_link");
        nh_.param<double>("update_frequency", update_frequency_, 1.0); // Hz
        nh_.param<int>("min_cluster_size", min_cluster_size_, 10); // Gurultuyu engellemek için minimum kume boyutu

        // Subscriber ve Publisher'lari baslat
        octomap_sub_ = nh_.subscribe(octomap_topic_, 1, &ExplorationNode::octomapCallback, this);
        goal_pub_ = nh_.advertise<geometry_msgs::PoseStamped>(goal_topic_, 1);
        marker_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("/exploration_markers", 10); // <-- BU SATIRI EKLE
        
        // Ana islem dongusu için Timer
        timer_ = nh_.createTimer(ros::Duration(1.0 / update_frequency_), &ExplorationNode::processMapCallback, this);

        ROS_INFO("Apartment Exploration Node baslatildi.");
        ROS_INFO("OctoMap konusu: %s", octomap_topic_.c_str());
        ROS_INFO("Hedef konusu: %s", goal_topic_.c_str());
    }

private:

        double min_x_, max_x_, min_y_, max_y_, min_z_, max_z_;

    void octomapCallback(const octomap_msgs::Octomap::ConstPtr& msg) {
        octomap::AbstractOcTree* tree = octomap_msgs::msgToMap(*msg);
        if (tree) {
            octree_.reset(dynamic_cast<octomap::OcTree*>(tree));
            map_received_ = true;
            ROS_INFO_ONCE("İlk OctoMap haritasi basariyla alindi.");
        } else {
            ROS_ERROR("OctoMap mesaji donusturulemedi.");
        }
    }

    void processMapCallback(const ros::TimerEvent& event) {
        if (!map_received_ || !octree_) {
            ROS_WARN("Henuz harita alinmadi, islem bekleniyor...");
            return;
        }

        geometry_msgs::TransformStamped robot_transform;
        try {
            robot_transform = tf_buffer_.lookupTransform(global_frame_, robot_base_frame_, ros::Time(0));
        } catch (tf2::TransformException &ex) {
            ROS_WARN("Robot konumu alinamadi: %s", ex.what());
            return;
        }
        octomap::point3d robot_pos(robot_transform.transform.translation.x,
                                   robot_transform.transform.translation.y,
                                   robot_transform.transform.translation.z);

        // 1. Adim: Sinir (Frontier) Hucrelerini Bul
        std::vector<octomap::point3d> frontier_cells = findFrontierCells();
        if (frontier_cells.empty()) {
            ROS_INFO("Kesfedilecek yeni sinir hucresi bulunamadi. Kesif tamamlanmis olabilir.");
            return;
        }
        
        // 2. Adim: Sinir Hucrelerini Kumele (Connected Component Labelling)
        std::vector<std::vector<octomap::point3d>> clusters = clusterFrontierCells(frontier_cells);

        // 3. Adim: Kumelerden Potansiyel Hedefler Olustur
        std::vector<geometry_msgs::Point> potential_goals = calculateGoalsFromClusters(clusters);
        if (potential_goals.empty()) {
            ROS_INFO("Yeterli buyuklukte bir hedef kumesi bulunamadi.");
            return;
        }
        
        // 4. Adim: En Yakin Hedefi Seç ve Yayinla
        selectAndPublishBestGoal(potential_goals, robot_pos);
    }

    std::vector<octomap::point3d> findFrontierCells() {
        std::vector<octomap::point3d> frontiers;
        for (octomap::OcTree::leaf_iterator it = octree_->begin_leafs(), end = octree_->end_leafs(); it != end; ++it) {
            // Sadece 'bos' (free) hucrelerle ilgileniyoruz
            if (octree_->isNodeOccupied(*it)) continue;

            octomap::point3d p = it.getCoordinate();

                // SINIR KONTROLuNu BURAYA EKLE
            if (p.x() < min_x_ || p.x() > max_x_ ||
                p.y() < min_y_ || p.y() > max_y_ ||
                p.z() < min_z_ || p.z() > max_z_) {
                continue; // Eğer hucre sinirlar disindaysa, atla
            }

            bool is_frontier = false;

            // 6 komsuyu kontrol et (up, down, left, right, forward, back)
            for (int dx = -1; dx <= 1; ++dx) {
                for (int dy = -1; dy <= 1; ++dy) {
                    for (int dz = -1; dz <= 1; ++dz) {
                        if (abs(dx) + abs(dy) + abs(dz) != 1) continue; // Sadece 6'li komsuluk

                        octomap::point3d neighbor = p;
                        neighbor.x() += dx * octree_->getResolution();
                        neighbor.y() += dy * octree_->getResolution();
                        neighbor.z() += dz * octree_->getResolution();

                        octomap::OcTreeNode* node = octree_->search(neighbor);
                        if (node == nullptr) { // Eğer komsu 'bilinmeyen' (unknown) ise
                            is_frontier = true;
                            break;
                        }
                    }
                    if (is_frontier) break;
                }
                if (is_frontier) break;
            }

            if (is_frontier) {
                frontiers.push_back(p);
            }
        }
        ROS_INFO("%zu adet sinir hucresi bulundu.", frontiers.size());
        return frontiers;
    }
    
    std::vector<std::vector<octomap::point3d>> clusterFrontierCells(const std::vector<octomap::point3d>& frontier_cells) {
        std::vector<std::vector<octomap::point3d>> clusters;
        std::map<octomap::point3d, bool, Point3dComparator> visited;
        for(const auto& cell : frontier_cells) {
            visited[cell] = false;
        }

        for (const auto& cell : frontier_cells) {
            if (!visited[cell]) {
                std::vector<octomap::point3d> current_cluster;
                std::queue<octomap::point3d> q;

                q.push(cell);
                visited[cell] = true;

                while (!q.empty()) {
                    octomap::point3d current_cell = q.front();
                    q.pop();
                    current_cluster.push_back(current_cell);

                    // Komsulari kontrol et (BFS)
                     for (int dx = -1; dx <= 1; ++dx) {
                        for (int dy = -1; dy <= 1; ++dy) {
                            for (int dz = -1; dz <= 1; ++dz) {
                                if (dx == 0 && dy == 0 && dz == 0) continue;

                                octomap::point3d neighbor = current_cell;
                                neighbor.x() += dx * octree_->getResolution();
                                neighbor.y() += dy * octree_->getResolution();
                                neighbor.z() += dz * octree_->getResolution();

                                // Eğer komsu da bir sinir hucresiyse ve ziyaret edilmediyse, kuyruğa ekle
                                if (visited.count(neighbor) && !visited[neighbor]) {
                                    visited[neighbor] = true;
                                    q.push(neighbor);
                                }
                            }
                        }
                    }
                }
                clusters.push_back(current_cluster);
            }
        }
        ROS_INFO("%zu adet kume olusturuldu.", clusters.size());
        return clusters;
    }

    std::vector<geometry_msgs::Point> calculateGoalsFromClusters(const std::vector<std::vector<octomap::point3d>>& clusters) {
        std::vector<geometry_msgs::Point> goals;
        for (const auto& cluster : clusters) {
            if (cluster.size() < min_cluster_size_) continue;

            // Ağirlik merkezini (centroid) hesapla
            octomap::point3d sum(0, 0, 0);
            for (const auto& point : cluster) {
                sum += point;
            }
            //octomap::point3d centroid = sum / cluster.size();

            // Hatali satiri silip yerine bu bloğu ekleyin

            // Ondalikli bolme isleminin doğru yapilmasi için boleni float'a çeviriyoruz. Bu iyi bir aliskanliktir.
            float num_points = static_cast<float>(cluster.size());

            // Centroid'i bilesen bilesen hesapliyoruz
            octomap::point3d centroid;
            centroid.x() = sum.x() / num_points;
            centroid.y() = sum.y() / num_points;
            centroid.z() = sum.z() / num_points;

            // Centroid'e en yakin gerçek hucreyi bul (sizin onerdiğiniz iyilestirme)
            double min_dist_sq = -1.0;
            octomap::point3d closest_point;

            for (const auto& point : cluster) {
                double dist_sq = std::pow(centroid.distance(point), 2);
                if (min_dist_sq < 0 || dist_sq < min_dist_sq) {
                    min_dist_sq = dist_sq;
                    closest_point = point;
                }
            }
            
            geometry_msgs::Point goal_point;
            goal_point.x = closest_point.x();
            goal_point.y = closest_point.y();
            goal_point.z = closest_point.z();
            goals.push_back(goal_point);
        }
        ROS_INFO("%zu adet potansiyel hedef noktasi belirlendi.", goals.size());
        return goals;
    }

    void selectAndPublishBestGoal(std::vector<geometry_msgs::Point>& goals, const octomap::point3d& robot_pos) {
        // Hedefleri robota olan mesafeye gore sirala
        std::sort(goals.begin(), goals.end(), [&](const geometry_msgs::Point& a, const geometry_msgs::Point& b) {
            double dist_a = std::sqrt(pow(a.x - robot_pos.x(), 2) + pow(a.y - robot_pos.y(), 2) + pow(a.z - robot_pos.z(), 2));
            double dist_b = std::sqrt(pow(b.x - robot_pos.x(), 2) + pow(b.y - robot_pos.y(), 2) + pow(b.z - robot_pos.z(), 2));
            return dist_a < dist_b;
        });

        // En yakin hedefi seç
        geometry_msgs::Point best_goal = goals.front();

        // Hedefi PoseStamped olarak yayinla
        geometry_msgs::PoseStamped goal_pose;
        goal_pose.header.stamp = ros::Time::now();
        goal_pose.header.frame_id = global_frame_;
        goal_pose.pose.position = best_goal;
        goal_pose.pose.orientation.w = 1.0; // Basit bir orientation, robot hedefe donuk olsun

        goal_pub_.publish(goal_pose);
        ROS_INFO("En yakin hedef yayinlandi: [x: %.2f, y: %.2f, z: %.2f]", best_goal.x, best_goal.y, best_goal.z);
    // --- YENİ KISIM: RViz İşaretçilerini Oluştur ve Yayınla ---
        visualization_msgs::MarkerArray marker_array;
        int marker_id = 0;

        // 1. İşaretçi: En yakın hedef (Büyük ve Yeşil Küre)
        visualization_msgs::Marker best_goal_marker;
        best_goal_marker.header.frame_id = global_frame_;
        best_goal_marker.header.stamp = ros::Time::now();
        best_goal_marker.ns = "exploration_goals";
        best_goal_marker.id = marker_id++;
        best_goal_marker.type = visualization_msgs::Marker::SPHERE;
        best_goal_marker.action = visualization_msgs::Marker::ADD;
        best_goal_marker.pose.position = best_goal;
        best_goal_marker.pose.orientation.w = 1.0;
        best_goal_marker.scale.x = 0.5; // Küre çapı
        best_goal_marker.scale.y = 0.5;
        best_goal_marker.scale.z = 0.5;
        best_goal_marker.color.a = 1.0; // Opaklık
        best_goal_marker.color.r = 0.0;
        best_goal_marker.color.g = 1.0; // Yeşil
        best_goal_marker.color.b = 0.0;
        marker_array.markers.push_back(best_goal_marker);

        // 2. İşaretçiler: Diğer potansiyel hedefler (Daha küçük ve Sarı Küreler)
        for (size_t i = 1; i < goals.size(); ++i) {
            visualization_msgs::Marker other_goal_marker;
            other_goal_marker.header.frame_id = global_frame_;
            other_goal_marker.header.stamp = ros::Time::now();
            other_goal_marker.ns = "exploration_goals";
            other_goal_marker.id = marker_id++;
            other_goal_marker.type = visualization_msgs::Marker::SPHERE;
            other_goal_marker.action = visualization_msgs::Marker::ADD;
            other_goal_marker.pose.position = goals[i];
            other_goal_marker.pose.orientation.w = 1.0;
            other_goal_marker.scale.x = 0.3;
            other_goal_marker.scale.y = 0.3;
            other_goal_marker.scale.z = 0.3;
            other_goal_marker.color.a = 0.8; 
            other_goal_marker.color.r = 1.0;
            other_goal_marker.color.g = 1.0; // Sarı
            other_goal_marker.color.b = 0.0;
            marker_array.markers.push_back(other_goal_marker);
        }
        
        // İşaretçi dizisini yayınla
        marker_pub_.publish(marker_array);
    }



    ros::NodeHandle nh_;
    ros::Subscriber octomap_sub_;
    ros::Publisher goal_pub_;
    ros::Publisher marker_pub_;
    ros::Timer timer_;

    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;
    
    std::unique_ptr<octomap::OcTree> octree_;
    bool map_received_ = false;

    // Parametreler
    std::string octomap_topic_;
    std::string goal_topic_;
    std::string global_frame_;
    std::string robot_base_frame_;
    double update_frequency_;
    int min_cluster_size_;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "apartment_exploration_node");
    ExplorationNode node;
    ros::spin();
    return 0;
}