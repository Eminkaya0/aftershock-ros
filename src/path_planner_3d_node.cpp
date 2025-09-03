// DOSYA: path_planner_3d_node.cpp
#include <ros/ros.h>
#include <octomap_msgs/Octomap.h> // eğer baş harfi büyükse harita mesaj formatı için kullanılır demek 
#include <octomap_msgs/conversions.h>
#include <octomap/octomap.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>
#include <tf2_ros/transform_listener.h> // Robot haritanın neresinde sorusunun cevabı için 
#include <visualization_msgs/Marker.h>

// OMPL Kütüphaneleri
#include <ompl/base/SpaceInformation.h> // Planlama yapılacak uzay hakkında bilgileri kullanır.(Sınırlar çarpışma kontrolü vesaire)
#include <ompl/base/spaces/SE3StateSpace.h> // 3 boyutlu uzayı x,y,z pozisyonu + rotasyon temsil eder. 
#include <ompl/geometric/planners/rrt/RRTConnect.h> // OMPL içindeki planlama algoritmalarından biridir. 
#include <ompl/geometric/PathSimplifier.h> // Pathi daha güzel çizmek için koydum bunu 
/*
RRT NEDİR?

RRT (Rapidly-exploring Random Tree), yüksek boyutlu uzaylarda yol bulma problemlerini çözmek için kullanılan bir algoritmadır. RRT, başlangıç noktasından rastgele noktalar seçerek ve bu noktaları bir ağaç yapısında birleştirerek çalışır. Bu sayede, karmaşık ve yüksek boyutlu alanlarda bile etkili bir şekilde yol bulma işlemi gerçekleştirilir.

RRT, genellikle robotik uygulamalarda, özellikle de hareket planlamada kullanılır. Robotun hareket edebileceği alanı temsil eden bir ağaç yapısı oluşturarak, hedef noktaya ulaşmak için en uygun yolu bulmaya çalışır. RRT'nin en önemli avantajlarından biri, yüksek boyutlu uzaylarda bile hızlı bir şekilde çözüm bulabilmesidir.

RRT'nin temel adımları şunlardır:

1. Başlangıç noktasından rastgele bir hedef noktası seçilir.
2. Seçilen hedef noktasına doğru bir yol oluşturmak için ağaç yapısına yeni bir düğüm eklenir.
3. Yeni düğüm, mevcut ağaç yapısındaki diğer düğümlerle birleştirilir.
4. Hedef noktaya ulaşana kadar bu adımlar tekrarlanır.

RRT, genellikle hızlı bir başlangıç çözümü sağlasa da, optimal çözümler bulmakta zorlanabilir. Bu nedenle, RRT'nin geliştirilmiş versiyonları ve diğer planlama algoritmaları ile birleştirilerek daha iyi sonuçlar elde edilmeye çalışılmaktadır.

Önemli
*/


#include <ompl/geometric/SimpleSetup.h> // OMPL'yi kolayca kurup çalıştırmak için kullanılır

namespace ob = ompl::base;
namespace og = ompl::geometric;

class PathPlanner3D {
public:
    PathPlanner3D() : nh_("~"), tf_listener_(tf_buffer_) {
        // Parametreler
        nh_.param<std::string>("global_frame", global_frame_, "world");
        nh_.param<std::string>("robot_base_frame", robot_base_frame_, "firefly/base_link");
        nh_.param<double>("planning_timeout", planning_timeout_, 5.0);
        nh_.param<double>("robot_radius", robot_radius_, 0.1);
        nh_.param<double>("safety_margin", safety_margin_, 0.1);
        nh_.param<int>("interpolation_points", interpolation_points_, 200);
        nh_.param<double>("goal_tolerance", goal_tolerance_, 0.1);
        
        // Hedef takip değişkenlerini başlat
        has_previous_goal_ = false;

        // Subscribers & Publishers
        goal_sub_ = nh_.subscribe("/exploration_goal", 1, &PathPlanner3D::goalCallback, this);
        octomap_sub_ = nh_.subscribe("/octomap_full", 1, &PathPlanner3D::octomapCallback, this);
        path_pub_ = nh_.advertise<nav_msgs::Path>("/planned_path", 1, true);
        marker_pub_ = nh_.advertise<visualization_msgs::Marker>("/path_marker", 1, true);

        setupOMPL(); // OMPL kurulumunu constructor'da çağır.
        ROS_INFO("3D Path Planner Node başlatıldı.");
    }

private:
    ros::NodeHandle nh_;
    ros::Subscriber goal_sub_, octomap_sub_;
    ros::Publisher path_pub_, marker_pub_;
    tf2_ros::Buffer tf_buffer_; // Farklı koordinat bilgileri arasındaki dönüşüm bilgilerini saklar.
    tf2_ros::TransformListener tf_listener_; // tf_buffer_'ı sürekli güncel tutan dinleyici

    // akıllı işaretçi işi bittiğinde siler veriyi
    std::shared_ptr<octomap::OcTree> octree_;  // pointer

    std::string global_frame_, robot_base_frame_;
    double planning_timeout_;
    // Bir üye değişken olarak
    double robot_radius_; // Drone'un yarıçapı - parametreden okunuyor
    double safety_margin_; // Ek güvenlik marjı - duvarlara daha uzak geçmek için
    int interpolation_points_; // Path interpolasyon nokta sayısı
    
    // Son hedef pozisyonu takibi için
    geometry_msgs::PoseStamped last_goal_;
    bool has_previous_goal_;
    double goal_tolerance_; // Hedef değişiklik eşiği

    ob::SpaceInformationPtr space_information_; // OMPL' nin planlama yapacağı uzay hakkındaki bilgileri tutan nesne 






    
    void octomapCallback(const octomap_msgs::Octomap::ConstPtr& msg) {
        octomap::AbstractOcTree* tree;
        if (msg->binary) { // binary mi mesaj 
            tree = octomap_msgs::binaryMsgToMap(*msg);
        } else {
            tree = octomap_msgs::fullMsgToMap(*msg); // Eğer mesaj binary değilse yani full formatta ise
        }
        
        if (tree) {
            octree_ = std::shared_ptr<octomap::OcTree>(dynamic_cast<octomap::OcTree*>(tree)); // Octomap Octree formatında dönüştürüldü mü bunu sorar 
            ROS_INFO("OctoMap başarıyla alındı. Leaf sayısı: %zu", octree_->getNumLeafNodes());
        } else {
            ROS_ERROR("OctoMap mesaji dönüştürülemedi");
        }
    }


    // BELİRLİ BİR KOORDİNAT BİR ENGELE ÇARPIYOR MU??????????????

    bool isStateValid(const ob::State *state) {

        // Haritam yoksa hiçbir yer güvenli değil
        if (!octree_) {
            // Harita henüz gelmediyse, hiçbir yer güvenli değil.
            return false;
        }

        // state paketimnden robotun 3 boyutlu konumunu çıkartır 
        const auto *se3_state = state->as<ob::SE3StateSpace::StateType>();
        const double x = se3_state->getX();
        const double y = se3_state->getY();
        const double z = se3_state->getZ();

        // Dronun etrafındaki küresel bir alanı kontrol et.
        // Dronun merkezinden (yarıçap + güvenlik marjı) kadar uzaklıktaki noktaları kontrol ederiz.
        // Bu, dronun gövdesinin herhangi bir engele çarpmamasını sağlar.
        double total_radius = robot_radius_ + safety_margin_;
        for (double dx = -total_radius; dx <= total_radius; dx += octree_->getResolution()) {
            for (double dy = -total_radius; dy <= total_radius; dy += octree_->getResolution()) {
                for (double dz = -total_radius; dz <= total_radius; dz += octree_->getResolution()) {
                    // Sadece kürenin içindeki noktaları kontrol et
                    if (dx*dx + dy*dy + dz*dz <= total_radius*total_radius) {
                        octomap::OcTreeNode* node = octree_->search(x + dx, y + dy, z + dz);
                        // Eğer bu nokta haritada doluysa veya bilinmeyen bir alandaysa, state geçersizdir.
                        if (!node || octree_->isNodeOccupied(node)) {
                            return false; // Çarpışma var!
                        }
                    }
                }
            }
        }
        
        // Eğer döngü bittiyse ve hiçbir çarpışma bulunmadıysa, state geçerlidir.
        return true;
    }

    void setupOMPL() {
        // 1. Oyunun alanını tanımla (State Space)
        auto space = std::make_shared<ob::SE3StateSpace>(); // Planlama yapacağımız uzayın 3 boyutlu olduğunu söylüyoruz (3B +rotasyon)


        // 2. Oyunun alanının sınırlarını çiz.
        ob::RealVectorBounds bounds(3); // 3B alanın sınırlarını tanımlıyoruz
        bounds.setLow(0, -20); bounds.setHigh(0, 20); // X ekseni : -20 ile  +20 arası
        bounds.setLow(1, -20); bounds.setHigh(1, 20); // Y ekseni : -20 ile  +20 arası
        bounds.setLow(2, 0); bounds.setHigh(2, 15);   // Z ekseni : 0 ile 15 arası
        space->setBounds(bounds); // Robota sanal bir kutu tanımlıyoruz bunun dışına path çizemezsin

        // 3. Alan bilgilerini ve Hakem ayarı
        space_information_ = std::make_shared<ob::SpaceInformation>(space);
        space_information_->setStateValidityChecker([this](const ob::State *state) { return isStateValid(state); });
        
        // Bu satır, OMPL'e iki nokta arasındaki yolu kontrol ederken hangi sıklıkta
        // isStateValid fonksiyonunu çağıracağını söyler. %1 (0.01) iyi bir başlangıçtır.
        space_information_->setStateValidityCheckingResolution(0.01);


        space_information_->setup();
    }

    void goalCallback(const geometry_msgs::PoseStamped::ConstPtr& goal_msg) {

        // Hedef değişiklik kontrolü - aynı hedefe tekrar planlama yapma
        if (has_previous_goal_) {
            double dx = goal_msg->pose.position.x - last_goal_.pose.position.x;
            double dy = goal_msg->pose.position.y - last_goal_.pose.position.y;
            double dz = goal_msg->pose.position.z - last_goal_.pose.position.z;
            double distance = sqrt(dx*dx + dy*dy + dz*dz);
            
            if (distance < goal_tolerance_) {
                ROS_DEBUG("Hedef değişmedi (mesafe: %.3f < %.3f), planlama atlandı", distance, goal_tolerance_);
                return; // Aynı hedef, planlama yapma
            }
        }

        // HAZIRLIK VE KONTROL KODLARI
        ROS_INFO("Yeni hedef alındı: [%.2f, %.2f, %.2f]", 
                 goal_msg->pose.position.x, goal_msg->pose.position.y, goal_msg->pose.position.z);
        
        if (!octree_ || !space_information_) {
            ROS_WARN("Harita veya OMPL kurulumu hazır değil. octree: %s, space_info: %s", 
                     octree_ ? "OK" : "NULL", space_information_ ? "OK" : "NULL");
            return;
        }

        geometry_msgs::TransformStamped transform_stamped;
        try {
            transform_stamped = tf_buffer_.lookupTransform(global_frame_, robot_base_frame_, ros::Time(0), ros::Duration(1.0));
        } catch (tf2::TransformException &ex) {
            ROS_WARN("Başlangıç konumu alınamadı: %s", ex.what());
            return;
        }


        // 1.Başlangıç konumunu belirle (Robot şuan nerede?)
        //  !!!!!!   (tf_buffer_.lookupTransform ile robotun anlık konumu bulunur)
        ob::ScopedState<> start(space_information_);
        start->as<ob::SE3StateSpace::StateType>()->setXYZ(
            transform_stamped.transform.translation.x,
            transform_stamped.transform.translation.y,
            transform_stamped.transform.translation.z);
        start->as<ob::SE3StateSpace::StateType>()->rotation().setIdentity(); //Robotun mevcut x y z'si 


        // 2. hedef konumu belirle (Robot nereye gitmeli?)
        ob::ScopedState<> goal(space_information_);
        goal->as<ob::SE3StateSpace::StateType>()->setXYZ(
            goal_msg->pose.position.x, // Gelen hedef mesajındaki x,y,z
            goal_msg->pose.position.y,
            goal_msg->pose.position.z);
        goal->as<ob::SE3StateSpace::StateType>()->rotation().setIdentity();

        // 3. Planlayıcıyı kur
        auto ss = std::make_shared<og::SimpleSetup>(space_information_);
        ss->setStartAndGoalStates(start, goal);
        ss->setPlanner(std::make_shared<og::RRTConnect>(space_information_)); // RRT algoritmasını kullan

        // 4. Planlamayı Başlat ve Sonucu Kontrol Et
        ROS_INFO("3D yol planlaması başlatıldı...");
        ob::PlannerStatus solved = ss->solve(planning_timeout_);

        if (solved) {
        ROS_INFO("Yol bulundu! Şimdi basitleştiriliyor...");
        
        // --- YENİ EKLENEN KISIM ---
        // PathSimplifier objesi oluştur.
        auto simplifier = std::make_shared<og::PathSimplifier>(space_information_);
        og::PathGeometric path = ss->getSolutionPath();

        // Yolu basitleştir ve daha pürüzsüz hale getirmeye çalış.
        // Bu işlem, gereksiz ara noktaları kaldırır ve yolu kısaltır.
        if (simplifier->simplify(path, planning_timeout_)) { // Kalan sürede basitleştirmeyi dene
             ROS_INFO("Yol başarıyla basitleştirildi.");
        } else {
             ROS_WARN("Yol basitleştirilemedi, orijinal yol kullanılıyor.");
        }
        
        // Daha fazla nokta ekleyerek yolu daha akıcı hale getir.
        path.interpolate(interpolation_points_); // Parametreden okunan nokta sayısı kadar ekle
        // --- YENİ KISIM SONU ---

        publishPath(path);
        
        // Başarılı planlama sonrası hedefi kaydet
        last_goal_ = *goal_msg;
        has_previous_goal_ = true;

        } else {
            ROS_WARN("Belirtilen sürede yol bulunamadı.");
        }
    }

        // Ompl'nin oluşturduğu yolu yayınla
    void publishPath(const og::PathGeometric& path) {
        // 1. Makine için Yol Mesajı Oluştur (Robotun Anlayacağı Format)
        nav_msgs::Path ros_path;
        ros_path.header.stamp = ros::Time::now();      // Mesaja zaman damgası ekle
        ros_path.header.frame_id = global_frame_;     // Koordinat sistemini belirt
        
        // 2. İnsan için Görsel İşaretçi Oluştur (Bizim Göreceğimiz Format)
        visualization_msgs::Marker path_marker;
        path_marker.header = ros_path.header;          // Aynı zaman ve koordinat sistemi
        path_marker.ns = "planned_path";               // İşaretçiye bir isim ver
        path_marker.id = 0;                            // Benzersiz bir kimlik numarası
        path_marker.type = visualization_msgs::Marker::LINE_STRIP; // Tipi: Birleştirilmiş çizgiler
        path_marker.action = visualization_msgs::Marker::ADD;  // Ekrana ekle/güncelle
        path_marker.pose.orientation.w = 1.0;          // Dönüklük (önemsiz)
        path_marker.scale.x = 0.1;                     // Çizgi kalınlığı (0.1 metre)
        path_marker.color.a = 1.0;                     // Opaklık (tamamen görünür)
        path_marker.color.g = 1.0;                     // Renk (Yeşil)

        // 3. Yolu Nokta Nokta Gez ve İki Mesajı da Doldur
        for (std::size_t i = 0; i < path.getStateCount(); ++i) {
            // OMPL formatındaki sıradaki noktayı al ve (x,y,z)'sini çıkar
            const auto* state = path.getState(i)->as<ob::SE3StateSpace::StateType>();
            double x = state->getX(), y = state->getY(), z = state->getZ();
            
            // Bu noktayı makine için olan mesaja ekle
            geometry_msgs::PoseStamped pose;
            pose.header = ros_path.header;
            pose.pose.position.x = x;
            pose.pose.position.y = y;
            pose.pose.position.z = z;
            ros_path.poses.push_back(pose);
            
            // Aynı noktayı insan için olan görsel çizgiye ekle
            geometry_msgs::Point p;
            p.x = x; p.y = y; p.z = z;
            path_marker.points.push_back(p);
        }

        // 4. Hazırlanan Mesajları Yayınla
        path_pub_.publish(ros_path);
        marker_pub_.publish(path_marker);
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "path_planner_3d_node");
    PathPlanner3D planner;
    ros::spin();
    return 0;
}