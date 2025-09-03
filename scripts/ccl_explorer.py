#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import numpy as np
from scipy.ndimage import label, center_of_mass
import tf.transformations as tft

# --- Gerekli ROS Mesajları ---
from nav_msgs.msg import Odometry
from octomap_msgs.msg import Octomap # Gerçek harita için
from visualization_msgs.msg import Marker

# --- Harita Durumları (Simülasyon için) ---
FREE_SPACE = 1
OBSTACLE = 2
UNKNOWN = 3

def create_simulated_building(width, depth, height, wall_thickness=1):
    """Hızlı test için, içi boş, 3D bir bina simülasyonu oluşturur."""
    grid = np.full((width, depth, height), UNKNOWN, dtype=int)
    # İçini boşalt
    grid[wall_thickness:-wall_thickness, wall_thickness:-wall_thickness, wall_thickness:-wall_thickness] = FREE_SPACE
    # Duvarları (engelleri) oluştur
    grid[0:wall_thickness, :, :] = OBSTACLE
    grid[-wall_thickness:, :, :] = OBSTACLE
    grid[:, 0:wall_thickness, :] = OBSTACLE
    grid[:, -wall_thickness:, :] = OBSTACLE
    grid[:, :, 0:wall_thickness] = OBSTACLE # Zemin
    grid[:, :, -wall_thickness:] = OBSTACLE # Tavan
    return grid

class InteractiveExplorer:
    def __init__(self):
        rospy.init_node('interactive_ccl_explorer', anonymous=True)
        rospy.loginfo("Interactive CCL Explorer (Yardımcı Pilot) başlatılıyor...")

        # --- Parametreler ---
        self.map_resolution = 0.5  # Haritanızın çözünürlüğü (metre/hücre)
        self.perception_radius_cells = 4 # Dronun etrafındaki alanı ne kadar "açık" sayacağı

        # --- Veri Depolama ---
        self.map_grid = None
        self.map_origin = np.array([0.0, 0.0, 0.0]) # Haritanın başlangıç koordinatı
        self.robot_pose = None
        
        # --- ÖNEMLİ: Hızlı Test için Simülasyon ---
        self.map_grid = create_simulated_building(40, 40, 10) 
        rospy.loginfo("Hızlı test için simüle edilmiş bina haritası yüklendi.")


        # --- ROS Arayüzleri ---
        # Publishers: RViz'de görselleştirmek için
        self.frontier_marker_pub = rospy.Publisher('/exploration/frontiers', Marker, queue_size=1)
        self.target_marker_pub = rospy.Publisher('/exploration/suggested_target', Marker, queue_size=1)

        # Subscribers: Dron ve Harita verisini dinlemek için
        self.odom_sub = rospy.Subscriber('/firefly/ground_truth/odometry', Odometry, self.odom_callback)
        # self.octomap_sub = rospy.Subscriber('/octomap_full', Octomap, self.octomap_callback) # GERÇEK HARİTA İÇİN BUNU AÇIN

        # Ana döngüyü her 3 saniyede bir çalıştır
        self.timer = rospy.Timer(rospy.Duration(3), self.update_exploration_targets)
        rospy.loginfo("Yardımcı pilot hazır. RViz'de hedefler gösterilecek.")

    def odom_callback(self, msg):
        self.robot_pose = msg.pose.pose

    def octomap_callback(self, msg):
        rospy.logwarn_once("OctoMap callback'i henüz gerçek bir dönüşüm yapmıyor!")
        # self.map_grid, self.map_origin, self.map_resolution = convert_octomap_to_numpy(msg)
        pass
    
    def world_to_grid(self, world_pos):
        if world_pos is None: return None
        grid_pos = (np.array([world_pos.x, world_pos.y, world_pos.z]) - self.map_origin) / self.map_resolution
        return grid_pos.astype(int)

    def grid_to_world(self, grid_pos):
        world_pos = (np.array(grid_pos) * self.map_resolution) + self.map_origin
        return world_pos

    def update_exploration_targets(self, event):
        if self.map_grid is None or self.robot_pose is None:
            rospy.loginfo_throttle(10, "Harita veya pozisyon verisi bekleniyor...")
            return

        robot_grid_pos = self.world_to_grid(self.robot_pose.position)
        
        # Pozisyon harita sınırları dışındaysa işlem yapma
        if np.any(robot_grid_pos < 0) or np.any(robot_grid_pos >= self.map_grid.shape):
             rospy.logwarn_throttle(10, f"Robot pozisyonu ({robot_grid_pos}) harita sınırları dışında. Bekleniyor...")
             return

        # Simülasyon için: Dronun etrafını "görülmüş" olarak işaretle
        x, y, z = robot_grid_pos
        r = self.perception_radius_cells
        x_min, x_max = max(0, x-r), min(self.map_grid.shape[0], x+r+1)
        y_min, y_max = max(0, y-r), min(self.map_grid.shape[1], y+r+1)
        z_min, z_max = max(0, z-r), min(self.map_grid.shape[2], z+r+1)
        view_area = self.map_grid[x_min:x_max, y_min:y_max, z_min:z_max]
        view_area[view_area == UNKNOWN] = FREE_SPACE

        # --- CCL Keşif Mantığı ---
        frontiers = self.find_frontiers()
        if len(frontiers) == 0:
            rospy.loginfo("Keşfedilecek alan kalmadı!")
            self.publish_frontier_markers([]) # Eski markerları sil
            self.publish_target_marker(None)
            return
            
        frontier_map = np.zeros(self.map_grid.shape)
        frontier_map[tuple(frontiers.T)] = 1
        labeled_frontiers, num_clusters = label(frontier_map)
        
        if num_clusters == 0: return
        
        potential_targets = []
        cluster_centroids = np.array([c for c in center_of_mass(frontier_map, labeled_frontiers, range(1, num_clusters + 1)) if not np.isnan(c).any()])
        if len(cluster_centroids) == 0: return

        for i, centroid in enumerate(cluster_centroids):
            cluster_id = i + 1
            cluster_coords = np.argwhere(labeled_frontiers == cluster_id)
            if len(cluster_coords) == 0: continue
            distances = np.linalg.norm(cluster_coords - np.array(centroid), axis=1)
            closest_cell = cluster_coords[np.argmin(distances)]
            potential_targets.append(closest_cell)
        
        if not potential_targets: return

        distances_to_robot = np.linalg.norm(np.array(potential_targets) - robot_grid_pos, axis=1)
        best_target_grid = potential_targets[np.argmin(distances_to_robot)]

        # --- Görselleştirme ---
        self.publish_frontier_markers(frontiers)
        self.publish_target_marker(best_target_grid)

    def find_frontiers(self):
        free_coords = np.argwhere(self.map_grid == FREE_SPACE)
        frontiers = set()
        for x, y, z in free_coords:
            for dx, dy, dz in [(0,0,1), (0,0,-1), (0,1,0), (0,-1,0), (1,0,0), (-1,0,0)]:
                nx, ny, nz = x + dx, y + dy, z + dz
                if 0 <= nx < self.map_grid.shape[0] and 0 <= ny < self.map_grid.shape[1] and 0 <= nz < self.map_grid.shape[2]:
                    if self.map_grid[nx, ny, nz] == UNKNOWN:
                        frontiers.add((nx, ny, nz))
        return np.array(list(frontiers))

    def publish_frontier_markers(self, frontier_cells_grid):
        marker = Marker()
        marker.header.frame_id = "world"
        marker.header.stamp = rospy.Time.now()
        marker.ns = "frontiers"
        marker.id = 0
        marker.type = Marker.CUBE_LIST
        marker.action = Marker.ADD
        marker.scale.x = self.map_resolution
        marker.scale.y = self.map_resolution
        marker.scale.z = self.map_resolution
        marker.color.b = 1.0
        marker.color.a = 0.4

        if len(frontier_cells_grid) == 0:
            marker.action = Marker.DELETEALL
        else:
            for p in frontier_cells_grid:
                world_p = self.grid_to_world(p)
                marker.points.append(rospy.rostime.Point(*world_p))
        
        self.frontier_marker_pub.publish(marker)

    def publish_target_marker(self, target_cell_grid):
        marker = Marker()
        marker.header.frame_id = "world"
        marker.header.stamp = rospy.Time.now()
        marker.ns = "suggested_target"
        marker.id = 1
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.scale.x = self.map_resolution * 3
        marker.scale.y = self.map_resolution * 3
        marker.scale.z = self.map_resolution * 3
        marker.color.g = 1.0
        marker.color.a = 0.8
        
        if target_cell_grid is None:
            marker.action = Marker.DELETE
        else:
            world_p = self.grid_to_world(target_cell_grid)
            marker.pose.position.x = world_p[0]
            marker.pose.position.y = world_p[1]
            marker.pose.position.z = world_p[2]
            marker.pose.orientation.w = 1.0
        
        self.target_marker_pub.publish(marker)

# Burası en önemli kısım. Bu blok olmadan kod hemen kapanır.
if __name__ == '__main__':
    try:
        explorer = InteractiveExplorer()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("Yardımcı pilot kapatılıyor.")