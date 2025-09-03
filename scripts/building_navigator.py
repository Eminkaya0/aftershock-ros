#!/usr/bin/env python3
# # -*- coding: utf-8 -*-

import rospy
from nav_msgs.msg import Odometry
from trajectory_msgs.msg import MultiDOFJointTrajectory, MultiDOFJointTrajectoryPoint
from geometry_msgs.msg import Transform, Twist, PoseStamped
from std_msgs.msg import String
import yaml
import tf
import math

class BuildingNavigator:
    def __init__(self):
        rospy.init_node('building_navigator', anonymous=True)
        rospy.loginfo("Building Navigator başlatılıyor...")
        
        # Parametreler
        self.geofence_file = rospy.get_param('~geofence_file', '/tmp/geofence.yaml')
        self.approach_altitude = rospy.get_param('~approach_altitude', 30.0)
        self.inspection_radius = rospy.get_param('~inspection_radius', 10.0)
        self.waypoint_tolerance = rospy.get_param('~waypoint_tolerance', 2.0)
        
        # Veri
        self.geofence_data = None
        self.current_pose = None
        self.current_building_index = 0
        self.waypoints = []
        
        # Publishers
        self.trajectory_pub = rospy.Publisher('/firefly/command/trajectory', MultiDOFJointTrajectory, queue_size=10)
        self.status_pub = rospy.Publisher('/building_navigator/status', String, queue_size=1)
        self.waypoint_pub = rospy.Publisher('/building_navigator/current_waypoint', PoseStamped, queue_size=1)
        
        # Subscribers
        self.odom_sub = rospy.Subscriber('/firefly/ground_truth/odometry', Odometry, self.odom_callback)
        self.command_sub = rospy.Subscriber('/mission/command', String, self.command_callback)
        
        self.active = False
        rospy.loginfo("Building Navigator hazır!")
        
    def odom_callback(self, msg):
        self.current_pose = msg.pose.pose
        
    def command_callback(self, msg):
        if msg.data == "start_navigation":
            self.load_geofence_data()
            self.plan_waypoints()
            self.active = True
            rospy.loginfo("Navigasyon başlatıldı!")
            
    def load_geofence_data(self):
        """YAML dosyasından geofence verilerini yükle"""
        try:
            with open(self.geofence_file, 'r') as f:
                self.geofence_data = yaml.safe_load(f)
            rospy.loginfo(f"Geofence verisi yüklendi: {len(self.geofence_data['safe_buildings'])} bina bulundu.")
        except Exception as e:
            rospy.logerr(f"Geofence dosyası yüklenemedi: {e}")
            
    def plan_waypoints(self):
        """Her bina için waypoint'ler oluştur"""
        if not self.geofence_data:
            return
            
        self.waypoints = []
        
        for building in self.geofence_data['safe_buildings']:
            center = building['center']
            
            # Binaya yaklaşma waypoint'i
            approach_point = {
                'x': center[0],
                'y': center[1],
                'z': self.approach_altitude,
                'type': 'approach',
                'building_id': building['id']
            }
            self.waypoints.append(approach_point)
            
            # Bina etrafında dönüş waypoint'leri (opsiyonel)
            for angle in [0, 90, 180, 270]:
                rad = math.radians(angle)
                inspection_point = {
                    'x': center[0] + self.inspection_radius * math.cos(rad),
                    'y': center[1] + self.inspection_radius * math.sin(rad),
                    'z': self.approach_altitude,
                    'type': 'inspection',
                    'building_id': building['id']
                }
                self.waypoints.append(inspection_point)
                
        rospy.loginfo(f"Toplam {len(self.waypoints)} waypoint oluşturuldu.")
        
    def send_trajectory_command(self, x, y, z, yaw=0):
        """Firefly için trajectory komutu gönder"""
        traj = MultiDOFJointTrajectory()
        traj.header.stamp = rospy.Time.now()
        traj.header.frame_id = "world"
        
        point = MultiDOFJointTrajectoryPoint()
        
        # Transform
        transform = Transform()
        transform.translation.x = x
        transform.translation.y = y
        transform.translation.z = z
        
        # Quaternion
        q = tf.transformations.quaternion_from_euler(0, 0, yaw)
        transform.rotation.x = q[0]
        transform.rotation.y = q[1]
        transform.rotation.z = q[2]
        transform.rotation.w = q[3]
        
        point.transforms.append(transform)
        
        # Velocity
        twist = Twist()
        point.velocities.append(twist)
        
        # Time
        point.time_from_start = rospy.Duration(5.0)  # Waypoint'e ulaşma süresi
        
        traj.points.append(point)
        self.trajectory_pub.publish(traj)
        
    def distance_to_waypoint(self, waypoint):
        """Mevcut pozisyondan waypoint'e mesafe"""
        if not self.current_pose:
            return float('inf')
            
        dx = self.current_pose.position.x - waypoint['x']
        dy = self.current_pose.position.y - waypoint['y']
        dz = self.current_pose.position.z - waypoint['z']
        
        return math.sqrt(dx*dx + dy*dy + dz*dz)
        
    def run(self):
        rate = rospy.Rate(10)  # 10 Hz
        
        while not rospy.is_shutdown():
            if not self.active or not self.waypoints or not self.current_pose:
                rate.sleep()
                continue
                
            if self.current_building_index >= len(self.waypoints):
                rospy.loginfo("Tüm waypoint'ler tamamlandı!")
                self.status_pub.publish("navigation_completed")
                self.active = False
                continue
                
            # Mevcut waypoint
            current_waypoint = self.waypoints[self.current_building_index]
            
            # Waypoint'e git
            self.send_trajectory_command(
                current_waypoint['x'],
                current_waypoint['y'],
                current_waypoint['z']
            )
            
            # Waypoint'i görselleştir
            waypoint_msg = PoseStamped()
            waypoint_msg.header.frame_id = "world"
            waypoint_msg.header.stamp = rospy.Time.now()
            waypoint_msg.pose.position.x = current_waypoint['x']
            waypoint_msg.pose.position.y = current_waypoint['y']
            waypoint_msg.pose.position.z = current_waypoint['z']
            waypoint_msg.pose.orientation.w = 1.0
            self.waypoint_pub.publish(waypoint_msg)
            
            # Waypoint'e ulaşıldı mı?
            distance = self.distance_to_waypoint(current_waypoint)
            if distance < self.waypoint_tolerance:
                rospy.loginfo(f"Waypoint {self.current_building_index + 1}/{len(self.waypoints)} tamamlandı. "
                            f"(Tip: {current_waypoint['type']}, Bina: {current_waypoint['building_id']})")
                
                # İnceleme waypoint'inde biraz bekle
                if current_waypoint['type'] == 'inspection':
                    rospy.sleep(2.0)
                    
                self.current_building_index += 1
                
            rate.sleep()

if __name__ == '__main__':
    try:
        navigator = BuildingNavigator()
        navigator.run()
    except rospy.ROSInterruptException:
        rospy.loginfo("Building Navigator kapatılıyor...")