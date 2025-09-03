#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Point, PointStamped
from visualization_msgs.msg import Marker, MarkerArray
import yaml
import numpy as np

class GeofenceGenerator:
    def __init__(self):
        rospy.init_node('geofence_generator', anonymous=True)
        rospy.loginfo("Geofence Generator başlatılıyor...")
        
        # YENİ ve DOĞRU HALİ
        # Parametreleri global ilan panosundan, YAML'deki yapıya göre oku.
        self.output_file = rospy.get_param('/outputs/geofence_file', '/tmp/geofence.yaml')
        self.building_height_estimate = rospy.get_param('/mission/building_height_estimate', 15.0)
        self.safety_margin = rospy.get_param('/safety/geofence_margin', 5.0)
        # Veri depolama
        self.detected_building_corners = []
        
        # Publishers
        self.status_pub = rospy.Publisher('/geofence_generator/status', String, queue_size=1)
        self.marker_pub = rospy.Publisher('/geofence/visualization', MarkerArray, queue_size=1)
        
        # Subscribers
        self.building_corner_sub = rospy.Subscriber('/building_detector/corners', PointStamped, self.corner_callback)
        self.command_sub = rospy.Subscriber('/mission/command', String, self.command_callback)
        
        self.active = False
        rospy.loginfo("Geofence Generator hazır!")
        
        
        
        ''' İyileştirme Önerisi: geofence_generator'ı building_detector'dan gelen detection_completed durum mesajını dinleyecek şekilde güncelleyebilirsiniz.

        geofence_generator'a /building_detector/status konusuna abone olan yeni bir subscriber ekleyin.

        command_callback'te rospy.sleep ve calculate_geofence çağrısını kaldırın. Sadece self.active = True kalsın.

        Yeni subscriber'ın callback fonksiyonu, "detection_completed" mesajını aldığında calculate_geofence() fonksiyonunu tetiklesin.
        
        self.detection_completed_sub = rospy.Subscriber('/building_detector/status', String, self.detection_completed_callback)

    def detection_completed_callback(self, msg):
        if msg.data == "detection_completed":
            rospy.loginfo("Bina tespiti tamamlandı, geofence oluşturuluyor...")
            self.calculate_geofence()
            '''

    def corner_callback(self, msg):
        """Tespit edilen bina köşelerini topla"""
        if self.active:
            self.detected_building_corners.append([msg.point.x, msg.point.y, msg.point.z])
            rospy.loginfo(f"Köşe eklendi: ({msg.point.x:.2f}, {msg.point.y:.2f})")
            
    def command_callback(self, msg):
        if msg.data == "generate_geofence":
            self.active = True
            rospy.loginfo("Geofence oluşturma komutu alındı!")
            rospy.sleep(2.0)  # Tüm köşelerin gelmesini bekle
            self.calculate_geofence()
            
    def calculate_geofence(self):
        """En uzak 2 köşeyi bul ve 3D kutu oluştur"""
        if len(self.detected_building_corners) < 2:
            rospy.logerr("Yeterli köşe bulunamadı!")
            return
            
        points = np.array(self.detected_building_corners)
        
        # XY düzleminde min-max köşeleri bul
        min_x, min_y = np.min(points[:, :2], axis=0)
        max_x, max_y = np.max(points[:, :2], axis=0)
        
        # Güvenlik marjı ekle
        min_x -= self.safety_margin
        min_y -= self.safety_margin
        max_x += self.safety_margin
        max_y += self.safety_margin
        
        # 3D kutu köşeleri (8 köşe)
        geofence_box = {
            'min_corner': [float(min_x), float(min_y), 0.0],
            'max_corner': [float(max_x), float(max_y), float(self.building_height_estimate)],
            'center': [float((min_x + max_x) / 2), float((min_y + max_y) / 2), float(self.building_height_estimate / 2)],
            'dimensions': {
                'width': float(max_x - min_x),
                'depth': float(max_y - min_y),
                'height': float(self.building_height_estimate)
            },
            'safe_buildings': []
        }
        
        # Sağlam binaların merkez noktalarını ekle
        for i in range(0, len(points), 2):  # Her 2 köşe bir bina
            if i + 1 < len(points):
                center_x = (points[i][0] + points[i+1][0]) / 2
                center_y = (points[i][1] + points[i+1][1]) / 2
                geofence_box['safe_buildings'].append({
                    'id': i // 2,
                    'center': [float(center_x), float(center_y), 0.0],
                    'status': 'intact'
                })
        
        # YAML dosyasına kaydet
        with open(self.output_file, 'w') as f:
            yaml.dump(geofence_box, f)
            
        rospy.loginfo(f"Geofence başarıyla oluşturuldu ve {self.output_file} dosyasına kaydedildi.")
        rospy.loginfo(f"Geofence boyutları: {geofence_box['dimensions']['width']:.2f}m x {geofence_box['dimensions']['depth']:.2f}m x {geofence_box['dimensions']['height']:.2f}m")
        rospy.loginfo(f"Tespit edilen sağlam bina sayısı: {len(geofence_box['safe_buildings'])}")
        
        # Görselleştirme için marker yayınla
        self.publish_geofence_marker(geofence_box)
        
        # Durumu bildir
        self.status_pub.publish("geofence_generated")
        self.active = False
        
    def publish_geofence_marker(self, geofence):
        """RViz'de geofence görselleştirmesi"""
        markers = MarkerArray()
        
        # Geofence kutusu
        marker = Marker()
        marker.header.frame_id = "world"
        marker.header.stamp = rospy.Time.now()
        marker.ns = "geofence"
        marker.id = 0
        marker.type = Marker.CUBE
        marker.action = Marker.ADD
        
        marker.pose.position.x = geofence['center'][0]
        marker.pose.position.y = geofence['center'][1]
        marker.pose.position.z = geofence['center'][2]
        marker.pose.orientation.w = 1.0
        
        marker.scale.x = geofence['dimensions']['width']
        marker.scale.y = geofence['dimensions']['depth']
        marker.scale.z = geofence['dimensions']['height']
        
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.color.a = 0.3
        
        markers.markers.append(marker)
        
        # Sağlam binalar
        for i, building in enumerate(geofence['safe_buildings']):
            marker = Marker()
            marker.header.frame_id = "world"
            marker.header.stamp = rospy.Time.now()
            marker.ns = "buildings"
            marker.id = i + 1
            marker.type = Marker.CYLINDER
            marker.action = Marker.ADD
            
            marker.pose.position.x = building['center'][0]
            marker.pose.position.y = building['center'][1]
            marker.pose.position.z = self.building_height_estimate / 2
            marker.pose.orientation.w = 1.0
            
            marker.scale.x = 10.0
            marker.scale.y = 10.0
            marker.scale.z = self.building_height_estimate
            
            marker.color.r = 0.0
            marker.color.g = 0.0
            marker.color.b = 1.0
            marker.color.a = 0.5
            
            markers.markers.append(marker)
            
        self.marker_pub.publish(markers)

if __name__ == '__main__':
    try:
        generator = GeofenceGenerator()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("Geofence Generator kapatılıyor...")