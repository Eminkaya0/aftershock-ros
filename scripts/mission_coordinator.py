#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from std_msgs.msg import String, Bool
from geometry_msgs.msg import Point
import yaml

class MissionCoordinator:
    def __init__(self):
        rospy.init_node('mission_coordinator', anonymous=True)
        rospy.loginfo("Görev Koordinatörü başlatılıyor...")
        
        # FSM Durumları
        self.states = ["IDLE", "ASCENDING", "SCANNING", "DETECTING", "GEOFENCE_GEN", "NAVIGATING", "COMPLETED"]
        self.current_state = "IDLE"
        
        # Publishers
        self.state_pub = rospy.Publisher('/mission/state', String, queue_size=1)
        self.command_pub = rospy.Publisher('/mission/command', String, queue_size=1, latch=True)
        
        # Subscribers
        self.altitude_status_sub = rospy.Subscriber('/altitude_scanner/status', String, self.altitude_status_cb)
        self.detection_status_sub = rospy.Subscriber('/building_detector/status', String, self.detection_status_cb)
        self.geofence_status_sub = rospy.Subscriber('/geofence_generator/status', String, self.geofence_status_cb)
        self.navigation_status_sub = rospy.Subscriber('/building_navigator/status', String, self.navigation_status_cb)
        
        # Durum geçiş bayrakları
        self.altitude_reached = False
        self.scan_completed = False
        self.detection_completed = False
        self.geofence_generated = False
        self.navigation_completed = False
        
        rospy.loginfo("Görev Koordinatörü hazır!")
        
    def altitude_status_cb(self, msg):
        if msg.data == "altitude_reached":
            self.altitude_reached = True
        elif msg.data == "scan_completed":
            self.scan_completed = True
            
    def detection_status_cb(self, msg):
        if msg.data == "detection_completed":
            self.detection_completed = True
            
    def geofence_status_cb(self, msg):
        if msg.data == "geofence_generated":
            self.geofence_generated = True
            
    def navigation_status_cb(self, msg):
        if msg.data == "navigation_completed":
            self.navigation_completed = True
            
    def run_state_machine(self):
        rate = rospy.Rate(1)  # 1 Hz
        
        while not rospy.is_shutdown():
            # Mevcut durumu yayınla
            self.state_pub.publish(self.current_state)
            
            # State Machine Logic
            if self.current_state == "IDLE":
                rospy.loginfo("Görev başlatılıyor...")
                self.command_pub.publish("start_ascent")
                self.current_state = "ASCENDING"
                
            elif self.current_state == "ASCENDING":
                if self.altitude_reached:
                    rospy.loginfo("80 metreye ulaşıldı. Tarama başlıyor...")
                    self.command_pub.publish("start_scan")
                    self.current_state = "SCANNING"
                    
            elif self.current_state == "SCANNING":
                if self.scan_completed:
                    rospy.loginfo("Tarama tamamlandı. Bina tespiti başlıyor...")
                    self.command_pub.publish("start_detection")
                    self.current_state = "DETECTING"
                    
            elif self.current_state == "DETECTING":
                if self.detection_completed:
                    rospy.loginfo("Binalar tespit edildi. Geofence oluşturuluyor...")
                    self.command_pub.publish("generate_geofence")
                    self.current_state = "GEOFENCE_GEN"
                    
            elif self.current_state == "GEOFENCE_GEN":
                if self.geofence_generated:
                    rospy.loginfo("Geofence oluşturuldu. Navigasyon başlıyor...")
                    self.command_pub.publish("start_navigation")
                    self.current_state = "NAVIGATING"
                    
            elif self.current_state == "NAVIGATING":
                if self.navigation_completed:
                    rospy.loginfo("Görev tamamlandı!")
                    self.current_state = "COMPLETED"
                    
            elif self.current_state == "COMPLETED":
                rospy.loginfo_throttle(10, "Görev başarıyla tamamlandı. Sonuçlar kaydedildi.")
                
            rate.sleep()

if __name__ == '__main__':
    try:
        coordinator = MissionCoordinator()
        coordinator.run_state_machine()
    except rospy.ROSInterruptException:
        rospy.loginfo("Görev Koordinatörü kapatılıyor...")