#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from sensor_msgs.msg import Image, CameraInfo
from nav_msgs.msg import Odometry # YENİ: Dronun pozisyonu için
from std_msgs.msg import String
from geometry_msgs.msg import Point, PointStamped # YENİ: 3D nokta yayınlamak için
import cv2
from ultralytics import YOLO
import os
from cv_bridge import CvBridge, CvBridgeError
import numpy as np # YENİ: Matematiksel hesaplamalar için
import tf.transformations as tft # YENİ: Rotasyon işlemleri için

class BuildingDetector:
    def __init__(self):
        rospy.init_node('building_detector', anonymous=True)
        rospy.loginfo("Bina Tespit Düğümü başlatılıyor...")

        # Modelinizi yükleyin
        try:
            model_path = '/home/emin/catkin_ws/src/enkaz_rescue/models/best.pt'
            self.model = YOLO(model_path)
            rospy.loginfo(f"YOLO modeli '{model_path}' adresinden başarıyla yüklendi.")
        except Exception as e:
            rospy.logerr(f"Model yüklenirken kritik hata: {e}")
            return

        self.bridge = CvBridge()
        
        self.received_images = []
        self.images_to_process = 5
        
        # --- YENİ: Projeksiyon için gerekli veriler ---
        self.camera_intrinsics = None
        self.latest_odom = None
        
        # Kamera bilgilerini almak için bekle. Bu genellikle tek sefer yayınlanır.
        try:
            rospy.loginfo("Kamera bilgisi bekleniyor...")
            camera_info_msg = rospy.wait_for_message('/firefly/new_extra_camera/camera_info', CameraInfo, timeout=10)
            # Kamera matrisini (K) alıyoruz. [fx, 0, cx, 0, fy, cy, 0, 0, 1]
            self.camera_intrinsics = np.array(camera_info_msg.K).reshape(3, 3)
            rospy.loginfo("Kamera bilgisi başarıyla alındı.")
        except rospy.ROSException as e:
            rospy.logerr(f"Kamera bilgisi alınamadı: {e}")
            return

        # --- YENİ ve GÜNCELLENMİŞ PUBLISHER/SUBSCRIBER'lar ---
        self.status_pub = rospy.Publisher('/building_detector/status', String, queue_size=1)
        # YENİ: GeofenceGenerator'ın beklediği 3D köşe noktalarını yayınlamak için
        self.corner_pub = rospy.Publisher('/building_detector/corners', PointStamped, queue_size=20)
        
        self.command_sub = rospy.Subscriber('/mission/command', String, self.command_callback)
        # YENİ: Dronun anlık pozisyonunu ve yönelimini almak için
        self.odom_sub = rospy.Subscriber('/firefly/ground_truth/odometry', Odometry, self.odom_callback)
        
        # Subscriber'ı baştan oluştur - active flag ile kontrol et
        self.image_sub = rospy.Subscriber('/captured_images', Image, self.image_callback, queue_size=50)
        self.active = False
        
        rospy.loginfo("Bina Tespit Düğümü hazır. Komut bekleniyor...")

    def odom_callback(self, msg):
        # YENİ: Dronun en son pozisyonunu sakla
        self.latest_odom = msg

    def command_callback(self, msg):
        if msg.data == "start_detection" and not self.active:
            self.active = True
            self.received_images = []  # Listeyi temizle
            rospy.loginfo("Tespit komutu alındı. Görüntüler dinleniyor...")

    # YENİ: Projeksiyonu yapan ana fonksiyon
    def project_pixel_to_world(self, u, v):
        """Verilen piksel koordinatını (u,v) yer düzlemindeki (z=0) 3D dünya koordinatına dönüştürür."""
        if self.camera_intrinsics is None or self.latest_odom is None:
            rospy.logwarn("Projeksiyon için kamera bilgisi veya odometri verisi eksik.")
            return None

        # Adım 1: Piksel koordinatlarını normalize edilmiş kamera koordinatlarına çevir
        fx = self.camera_intrinsics[0, 0]
        fy = self.camera_intrinsics[1, 1]
        cx = self.camera_intrinsics[0, 2]
        cy = self.camera_intrinsics[1, 2]
        
        x_norm = (u - cx) / fx
        y_norm = (v - cy) / fy
        
        # Adım 2: Kamera koordinat sisteminde bir ışın (ray) oluştur
        ray_c = np.array([x_norm, y_norm, 1.0])
        
        # Adım 3: Dronun pozisyonunu ve yönelimini al
        pose = self.latest_odom.pose.pose
        pos_w = np.array([pose.position.x, pose.position.y, pose.position.z])
        quat_w = [pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w]
        
        # Adım 4: Işını kamera sisteminden dünya sistemine döndür (rotate)
        rot_matrix = tft.quaternion_matrix(quat_w)[:3, :3]
        ray_w = rot_matrix.dot(ray_c)
        
        # Adım 5: Işının yer düzlemiyle (z=0) kesişimini bul
        # Eğer ışın yere paralelse veya yukarı bakıyorsa kesişim olmaz
        if ray_w[2] >= -1e-6: # Çok küçük bir toleransla kontrol
            return None
            
        t = -pos_w[2] / ray_w[2]
        intersection_point = pos_w + t * ray_w
        
        return intersection_point


    def image_callback(self, msg):
        # Debug için hemen başta log
        rospy.loginfo(f"📥 Image callback çağrıldı - Active: {self.active}, Mevcut: {len(self.received_images)}/{self.images_to_process}")
        
        # Active kontrolü
        if not self.active:
            rospy.loginfo("⚠️ Detector aktif değil, görüntü atlandı")
            return
            
        # Maksimum görüntü kontrolü
        if len(self.received_images) >= self.images_to_process:
            rospy.loginfo(f"⚠️ Maksimum görüntü sayısına ulaşıldı: {len(self.received_images)}/{self.images_to_process}")
            return

        self.received_images.append(msg)
        rospy.loginfo(f"Görüntü alındı ({len(self.received_images)}/{self.images_to_process}). İşleniyor...")
        
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            results = self.model(cv_image)
            
            # YOLOv8 yeni API kullan
            for result in results:
                boxes = result.boxes
                if boxes is not None:
                    for box in boxes:
                        # Confidence ve class kontrolü
                        conf = float(box.conf[0])
                        cls = int(box.cls[0])
                        
                        # Class name'i al (model.names[cls] ile)
                        class_name = self.model.names[cls]
                        
                        if class_name == 'bina' and conf > 0.6:
                            # Bounding box koordinatları
                            x1, y1, x2, y2 = box.xyxy[0].cpu().numpy()
                            xmin, ymin, xmax, ymax = int(x1), int(y1), int(x2), int(y2)
                            
                            bottom_left_pixel = (xmin, ymax)
                            bottom_right_pixel = (xmax, ymax)
                            
                            # Bu iki köşeyi 3D dünyaya projekte et
                            world_corner1 = self.project_pixel_to_world(bottom_left_pixel[0], bottom_left_pixel[1])
                            world_corner2 = self.project_pixel_to_world(bottom_right_pixel[0], bottom_right_pixel[1])
                            
                            # Projeksiyon başarılıysa yayınla
                            if world_corner1 is not None and world_corner2 is not None:
                                # Köşe 1'i yayınla
                                ps1 = PointStamped()
                                ps1.header.stamp = rospy.Time.now()
                                ps1.header.frame_id = "world"
                                ps1.point = Point(world_corner1[0], world_corner1[1], world_corner1[2])
                                self.corner_pub.publish(ps1)
                                
                                # Köşe 2'yi yayınla
                                ps2 = PointStamped()
                                ps2.header.stamp = rospy.Time.now()
                                ps2.header.frame_id = "world"
                                ps2.point = Point(world_corner2[0], world_corner2[1], world_corner2[2])
                                self.corner_pub.publish(ps2)
                                
                                rospy.loginfo(f"Bina köşeleri yayınlandı: K1({world_corner1[0]:.2f}, {world_corner1[1]:.2f}), K2({world_corner2[0]:.2f}, {world_corner2[1]:.2f})")


##########################################################################################
# DAHA GELİŞMİTİRMEK İÇİN
# 5 FOTOĞRAFTAN GELEN TESPİTLERİ KÜMELEYİP ORTALAMASINI ALABİLİRİZ.
###########################################################################################
        except CvBridgeError as e:
            rospy.logerr(e)
        
        # Tüm görüntüler işlendi mi kontrol et
        rospy.loginfo(f"🔍 İşlenen görüntü sayısı: {len(self.received_images)}/{self.images_to_process}")
        if len(self.received_images) == self.images_to_process:
            rospy.loginfo(f"✅ Tüm görüntüler işlendi ({len(self.received_images)}/{self.images_to_process}). Görev tamamlandı.")
            self.status_pub.publish("detection_completed")
            # SUBSCRIBER'I KAPATMA - 5. görüntüyü kaçırmasın
            # if self.image_sub:
            #     self.image_sub.unregister()
            #     rospy.loginfo("🔌 Image subscriber kapatıldı")
            rospy.loginfo("📌 Subscriber açık bırakıldı - gelecek görevler için hazır")
            self.active = False

if __name__ == '__main__':
    try:
        detector = BuildingDetector()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("Bina Tespit Düğümü kapatılıyor...")