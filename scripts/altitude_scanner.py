#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from nav_msgs.msg import Odometry
from trajectory_msgs.msg import MultiDOFJointTrajectory, MultiDOFJointTrajectoryPoint
from geometry_msgs.msg import Transform, Twist
from std_msgs.msg import String
from sensor_msgs.msg import Image
import math
import cv2
from cv_bridge import CvBridge, CvBridgeError
import subprocess

class AltitudeScanner:
    def __init__(self):
        rospy.init_node('altitude_scanner', anonymous=True)
        rospy.loginfo("Altitude Scanner başlatılıyor...")

        # ÖNEMLİ: Hovering example'ı durdur - flip atmamak için!
        print("Hovering example durduruluyor...")
        try:
            subprocess.call(['rosnode', 'kill', '/firefly/hovering_example'])
        except:
            pass
        
        rospy.sleep(3)  # Sistem stable olsun diye bekle

        self.bridge = CvBridge()

        # BASİT PARAMETRELER - YAML YOK!
        self.target_altitude = 80.0  # 80 metre hedef
        self.altitude_tolerance = 2.0  # 2 metre tolerance
        self.scan_duration = 10.0  # 10 saniye tarama
        self.hover_duration = 3.0  # 3 saniye hover

        # Görüntü işleme parametreleri
        self.image_topic = '/firefly/new_extra_camera/image_raw'
        self.image_encoding = 'rgb8'
        self.sharpness_threshold = 20.0
        self.max_images = 5

        rospy.loginfo(f"Parametreler yüklendi:")
        rospy.loginfo(f"  - Hedef yükseklik: {self.target_altitude}m")
        rospy.loginfo(f"  - Tarama süresi: {self.scan_duration}s")
        rospy.loginfo(f"  - Kamera topic: {self.image_topic}")

        # Durum değişkenleri
        self.current_pose = None
        self.start_x = 0.0
        self.start_y = 0.0
        self.start_yaw = 0.0
        self.position_locked = False
        
        self.altitude_reached = False
        self.scan_started = False
        self.scan_start_time = None
        self.captured_images = []
        
        # Publishers
        self.trajectory_pub = rospy.Publisher('/firefly/command/trajectory', 
                                            MultiDOFJointTrajectory, queue_size=10)
        self.status_pub = rospy.Publisher('/altitude_scanner/status', String, queue_size=1)
        self.captured_images_pub = rospy.Publisher('/captured_images', Image, queue_size=10)
        
        # Subscribers
        self.odom_sub = rospy.Subscriber('/firefly/ground_truth/odometry', 
                                       Odometry, self.odom_callback)
        self.command_sub = rospy.Subscriber('/mission/command', String, 
                                          self.command_callback, queue_size=1)
        
        # Kamera subscriber'ını güvenli başlat
        self.image_sub = None
        self.setup_camera()
        
        self.active = False
        self.start_command_received = False
        
        rospy.loginfo("Altitude Scanner hazır! Mission coordinator'dan komut bekleniyor...")

    def setup_camera(self):
        """Kamera topic'ini güvenli şekilde setup et"""
        try:
            rospy.loginfo(f"Kamera topic'ine bağlanmaya çalışılıyor: {self.image_topic}")
            # Önce topic'in var olup olmadığını kontrol et
            rospy.wait_for_message(self.image_topic, Image, timeout=5.0)
            self.image_sub = rospy.Subscriber(self.image_topic, Image, self.image_callback, queue_size=1)
            rospy.loginfo("✅ Kamera topic'ine başarıyla bağlanıldı!")
        except rospy.ROSException:
            rospy.logwarn(f"⚠️ Kamera topic'i bulunamadı: {self.image_topic}")
            rospy.logwarn("Sadece pozisyon kontrolü yapılacak, görüntü alınmayacak.")
            self.image_sub = None

    def odom_callback(self, msg):
        self.current_pose = msg.pose.pose
        
    def command_callback(self, msg):
        """Mission coordinator'dan gelen komutları işle"""
        rospy.loginfo(f"🔔 Komut alındı: '{msg.data}' (active: {self.active}, start_command_received: {self.start_command_received})")
        
        if msg.data == "start_ascent" and not self.start_command_received:
            rospy.loginfo("✅ ALTITUDE SCAN komutu alındı!")
            self.start_command_received = True
        elif msg.data == "start_scan":
            rospy.loginfo("✅ START SCAN komutu alındı! Yeni bir görev başlatılıyor...")
            # Yeni görev için reset
            self.active = True
            self.start_command_received = True
            self.altitude_reached = False
            self.scan_started = False
            self.captured_images.clear()
        elif msg.data == "abort_mission":
            rospy.logwarn("⚠️ Görev iptal komutu alındı!")
            self.active = False
            self.start_command_received = False

    def image_callback(self, msg):
        """Görüntü yakalama callback'i"""
        #rospy.loginfo(f"🔍 IMAGE_CALLBACK: scan_started={self.scan_started}, captured={len(self.captured_images)}/{self.max_images}")
        if not self.scan_started or len(self.captured_images) >= self.max_images:
        #    rospy.loginfo(f"⚠️ IMAGE_CALLBACK SKIPPED: scan_started={self.scan_started}, captured={len(self.captured_images)}")
            return

        try:
            # Görüntüyü işle
            if self.image_encoding == 'passthrough':
                cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
            else:
                cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding=self.image_encoding)

            # Görüntü kalitesini kontrol et
            if cv_image.ndim == 2:
                gray = cv_image.astype('uint8')
            else:
                gray = cv2.cvtColor(cv_image, cv2.COLOR_RGB2GRAY)

            # Laplacian variance ile keskinlik ölç
            laplacian_var = cv2.Laplacian(gray, cv2.CV_64F).var()

            if laplacian_var < self.sharpness_threshold:
                rospy.logwarn(f"Bulanık görüntü atlandı (keskinlik: {laplacian_var:.2f})")
                return

            # Kaliteli görüntüyü kaydet
            self.captured_images.append(msg)
            current_count = len(self.captured_images)
            rospy.loginfo(f"📸 Görüntü yakalandı ({current_count}/{self.max_images}) - keskinlik: {laplacian_var:.2f}")
            
            # Maksimum sayıya ulaşıldı mı debug
            if current_count >= self.max_images:
                rospy.loginfo(f"🎯 Maksimum görüntü sayısına ulaşıldı ({current_count}/{self.max_images})")

        except CvBridgeError as e:
            rospy.logerr(f"Görüntü işleme hatası: {e}")

    def send_trajectory_command(self, x, y, z, yaw=0):
        """Trajectory komutu gönder - çalışan format!"""
        traj = MultiDOFJointTrajectory()
        traj.header.stamp = rospy.Time.now()
        traj.header.frame_id = "world"
        
        point = MultiDOFJointTrajectoryPoint()
        
        transform = Transform()
        transform.translation.x = x
        transform.translation.y = y  
        transform.translation.z = z
        
        # Senin çalışan kodundaki gibi basit quaternion
        transform.rotation.w = math.cos(yaw/2)
        transform.rotation.z = math.sin(yaw/2)
        transform.rotation.x = 0.0
        transform.rotation.y = 0.0
        
        point.transforms.append(transform)
        point.velocities.append(Twist())
        point.accelerations.append(Twist())
        
        traj.points.append(point)
        self.trajectory_pub.publish(traj)
        
        rospy.logdebug(f"Komut gönderildi: x={x:.1f}, y={y:.1f}, z={z:.1f}")

    def get_current_yaw(self):
        """Mevcut yaw açısını hesapla"""
        if self.current_pose is None:
            return 0.0
            
        q = self.current_pose.orientation
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        return yaw

    def publish_captured_images(self):
        """Yakalanan görüntüleri yayınla"""
        image_count = len(self.captured_images)
        rospy.loginfo(f"📤 {image_count} görüntü yayınlanıyor...")
        
        if image_count == 0:
            rospy.logwarn("⚠️ Yayınlanacak görüntü bulunamadı!")
            return
            
        for i, img_msg in enumerate(self.captured_images):
            try:
                rospy.loginfo(f"   📸 Görüntü {i+1}/{image_count} yayınlanıyor...")
                self.captured_images_pub.publish(img_msg)
                rospy.loginfo(f"   ✅ Görüntü {i+1}/{image_count} başarıyla yayınlandı")
                rospy.sleep(1.0)  # 1 saniye bekle - building detector'ın yakalayabilmesi için
            except Exception as e:
                rospy.logerr(f"   ❌ Görüntü {i+1} yayınlanırken hata: {e}")
                
        rospy.loginfo(f"✅ Toplam {image_count} görüntü yayınlandı!")

    def run(self):
        rate = rospy.Rate(20)  # 20 Hz - daha responsive
        
        rospy.loginfo("🔄 Ana döngü başlıyor...")
        
        while not rospy.is_shutdown():
            # Görev başlatma kontrolleri
            if not self.active:
                if self.start_command_received and self.current_pose is not None:
                    # Başlangıç pozisyonunu kaydet
                    self.start_x = self.current_pose.position.x
                    self.start_y = self.current_pose.position.y
                    self.start_yaw = self.get_current_yaw()
                    self.position_locked = True
                    self.active = True
                    
                    rospy.loginfo("🚀 YÜKSEKLIK TARAMASI BAŞLIYOR!")
                    rospy.loginfo(f"📍 Başlangıç: x={self.start_x:.2f}, y={self.start_y:.2f}")
                    rospy.loginfo(f"🎯 Hedef: {self.target_altitude:.1f}m yükseklik")
                    
                    # Reset states
                    self.altitude_reached = False
                    self.scan_started = False
                    self.captured_images.clear()
                else:
                    # Bekle
                    if not self.start_command_received:
                        rospy.loginfo_throttle(10, "⏳ Mission coordinator komut bekleniyor...")
                    if self.current_pose is None:
                        rospy.loginfo_throttle(10, "⏳ Odometry verisi bekleniyor...")
                    rate.sleep()
                    continue

            # Ana görev döngüsü
            if self.current_pose is None:
                rate.sleep()
                continue
                
            current_altitude = self.current_pose.position.z
            
            # YAVAŞ YAVAS YÜKSELİM - flip atmayı engelleyecek!
            if not self.altitude_reached:
                # İlk kez çalışıyorsa hedef yüksekliği ayarla
                if not hasattr(self, 'current_target_z'):
                    self.current_target_z = max(1.0, current_altitude)  # En az 1m'den başla
                    self.step_size = 3.0  # 3 metre aşamalar
                    self.last_step_time = rospy.Time.now()
                    rospy.loginfo(f"🚀 AŞAMALI YÜKSELİM BAŞLIYOR: {self.current_target_z:.1f}m → {self.target_altitude:.1f}m")
                
                # Her 2 saniyede bir aşama yükselt
                if (rospy.Time.now() - self.last_step_time).to_sec() > 2.0:
                    if self.current_target_z < self.target_altitude:
                        # Hedefe yakınsa tam hedefi ver
                        if abs(self.current_target_z - self.target_altitude) < self.step_size:
                            self.current_target_z = self.target_altitude
                        else:
                            self.current_target_z += self.step_size
                        self.last_step_time = rospy.Time.now()
                        rospy.loginfo(f"📈 AŞAMA YÜKSELTİLDİ: {self.current_target_z:.1f}m")
                
                rospy.loginfo_throttle(1, f"🔺 YÜKSELİYOR: {current_altitude:.1f}m → {self.current_target_z:.1f}m (hedef: {self.target_altitude:.1f}m)")
                
                # Mevcut hedef yüksekliğe git
                self.send_trajectory_command(self.start_x, self.start_y, self.current_target_z, self.start_yaw)
                
                # Final hedefe ulaştı mı? - DEBUG bilgisi ekleyelim
                altitude_diff = abs(current_altitude - self.target_altitude)
                rospy.loginfo_throttle(3, f"🔍 DEBUG: alt={current_altitude:.1f}, hedef={self.target_altitude:.1f}, fark={altitude_diff:.1f}, tolerans={self.altitude_tolerance:.1f}, current_target={self.current_target_z:.1f}")
                
                if altitude_diff < self.altitude_tolerance and self.current_target_z >= self.target_altitude:
                    self.altitude_reached = True
                    self.status_pub.publish("altitude_reached")
                    rospy.loginfo(f"✅ HEDEF YÜKSEKLİĞE ULAŞILDI: {current_altitude:.2f}m")
                    rospy.loginfo("📡 'altitude_reached' mesajı gönderildi!")
                    
                    # Stabilize ol
                    rospy.loginfo(f"⏸️ {self.hover_duration}s stabilizasyon...")
                    hover_start = rospy.Time.now()
                    while (rospy.Time.now() - hover_start).to_sec() < self.hover_duration:
                        if rospy.is_shutdown() or not self.active:
                            break
                        self.send_trajectory_command(self.start_x, self.start_y, self.target_altitude, self.start_yaw)
                        rospy.sleep(0.1)
                        
            # Tarama başlatma
            elif not self.scan_started:
                self.scan_started = True
                self.scan_start_time = rospy.Time.now()
                rospy.loginfo(f"📸 GÖRÜNTÜ TARAMASI BAŞLADI! ({self.scan_duration}s)")
                self.status_pub.publish("scan_started")
                
            # Tarama aşaması
            else:
                # Pozisyonu koru
                self.send_trajectory_command(self.start_x, self.start_y, self.target_altitude, self.start_yaw)
                
                scan_elapsed = (rospy.Time.now() - self.scan_start_time).to_sec()
                rospy.loginfo_throttle(2, f"📸 Tarama: {scan_elapsed:.1f}s / {self.scan_duration}s - Görüntü: {len(self.captured_images)}/{self.max_images}")
                
                # Tarama süresi doldu mu?
                if scan_elapsed > self.scan_duration:
                    image_count = len(self.captured_images)
                    rospy.loginfo(f"✅ TARAMA TAMAMLANDI! Yakalanan görüntü sayısı: {image_count}")
                    self.status_pub.publish("scan_completed")
                    
                    # Yakalanan görüntüleri yayınla
                    if self.captured_images:
                        rospy.loginfo(f"🚀 {image_count} görüntü publish edilecek...")
                        # --- YENİ EKLENEN KISIM ---
                        rospy.loginfo("Building detector'ın hazırlanması için 2 saniye bekleniyor...")
                        rospy.sleep(2.0)  # 2 saniye bekle
                        # --- BİTTİ --- 
                        self.publish_captured_images()
                    else:
                        rospy.logwarn("⚠️ Hiç görüntü yakalanamadı!")
                    
                    # Mission coordinator'a bildir
                    self.status_pub.publish("scan_completed")
                    rospy.loginfo("📋 Mission coordinator'a görev tamamlandı bildirimi gönderildi.")
                    
                    # Görev tamamlandı - bekleme moduna geç
                    self.active = False
                    self.start_command_received = False
                    rospy.loginfo("🎉 YÜKSEKLİK TARAMASI TAMAMLANDI! Yeni görev bekleniyor...")
                    
            rate.sleep()

if __name__ == '__main__':
    try:
        scanner = AltitudeScanner()
        scanner.run()
    except rospy.ROSInterruptException:
        rospy.loginfo("Altitude Scanner kapatılıyor...")