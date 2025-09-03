#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from trajectory_msgs.msg import MultiDOFJointTrajectory, MultiDOFJointTrajectoryPoint
from geometry_msgs.msg import Transform, Twist
import tf
import subprocess

# --- AYARLAR ---
HEDEF_IRTIFA = 30.0  # Dronun çıkmasını istediğin yükseklik (metre)
HEDEF_X = 0.0        # Sabit kalacağı X konumu
HEDEF_Y = 0.0        # Sabit kalacağı Y konumu

def main():
    # Düğümü (node) başlat
    rospy.init_node('basit_yukselme_testi', anonymous=True)
    rospy.loginfo("Basit yükselme testi başlatılıyor...")

    # ÖNEMLİ: Simülasyonla birlikte gelen ve kontrolü engelleyen
    # 'hovering_example' düğümünü kapatıyoruz.
    # Çalışan kodunda da bu olduğu için ekledim.
    try:
        rospy.loginfo("Mevcut hovering_example düğümü kapatılıyor...")
        subprocess.call(['rosnode', 'kill', '/firefly/hovering_example'])
        rospy.sleep(1) # Kapanması için kısa bir süre bekle
    except Exception as e:
        rospy.logwarn(f"hovering_example kapatılamadı, muhtemelen zaten çalışmıyordu: {e}")

    # Komutları yayınlayacağımız topic'i tanımla
    trajectory_pub = rospy.Publisher('/firefly/command/trajectory', 
                                      MultiDOFJointTrajectory, 
                                      queue_size=10)

    # ROS'un yayıncıyı hazırlaması için kısa bir bekleme süresi
    rospy.sleep(0.5)

    # Göndereceğimiz komut mesajını oluşturalım.
    # Bu mesaj, dronun gitmesi gereken tek bir noktayı içeriyor.
    traj = MultiDOFJointTrajectory()
    traj.header.stamp = rospy.Time.now()
    traj.header.frame_id = "world" # Genel koordinat sistemine göre

    point = MultiDOFJointTrajectoryPoint()

    # 1. Transform (Konum ve Yönelim) Bilgisi
    transform = Transform()
    transform.translation.x = HEDEF_X
    transform.translation.y = HEDEF_Y
    transform.translation.z = HEDEF_IRTIFA

    # Yönelim (Yaw=0) için quaternion
    q = tf.transformations.quaternion_from_euler(0, 0, 0) # Roll, Pitch, Yaw
    transform.rotation.x = q[0]
    transform.rotation.y = q[1]
    transform.rotation.z = q[2]
    transform.rotation.w = q[3]

    # 2. Hız ve İvme Bilgisi (boş bırakıyoruz, sıfır kabul edilecek)
    velocities = Twist()
    accelerations = Twist()

    # Oluşturulan bilgileri ana noktaya ekle
    point.transforms.append(transform)
    point.velocities.append(velocities)
    point.accelerations.append(accelerations)

    # Noktayı da trajectory mesajına ekle
    traj.points.append(point)

    # Kontrol döngüsü: Dronun pozisyonunu sürekli olarak koruması için
    # komutu saniyede 10 kez gönderiyoruz.
    rate = rospy.Rate(10) # 10 Hz
    rospy.loginfo(f"{HEDEF_IRTIFA} metreye yükselme komutu gönderiliyor...")

    while not rospy.is_shutdown():
        # Her döngüde mesajın zaman damgasını güncellemek iyi bir pratiktir.
        traj.header.stamp = rospy.Time.now()
        
        # Mesajı yayınla
        trajectory_pub.publish(traj)
        
        # Döngü hızını ayarla
        rate.sleep()

    rospy.loginfo("Test düğümü kapatıldı.")

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass