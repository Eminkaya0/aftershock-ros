#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rospy
import smach
import smach_ros
from std_msgs.msg import String

# =============================================================================
# TEKRAR KULLANILABİLİR, GENEL BİR GÖREV DURUMU (STATE)
# =============================================================================
class CommandState(smach.State):
    """
    Bu genel Durum (State), bir komut yayınlar ve belirli bir status
    konusundan bir başarı mesajı bekler. Bu, görevdeki tüm adımlar için
    tekrar tekrar kullanılabilen bir yapıdır.
    """
    def __init__(self, command, status_topic, success_msg, timeout_duration=60.0):
        # Bu durumun olası sonuçlarını tanımla: 'succeeded' veya 'aborted'
        smach.State.__init__(self, outcomes=['succeeded', 'aborted'])
        
        # Parametreleri sakla
        self.command_to_send = String(data=command)
        self.status_topic = status_topic
        self.expected_success_msg = success_msg
        self.timeout = rospy.Duration(timeout_duration)
        
        # Gerekli Publisher ve Subscriber'lar
        self.command_pub = rospy.Publisher('/mission/command', String, queue_size=1)
        self.status_sub = rospy.Subscriber(status_topic, String, self.status_callback)
        
        # Gelen durum mesajını saklamak için değişken
        self.received_status = None

    def status_callback(self, msg):

        """İlgili durum konusundan gelen mesajları dinler."""
        rospy.loginfo(f"'{self.status_topic}' konusundan durum alındı: '{msg.data}'")
        self.received_status = msg.data

    def execute(self, userdata):
        """
        Bu durum aktif olduğunda çalışan ana fonksiyon.
        """

        rospy.loginfo(f"Executing State: '{self.command_to_send.data}' komutu gönderiliyor...")
        rospy.loginfo(f" -> '{self.status_topic}' konusundan '{self.expected_success_msg}' mesajı bekleniyor...")

        # Başlamadan önce durumu sıfırla
        self.received_status = None
        
        # Diğer düğümün subscriber'ı aktif hale getirmesi için kısa bir bekleme
        rospy.sleep(1.0)
        
        # Komutu yayınla
        self.command_pub.publish(self.command_to_send)
        
        # Zaman aşımı için başlangıç zamanını kaydet
        start_time = rospy.Time.now()
        
        # Başarı mesajı gelene veya zaman aşımı olana kadar bekle
        while not rospy.is_shutdown():
            # Zaman aşımını kontrol et
            if rospy.Time.now() - start_time > self.timeout:
                rospy.logerr(f"HATA: Durum '{self.command_to_send.data}' zaman aşımına uğradı! ({self.timeout.to_sec()} saniye)")
                return 'aborted'
            
            # Başarı mesajı geldi mi kontrol et
            if self.received_status == self.expected_success_msg:
                rospy.loginfo(f"BAŞARILI: Beklenen durum '{self.expected_success_msg}' alındı.")
                return 'succeeded'
            
            rospy.sleep(0.5) # CPU'yu yormamak için kısa bekleme

        # Eğer ROS kapatılırsa, görevi iptal et
        return 'aborted'


# =============================================================================
# ANA GÖREV AKIŞI
# =============================================================================
def main():
    rospy.init_node('mission_state_machine')

    # Görevdeki tüm adımları içerecek olan ana Durum Makinesi'ni oluştur.
    # Bu makinenin nihai sonuçları 'MISSION_SUCCESS' veya 'MISSION_FAILURE' olacak.
    mission_sm = smach.StateMachine(outcomes=['MISSION_SUCCESS', 'MISSION_FAILURE'])

    # Durum Makinesi'nin içini doldurmaya başla
    with mission_sm:
        # ---------------------------------------------------------------------
        # DURUM 1: YÜKSELME VE TARAMA
        # ---------------------------------------------------------------------
        smach.StateMachine.add('ASCEND_AND_SCAN', 
                               CommandState(command='start_ascent',
                                            status_topic='/altitude_scanner/status',
                                            success_msg='scan_completed',
                                            timeout_duration=120.0), # Yükselme uzun sürebilir
                               transitions={'succeeded':'DETECT_BUILDINGS', 
                                            'aborted':'MISSION_FAILURE'})
        
        # ---------------------------------------------------------------------
        # DURUM 2: BİNA TESPİTİ
        # ---------------------------------------------------------------------
        smach.StateMachine.add('DETECT_BUILDINGS', 
                               CommandState(command='start_detection',
                                            status_topic='/building_detector/status',
                                            success_msg='detection_completed',
                                            timeout_duration=60.0),
                               transitions={'succeeded':'GENERATE_GEOFENCE', 
                                            'aborted':'MISSION_FAILURE'})

        # ---------------------------------------------------------------------
        # DURUM 3: GEOFENCE OLUŞTURMA
        # ---------------------------------------------------------------------
        smach.StateMachine.add('GENERATE_GEOFENCE', 
                               CommandState(command='generate_geofence',
                                            status_topic='/geofence_generator/status',
                                            success_msg='geofence_generated',
                                            timeout_duration=30.0),
                               transitions={'succeeded':'NAVIGATE_BUILDINGS', 
                                            'aborted':'MISSION_FAILURE'})
        
        # ---------------------------------------------------------------------
        # DURUM 4: BİNA NAVİGASYONU
        # ---------------------------------------------------------------------
        smach.StateMachine.add('NAVIGATE_BUILDINGS', 
                               CommandState(command='start_navigation',
                                            status_topic='/building_navigator/status',
                                            success_msg='navigation_completed',
                                            timeout_duration=300.0), # Navigasyon en uzun sürecek adım
                               transitions={'succeeded':'MISSION_SUCCESS', 
                                            'aborted':'MISSION_FAILURE'})

    # SMACH durumlarını RViz gibi araçlarda görselleştirmek için bir sunucu oluştur.
    introspection_server = smach_ros.IntrospectionServer('mission_server', mission_sm, '/SM_ROOT')
    introspection_server.start()

    # Durum makinesini çalıştır. Bu fonksiyon, makine bir sonuca ulaşana kadar bloke olur.
    rospy.loginfo("Durum Makinesi başlatılıyor...")
    outcome = mission_sm.execute()
    rospy.loginfo(f"Görev tamamlandı. Nihai sonuç: {outcome}")

    # Sunucuyu durdur
    introspection_server.stop()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        rospy.logwarn("Görev döngüsü manuel olarak kesildi.")