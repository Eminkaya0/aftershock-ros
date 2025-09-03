#!/usr/bin/env python3
import rospy

if __name__ == '__main__':
    rospy.init_node('test_node')
    rospy.loginfo("ROS Python ortamı sağlıklı bir şekilde çalışıyor!")
    rospy.sleep(1)
