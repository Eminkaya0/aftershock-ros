# aftershock-ros

Autonomous drone for navigating damaged buildings after earthquakes.  
**ROS1 Noetic** · mapping (SLAM), obstacle avoidance, mission logic.

## ✨ Features
- SLAM: (e.g., Hector SLAM / ORB-SLAM2 / RTAB-Map)
- Obstacle avoidance: lidar/depth + local planner
- Autonomy: mission nodes (room sweep, corridor follow)
- Simulation: Gazebo + PX4/ArduPilot SITL worlds
- Real-world: MAVROS integration

## 📦 Requirements
- Ubuntu 20.04 + ROS Noetic
- `catkin` workspace
- (Opsiyonel) PX4/ArduPilot SITL, Gazebo, MAVROS

## 🚀 Quick start
```bash
# 1) Workspace
mkdir -p ~/catkin_ws/src && cd ~/catkin_ws/src
git clone https://github.com/Eminkaya0/aftershock-ros.git
cd .. && rosdep install --from-paths src --ignore-src -y
catkin_make
source devel/setup.bash
