# Autonomous Robot Project: ROS 2 Jazzy and Nav2 Implementation
...with Ackermann kinematics, SLAM and Nav2 integration for urban environments.
<br>

<p align="center">
  <img src="https://github.com/HakanAkkurt/ROS2Robot/blob/master/Screenshots/ROS2RobotScreenshot1.png" width="1800" title="ROSMASTER A1 Simulation">
</p>

<p align="center">
  <img src="https://github.com/HakanAkkurt/ROS2Robot/blob/master/Screenshots/ROS2RobotScreenshot2.jpg" width="1800" title="ROSMASTER A1">
</p>

<p align="center">
  <img src="https://github.com/HakanAkkurt/ROS2Robot/blob/master/Screenshots/ROS2RobotScreenshot3.png" width="1800" title="Nav2 Navigation">
</p>

<p align="center">
  <img src="https://github.com/HakanAkkurt/ROS2Robot/blob/master/Screenshots/ROS2RobotScreenshot4.png" width="1800" title="Bumperbot">
</p>

Installation steps:

git clone https://github.com/HakanAkkurt/ROS2Robot.git
<br>
cd "workspace"
<br>
sudo rosdep init
<br>
rosdep update
<br>
rosdep install --from-paths src --ignore-src -r -y
<br>
sudo apt install libserial-dev
<br>
sudo apt install ros-jazzy-navigation2 ros-jazzy-nav2-bringup