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
<br>
<br>
### Open RViz with namespace ###
ros2 run rviz2 rviz2 --ros-args -r __ns:=/leader
<br>
#### Dockerfile with ROS2 Jazzy and all dependencies run command:
<br>
docker build -t bumperbot-hailo-jazzy .
<br>
docker run -it --privileged \
  --net=host \
  --name bumperbot_ai_container \
  --device /dev/hailo0:/dev/hailo0 \
  --device /dev/myserial:/dev/myserial \
  --device /dev/ydlidar:/dev/ydlidar \
  --device /dev/i2c-1:/dev/i2c-1 \
  --device /dev/snd:/dev/snd \
  --device /dev/video0:/dev/video0 \
  -v /sys:/sys \
  -v /dev:/dev \
  -v /usr/share/hailo-models:/ros2_ws/models \
  -v /home/hakan/ros2_hailo_project/ydlidar/ydlidar_ros2_driver:/ros2_ws/src/ydlidar_ros2_driver \
  -v /home/hakan/bumperbot_ws/src:/ros2_ws/src/bumperbot_project \
  -v /home/hakan/ros2_hailo_project/bitmap-fonts:/fonts \
  bumperbot-hailo-jazzy
<br>
<br>
docker start bumperbot_ai_container
<br>
docker exec -it bumperbot_ai_container bash