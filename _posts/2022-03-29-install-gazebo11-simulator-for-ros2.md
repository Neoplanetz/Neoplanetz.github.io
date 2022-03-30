# ROS 2를 위한 Gazebo 11 Robot Simulator 설치법

## 1. Gazebo 11 Simulator 설치
- Gazebo Key 다운로드 및 Repository 추가하여 설치
  ```bash
  $ sudo sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list'
  $ wget https://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add -
  $ sudo apt update & sudo apt install gazebo11 libgazebo11-dev -y
  ```

- Gazebo ROS 패키지 설치
  ```bash
  $ sudo apt install ros-foxy-gazebo-ros-pkgs -y
  ```

## 2. Gazebo 11 Simulator 실행
  ```bash
  $ gazebo
  ```
  ![Gazebo](/assets/img/gazebo.png)
