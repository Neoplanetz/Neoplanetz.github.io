# ROS 2 Foxy All in One 파일 하나로 설치

## 1. ROS 2 Foxy Installation Bash Shell 파일 다운로드
- ROS 2 Foxy를 한번에 All in one 파일로 설치하기 위해 Shell 파일을 다운로드
  ```bash
  $ wget http://raw.githubusercontent.com/neoplanetz/Neoplanetz.github.io/main/install_ros_foxy.sh  
  ```

## 2. 파일 권한 설정 및 ROS 2 Foxy 한방 설치
- 다운받은 파일에 읽기 권한을 부여하고 설치 shell 파일 실행
  ```bash
  $ chmod 755 ./install_ros_foxy.sh && bash ./install_ros_foxy.sh
  ```

## 3. ROS 2 Foxy All in One 파일에 포함된 패키지 및 어플리케이션
- ROS 2 Foxy Desktop
- Gazebo 11 Simulator for ROS 2
- Visual Studio Code