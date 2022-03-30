# ROS 2 Foxy 설치 및 환경 설정 (Ubuntu 20.04 Focal Fossa)

## 1. ROS 2 Foxy 설치
- locale 설정
    
    ```bash    
    $ sudo apt update && sudo apt install locales
    $ sudo locale-gen en_US en_US.UTF-8
    $ sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
    $ export LANG=en_US.UTF-8
    ```
    
- ROS 2 Key 다운로드 및 ROS 2 저장소 추가
    
    ```bash
    $ sudo apt update && sudo apt install curl gnupg2 lsb-release -y
    $ sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key  -o /usr/share/keyrings/ros-archive-keyring.gpg
    $ echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
    ```
    
- ROS 2 Foxy Desktop 및 Fast RTPS, Cyclone DDS, 그리고 유용한 패키지들 설치
    
    ```bash
    $ sudo apt update
    $ sudo apt install ros-foxy-desktop ros-foxy-rmw-fastrtps* ros-foxy-rmw-cyclonedds*

    # Install additional useful packages
    $ sudo apt install ros-foxy-rqt* ros-foxy-image-view ros-foxy-navigation2 ros-foxy-nav2-bringup ros-foxy-joint-state-publisher-gui ros-foxy-xacro
    ```
    
- 설치된 ROS 2 환경 설정 등록
    
    ```bash
    $ source /opt/ros/foxy/setup.bash
    ```
    

## 2. ROS 개발 툴 패키지 설치
- 추가적으로 ROS 개발시 필요한 툴 패키지 설치
    
    ```bash
    $ sudo apt update && sudo apt install -y \
      build-essential \
      cmake \
      git \
      libbullet-dev \
      python3-colcon-common-extensions \
      python3-flake8 \
      python3-pip \
      python3-pytest-cov \
      python3-rosdep \
      python3-setuptools \
      python3-vcstool \
      wget \
      gpg
    ```
    
    ```bash
    # install some pip packages needed for testing
    $ python3 -m pip install -U \
      argcomplete \
      flake8-blind-except \
      flake8-builtins \
      flake8-class-newline \
      flake8-comprehensions \
      flake8-deprecated \
      flake8-docstrings \
      flake8-import-order \
      flake8-quotes \
      pytest-repeat \
      pytest-rerunfailures \
      pytest
    ```
    
    ```bash
    # install Fast-RTPS and Cyclone DDS dependencies
    $ sudo apt install --no-install-recommends -y \
      libasio-dev \
      libtinyxml2-dev \
      libcunit1-dev
    ```
    

## 3. 유용한 환경변수 및 alias를 bashrc 에 설정 추가하기
- Add some alias and environment path in bashrc file Bashrc 에 유용한 환경변수와 alias를 추가하여 ros 명령어들을 편하게 사용 할 수 있도록 설정
    
    ```bash
    $ sudo gedit ~/.bashrc
    ```
    
  > Bashrc File (~/.bashrc)

  ```bash
  # Add some variables in bashrc file
  # ROS 2 Setup
  source /opt/ros/foxy/setup.bash
  source ~/ros2_ws/install/local_setup.bash
  
  source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash
  source /usr/share/vcstool-completion/vcs.bash
  source /usr/share/colcon_cd/function/colcon_cd.sh
  export _colcon_cd_root=~/ros2_ws
  
  export ROS_DOMAIN_ID=7
  export ROS_NAMESPACE=robot1
  
  export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
  # export RMW_IMPLEMENTATION=rmw_connext_cpp
  # export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
  # export RMW_IMPLEMENTATION=rmw_gurumdds_cpp
  
  # export RCUTILS_CONSOLE_OUTPUT_FORMAT='[{severity} {time}] [{name}]: {message} ({function_name}() at {file_name}:{line_number})'
  export RCUTILS_CONSOLE_OUTPUT_FORMAT='[{severity}]: {message}'
  export RCUTILS_COLORIZED_OUTPUT=1
  export RCUTILS_LOGGING_USE_STDOUT=0
  export RCUTILS_LOGGING_BUFFERED_STREAM=1
  
  # Alias Setting
  alias cw='cd ~/ros2_ws'
  alias cs='cd ~/ros2_ws/src'
  alias ccd='colcon_cd'
  
  alias cb='cd ~/ros2_ws&& colcon build --symlink-install'
  alias cbs='colcon build --symlink-install'
  alias cbp='colcon build --symlink-install --packages-select'
  alias cbu='colcon build --symlink-install --packages-up-to'
  alias ct='colcon test'
  alias ctp='colcon test --packages-select'
  alias ctr='colcon test-result'
  
  alias rt='ros2 topic list'
  alias re='ros2 topic echo'
  alias rn='ros2 node list'
  
  alias killgazebo='killall -9 gazebo & killall -9 gzserver  & killall -9 gzclient'
  
  alias af='ament_flake8'
  alias ac='ament_cpplint'
  
  alias testpub='ros2 run demo_nodes_cpp talker'
  alias testsub='ros2 run demo_nodes_cpp listener'
  alias testpubimg='ros2 run image_tools cam2image'
  alias testsubimg='ros2 run image_tools showimage'
  ```
    

## 4. Colcon Build 테스트
- 빌드를 위한 워크스페이스 폴더를 생성 (폴더명은 자유롭게 변경가능(ros2_ws))
    
    ```bash
    $ mkdir -p ~/ros2_ws/src
    $ cd ~/ros2_ws
    ```
    
- Colcon 빌드 테스트를 위해 ros2 example 코드를 clone    
    ```bash
    $ source /opt/ros/foxy/setup.bash
    $ cd ~/ros2_ws/src
    $ git clone https://github.com/ros2/examples
    $ cd examples
    $ git checkout $ROS_DISTRO
    ```
    
- Colcon 빌드를 수행
    
    ```bash
    $ cd ~/ros2_ws
    $ colcon build --symlink-install
    ```
    

## 5. 간단한 ROS 2 Node 테스트
- 기본적인 node 테스트를 통해 설치한 ROS 2 가 정상 작동하는지 확인
    
    ```bash
    # Run Publisher
    $ ros2 run demo_nodes_cpp talker
    
    # Run Subscriber
    $ ros2 run demo_nodes_py listener
    ```
