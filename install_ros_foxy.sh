#!/bin/bash
# Apache License 2.0
# Copyright (c) 2022, Neoplanetz CO., LTD. referenced from ROBOTIS

echo ""
echo "[Note] Target OS version  >>> Ubuntu 20.04.x (Focal Fossa)"
echo "[Note] Target ROS version >>> ROS2 Foxy"
echo "[Note] Colcon workspace   >>> $HOME/ros2_ws"
echo ""
echo "PRESS [ENTER] TO CONTINUE THE INSTALLATION"
echo "IF YOU WANT TO CANCEL, PRESS [CTRL] + [C]"
read

echo "[Set the target OS, ROS version and name of colcon workspace]"
name_os_version=${name_os_version:="focal"}
name_ros_version=${name_ros_version:="foxy"}
name_colcon_workspace=${name_colcon_workspace:="ros2_ws"}

echo "[Update the package lists]"
sudo apt update -y

echo "[Install Locales and set en_US.UTF-8 locale]"
sudo apt install -y locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

echo "[Add the ROS2 repository]"
if [ ! -e /etc/apt/sources.list.d/ros2-latest.list ]; then
  sudo sh -c 'echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu ${name_os_version} main" > /etc/apt/sources.list.d/ros2-latest.list'
fi

echo "[Download the ROS2 keys]"
roskey=`apt-key list | grep "Open Robotics"`
if [ -z "$roskey" ]; then
  sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key  -o /usr/share/keyrings/ros-archive-keyring.gpg
fi

echo "[Check the ROS2 keys]"
roskey=`apt-key list | grep "Open Robotics"`
if [ -n "$roskey" ]; then
  echo "[ROS2 key exists in the list]"
else
  echo "[Failed to receive the ROS2 key, aborts the installation]"
  exit 0
fi

echo "[Update the package lists]"
sudo apt update -y

echo "[Install the ros-foxy-desktop and rmw fastrtps, cyclonedds, rqt plugins, xacro]"
sudo apt install -y ros-$name_ros_version-desktop ros-$name_ros_version-rmw-fastrtps* ros-$name_ros_version-rmw-cyclonedds* ros-$name_ros_version-rqt* ros-$name_ros_version-image-view ros-$name_ros_version-xacro

echo "[Environment setup and install ROS development tool packages]"
source /opt/ros/$name_ros_version/setup.bash
sudo apt install -y build-essential cmake git libbullet-dev python3-colcon-common-extensions python3-flake8 python3-pip python3-pytest-cov python3-rosdep python3-setuptools python3-vcstool wget
python3 -m pip install -U argcomplete flake8-blind-except flake8-builtins flake8-class-newline flake8-comprehensions flake8-deprecated flake8-docstrings flake8-import-order flake8-quotes pytest-repeat pytest-rerunfailures pytest
sudo apt install --no-install-recommends -y libasio-dev libtinyxml2-dev libcunit1-dev

echo "[Make the ros2 workspace and git clone ros2 example and colcon build test]"
mkdir -p $HOME/$name_colcon_workspace/src
cd $HOME/$name_colcon_workspace/src
git clone https://github.com/ros2/examples
cd examples
git checkout $ROS_DISTRO

cd $HOME/$name_colcon_workspace
colcon build --symlink-install

echo "[Set the ROS2 evironment to bashrc file]"
sh -c "echo \"# ROS 2 Setup\" >> ~/.bashrc"
sh -c "echo \"source /opt/ros/$name_ros_version/setup.bash\" >> ~/.bashrc"
sh -c "echo \"source ~/$name_colcon_workspace/install/local_setup.bash\" >> ~/.bashrc"

sh -c "echo \" \" >> ~/.bashrc"
sh -c "echo \"alias eb='gedit ~/.bashrc'\" >> ~/.bashrc"
sh -c "echo \"alias sb='source ~/.bashrc'\" >> ~/.bashrc"

sh -c "echo \"source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash\" >> ~/.bashrc"
sh -c "echo \"source /usr/share/vcstool-completion/vcs.bash\" >> ~/.bashrc"
sh -c "echo \"source /usr/share/colcon_cd/function/colcon_cd.sh\" >> ~/.bashrc"
sh -c "echo \"export _colcon_cd_root=~/$name_colcon_workspace\" >> ~/.bashrc"

sh -c "echo \"export ROS_DOMAIN_ID=7\" >> ~/.bashrc"
sh -c "echo \"export ROS_NAMESPACE=robot1\" >> ~/.bashrc"

sh -c "echo \"export RMW_IMPLEMENTATION=rmw_fastrtps_cpp\" >> ~/.bashrc"
sh -c "echo \"# export RMW_IMPLEMENTATION=rmw_connext_cpp\" >> ~/.bashrc"
sh -c "echo \"# export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp\" >> ~/.bashrc"
sh -c "echo \"# export RMW_IMPLEMENTATION=rmw_gurumdds_cpp\" >> ~/.bashrc"

sh -c "echo \"# export RCUTILS_CONSOLE_OUTPUT_FORMAT='[{severity} {time}] [{name}]: {message} ({function_name}() at {file_name}:{line_number})'\" >> ~/.bashrc"
sh -c "echo \"export RCUTILS_CONSOLE_OUTPUT_FORMAT='[{severity}]: {message}'\" >> ~/.bashrc"
sh -c "echo \"export RCUTILS_COLORIZED_OUTPUT=1\" >> ~/.bashrc"
sh -c "echo \"export RCUTILS_LOGGING_USE_STDOUT=0\" >> ~/.bashrc"
sh -c "echo \"export RCUTILS_LOGGING_BUFFERED_STREAM=1\" >> ~/.bashrc"

sh -c "echo \"# Alias Setting\" >> ~/.bashrc"
sh -c "echo \"alias cw='cd ~/$name_colcon_workspace'\" >> ~/.bashrc"
sh -c "echo \"alias cs='cd ~/$name_colcon_workspace/src'\" >> ~/.bashrc"
sh -c "echo \"alias ccd='colcon_cd'\" >> ~/.bashrc"

sh -c "echo \"alias cb='cd ~/$name_colcon_workspace&& colcon build --symlink-install'\" >> ~/.bashrc"
sh -c "echo \"alias cbs='colcon build --symlink-install'\" >> ~/.bashrc"
sh -c "echo \"alias cbp='colcon build --symlink-install --packages-select'\" >> ~/.bashrc"
sh -c "echo \"alias cbu='colcon build --symlink-install --packages-up-to'\" >> ~/.bashrc"
sh -c "echo \"alias ct='colcon test'\" >> ~/.bashrc"
sh -c "echo \"alias ctp='colcon test --packages-select'\" >> ~/.bashrc"
sh -c "echo \"alias ctr='colcon test-result'\" >> ~/.bashrc"

sh -c "echo \"alias rt='ros2 topic list'\" >> ~/.bashrc"
sh -c "echo \"alias re='ros2 topic echo'\" >> ~/.bashrc"
sh -c "echo \"alias rn='ros2 node list'\" >> ~/.bashrc"

sh -c "echo \"alias killgazebo='killall -9 gazebo & killall -9 gzserver  & killall -9 gzclient'\" >> ~/.bashrc"

sh -c "echo \"alias af='ament_flake8'\" >> ~/.bashrc"
sh -c "echo \"alias ac='ament_cpplint'\" >> ~/.bashrc"

echo "[Install Gazebo 11 for ROS2 foxy]"
sudo sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list'
wget https://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add -
sudo apt update & sudo apt install gazebo11 libgazebo11-dev -y
sudo apt install ros-foxy-gazebo-ros-pkgs -y

source $HOME/.bashrc

echo "[Complete!!!]"
exit 0
