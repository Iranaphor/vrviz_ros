sudo apt-get update && sudo apt-get dist-upgrade && sudo apt-get install curl

sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
curl -sSL 'http://keyserver.ubuntu.com/pks/lookup?op=get&search=0xC1CF6E31E6BADE8868B172B4F42ED6FBAB17C654' | sudo apt-key add -
sudo apt update

curl https://raw.githubusercontent.com/LCAS/rosdistro/master/lcas-rosdistro-setup.sh | bash -
sudo apt install ros-melodic-ros-base

read -p "Set workspace path: ~/" ws_path
#export ws_path=vrviz_dev_ws

mkdir -p ~/$ws_path/src
cd ~/$ws_path/src
catkin_init_workspace

git clone --recursive https://github.com/Iranaphor/vrviz_ros.git
#git clone --recursive https://github.com/LCAS/mqtt_bridge.git

cd ~/$ws_path/
rosdep install --from-paths src --ignore-src -r -y

catkin_make

echo "export ws_path=$ws_path" >> ~/.bashrc
echo "source ~/$ws_path/src/vrviz_ros/.vrvizrc" >> ~/.bashrc

echo "roslaunch vrviz_ros basic_test_simulation.launch" >> ~/.bashrc

source ~/.bashrc
