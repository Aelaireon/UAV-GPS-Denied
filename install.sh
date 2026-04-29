# Search for ### to scroll through sections

### ------------------------------ ROS 2 JAZZY ------------------------------
locale  # check for UTF-8

sudo apt update && sudo apt install locales -y
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

locale  # verify settings

sudo apt install software-properties-common -y
sudo add-apt-repository universe -y

sudo apt update && sudo apt install curl -y
export ROS_APT_SOURCE_VERSION=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | grep -F "tag_name" | awk -F\" '{print $4}')
curl -L -o /tmp/ros2-apt-source.deb "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.$(. /etc/os-release && echo ${UBUNTU_CODENAME:-${VERSION_CODENAME}})_all.deb"
sudo dpkg -i /tmp/ros2-apt-source.deb

sudo apt update && sudo apt install ros-dev-tools -y

sudo apt update
sudo apt upgrade -y

sudo apt install ros-jazzy-desktop -y

source /opt/ros/jazzy/setup.bash

### ----------------------------- MAVROS Install -----------------------------
sudo apt install ros-jazzy-mavros -y
sudo apt install ros-jazzy-mavlink -y
sudo apt install ros-jazzy-libmavconn -y

sudo bash /opt/ros/jazzy/lib/mavros/install_geographiclib_datasets.sh
### ---------------------------- BUILD WORKSPACE ----------------------------
make
make

### ---------------------------- SETUP WORKSPACE ----------------------------
source install/setup.bash

### ------------------------------- Reminders -------------------------------
echo "REMINDER: Add 'source /opt/ros/jazzy/setup.bash' for automatic ROS 2 setup on new terminal launch."
echo "REMINDER: On new terminals, source the workspace setup file 'source /path/to/your/workspace/install/setup.bash' so ROS 2 can see your packages."
