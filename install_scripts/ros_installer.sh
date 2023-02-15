PYTHON_PREFIX=python3
PYTHON_VERSION=3.8
ROBOTPKG_PYTHON_VERSION=py38
PINOCCHIO_PYTHON_VERSION=py38
PIP_PREFIX=pip3
ROS_VERSION=noetic

sudo apt-get install curl
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
curl -sSL 'http://keyserver.ubuntu.com/pks/lookup?op=get&search=0xC1CF6E31E6BADE8868B172B4F42ED6FBAB17C654' | sudo apt-key add -
sudo apt-get update
sudo apt-get install -y \
    ros-$ROS_VERSION-desktop-full \
    ros-$ROS_VERSION-urdfdom-py \
    ros-$ROS_VERSION-srdfdom \
    ros-$ROS_VERSION-joint-state-publisher \
    ros-$ROS_VERSION-joint-state-publisher-gui \
    ros-$ROS_VERSION-joint-state-controller \
    ros-$ROS_VERSION-gazebo-msgs \
    ros-$ROS_VERSION-control-toolbox \
    ros-$ROS_VERSION-gazebo-ros \
    ros-$ROS_VERSION-controller-manager \
    ros-$ROS_VERSION-joint-trajectory-controller \
    ros-$ROS_VERSION-openni2-launch \
    ros-$ROS_VERSION-openni2-camera \
    ros-$ROS_VERSION-realsense2-description \
    ros-$ROS_VERSION-eigen-conversions \
    ros-$ROS_VERSION-object-recognition-msgs \
    ros-$ROS_VERSION-roslint \

sudo sh -c "echo 'deb [arch=amd64] http://robotpkg.openrobots.org/packages/debian/pub $(lsb_release -sc) robotpkg' > /etc/apt/sources.list.d/robotpkg.list"
sudo sh -c "echo 'deb [arch=amd64] http://robotpkg.openrobots.org/wip/packages/debian/pub $(lsb_release -sc) robotpkg' >> /etc/apt/sources.list.d/robotpkg.list"
sudo apt install -qqy lsb-release gnupg2 curl
curl http://robotpkg.openrobots.org/packages/debian/robotpkg.key | sudo apt-key add -
sudo apt-get update
sudo apt-get install -y \
    robotpkg-$PINOCCHIO_PYTHON_VERSION-eigenpy \
    robotpkg-$PINOCCHIO_PYTHON_VERSION-pinocchio \
    robotpkg-$PINOCCHIO_PYTHON_VERSION-quadprog  \

sudo apt-get install -y \
    $PYTHON_PREFIX-scipy \
    $PYTHON_PREFIX-matplotlib \
    $PYTHON_PREFIX-termcolor \
    python3-pip
$PIP_PREFIX install cvxpy==1.2.0


echo "export PYTHON_VERSION=$PYTHON_VERSION" >> $HOME/.bashrc
echo "export ROS_VERSION=$ROS_VERSION" >> $HOME/.bashrc
echo "source /opt/ros/$ROS_VERSION/setup.sh" >> $HOME/.bashrc
echo "export PATH=/opt/openrobots/bin:\$PATH" >> $HOME/.bashrc
echo "export ROS_PACKAGE_PATH=\$ROS_PACKAGE_PATH:/opt/openrobots/share/" >> $HOME/.bashrc
echo "export PYTHONPATH=/opt/openrobots/lib/python$PYTHON_VERSION/site-packages:\$PYTHONPATH" >> $HOME/.bashrc