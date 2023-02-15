ROS_WORKSPACE_DIR=$HOME/ros

mkdir -p $ROS_WORKSPACE_DIR/src
cd $ROS_WORKSPACE_DIR
catkin_make
git clone https://github.com/mfocchi/locosim.git $ROS_WORKSPACE_DIR/src/locosim
cd $ROS_WORKSPACE_DIR/src/locosim
git submodule update --init --recursive
cd $ROS_WORKSPACE_DIR
catkin_make install
rospack profile

echo "export ROS_WORKSPACE_DIR=$ROS_WORKSPACE_DIR" >> $HOME/.bashrc
echo "source \$ROS_WORKSPACE_DIR/install/setup.bash" >> $HOME/.bashrc
echo "export LOCOSIM_DIR=\$ROS_WORKSPACE_DIR/src/locosim" >> $HOME/.bashrc
echo "export PYTHONPATH=\$LOCOSIM_DIR/robot_control:\$PYTHONPATH" >> $HOME/.bashrc
