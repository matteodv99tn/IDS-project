# Installation instructions
The project is based in _ROS_ and the [_Locosim framework_](https://github.com/mfocchi/locosim), that are so required to be installed on the machine; furthermore all path variables must be set in the `.bashrc` file, as suggested in the Locosim repository.

To simplify the installation process scripts inside the [`install_scripts`](install_scripts/) can be used.
Once they are made executable (`chmod a+x install_scripts/*.sh`), you can install ROS with
``` sh
install_scripts/ros_installer.sh
```
The script is set to install the ROS version compatible with _Ubuntu 20.04_; to install on older platforms just change the variables in the first lines of the files (see _software versions_ inside the readme of the Locosim repository).

To install Locosim run
``` sh
install_scripts/locosim_installer.sh
```
In this case the lonely parameter that should be changed is the installation destination of the ROS workspace and locosim, that by default is in `~/ros`.

## Cloning this repository

Once the ROS workspace `~/ros` is set up, you can clone the repository inside the `~/ros/src` folder:
```
git clone https://github.com/matteodv99tn/IDS-project.git cooperative_ur5_estimation_control
```
The package for ROS has a different name then the repository, so be aware of the second argument of the clone command.


# Bibliography and references:

- [_Cooperative manipulation_](http://www.prisma.unina.it/courses/SHB_39.pdf): interesting book extract containing useful informations in order to work multiple robots;
- [_Control and Estimation for Cooperative Manipulator Tasks_](http://dspace.mit.edu/handle/1721.1/31217): technical report for a project whee they discuss the problem of handing a object from one robot to another;
- [_Real-Time Kinematic Control for Redundant Manipulators in a Time-Varying Environment: Multiple-Dynamic Obstacle Avoidance and Fast Tracking of a Movign Object_](https://ieeexplore.ieee.org/document/8716565): here they use principal component analysis to adapt the tracking capability of the system (the paper studies a mobile robot on top of which multiple manipulators are mounted);
