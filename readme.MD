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