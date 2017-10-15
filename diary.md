# Hai Long's diary
## 5 Oct 2017
- Try to run `ros` natively in my Ubuntu 16.04
    + Install `ros` (follow `ROS` website instruction)
    + Try to `catkin_make` from `ros` folder, got `cmake` **errors**: _Could not find a package configuration file provided by "dbw_mkz_msgs"_
    + Follow instruction in the project [README](https://github.com/9mat/CarND-Capstone#native-installation) to install Dataspeed DBW (since I am not using Udacity VM, which already included the package)
    + `catkin_make` succeeds
    + `roslaunch` fails because `_tf2` module not found. `_tf2` has been installed but somehow `roslaunch` cannot see if
    + Try using python 2.7 to see if the problem is because of the use of python 3
    + Create `conda` env `ros` with `python2.7` and `catkin_make` under this env
    + `pip install -r requirement.txt`
    + `catkin_male` complains about module `em`. Do `pip install empy`
    + `catkin_make` succeeds. `roslaunch` complains about `rospkg`
    + `pip install rospkg`
    + `roslaunch` complains about `catkin_pkg.packages`
    + `pip install catkin-pkg`
    + no more complains
    + run Udacity simulator
    + succesful open the simulator & see response fron `ros`
- Try to get the car to move
    + create branch `get-car-moving`
    + uncomment code that creates and calls the controller
    + set `throttle` = 1
    + `logwarn` confirms that the function is called, but the car does not move
    + In the simulator, uncheck manual --> the car moves
    + Implement simple way point updater
    + Use YawController for DBW controller
    + The car now follows the track