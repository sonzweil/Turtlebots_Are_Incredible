# turtlebots_are_incredible
rqt Plugin for controlling multiple turtlebots  

### Require
Need turtlebot3_burger, turtlebot3_waffle  
Need to finish 3. Quick Start Guide on https://emanual.robotis.com/docs/en/platform/turtlebot3/overview/  
Need to change namespace for turtlebot3_burger "burger"  
Need to change namespace for turtlebot3_waffle "waffle_pi"  
namespace guide for turtlebot : https://discourse.ros.org/t/giving-a-turtlebot3-a-namespace-for-multi-robot-experiments/10756  

### install
colcon build --symlink-install --packages-select incredible_interface  
colcon build --symlink-install --packages-select turtlebots_are_incredible  
source ./install/local_setup.bash  

### start
ros2 run turtlebots_are_incredible turtlebots_are_incredible  
or  
ros2 launch turtlebots_are_incredible incredible.launch.py  

### If error occurs
rqt --force-discover  
or  
rm ~/.config/ros.org/rqt_gui.ini  

### using with rqt
You can find plugin after run  
rqt>Plugins>Turtlebot3>Turtlebots Are Incredible
