# turtlebots_are_incredible
rqt Plugin for controlling multiple turtlebots

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
 <-- under construction -->
