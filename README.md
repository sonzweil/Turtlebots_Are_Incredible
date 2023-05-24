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
![K-003](https://github.com/sonzweil/turtlebots_are_incredible/assets/15362456/3746e5bb-a2ca-4e3c-a6ce-39ede1d2dad5)

### rqt_graph
![스크린샷, 2023-04-30 16-50-59](https://github.com/sonzweil/turtlebots_are_incredible/assets/15362456/b978ff0b-e294-435c-a59c-dd0cba510488)
![K-004](https://github.com/sonzweil/turtlebots_are_incredible/assets/15362456/1b2f51d8-1847-40bf-95a1-d655d7e2aa06)

