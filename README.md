# Sustech EE346 Final Contest
## Haoteng Ye, Hongjing Tang
--- 
## Download File
```
cd ~/catkin_ws/src
git clone https://github.com/LoGY0821/lane_following_6.git
```
+ Before running the program, the camera parameter should be put into the corresponding folder. The example folder is: 
```
~/catkin_ws/src/turtlebot3_autorace_2020/turtlebot3_autorace_traffic_light/turtlebot3_autorace_traffic_light_camera/calibration
```
+ Then: 
```
cd ~/catkin_ws
catkin_make
```
--- 
## Lane Following
+ Open one terminal on PC: 
```
roscore
```
+ Open two terminals on Pi, one code for each: 
```
roslaunch turtlebot3_bringup turtlebot3_robot.launch 
roslaunch turtlebot3_autorace_traffic_light_camera turtlebot3_autorace_camera_pi.launch
```
+ Open terminals on PC, one code for each: 
```
roslaunch turtlebot3_autorace_traffic_light_camera turtlebot3_autorace_intrinsic_camera_calibration.launch
rqt
rosrun rqt_reconfigure rqt_reconfigure
```
+ Try to change the camera to a suitable situation. One example that may be suitable: 
	+ Contrast: 100
	+ Sharpness: 100
	+ Brightness: 48
	+ Saturation: 100
	+ ISO: 100
	+ Exposure_compensation: 0
+ Open another terminal on PC: 
```
cd ~/catkin_ws/src/lane_following/scripts
python lane_following_real_8.py
```
--- 
## Navigation
+ Open one terminal on PC: 
```
roscore
```
+ Open one terminal on Pi: 
```
roslaunch turtlebot3_bringup turtlebot3_robot.launch
```
+ Open terminal on PC, one code for each: 
```
roslaunch turtlebot3_navigation turtlebot3_navigation.launch map_file:=$HOME/map.yaml
roslaunch turtlebot3_teleop turtlebot3_teleop_key.launch
```
+ Use the keyboard to estimate the initial pose and help it judge the environment. Then, open another terminal on PC: 
```
cd ~/.catkin_ws
rosrun hit_nav move_base_square_hit
```
