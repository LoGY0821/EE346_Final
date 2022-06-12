# Sustech EE346 Final Contest
## Haoteng Ye, Hongjing Tang
--- 
## Download File
```
cd ~/catkin_ws/src
git clone https://github.com/LoGY0821/EE346_Final.git
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
python final_test2.py
```
