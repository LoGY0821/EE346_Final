# EE346_SUSTech_Lane_Following
This is the lab assignment for SUSTech EE346, which is produced by Haoteng Ye, SID 11911414.

# Usage

## 1. Clone the source code
```bash
cd ~/catkin_ws/src
git clone https://github.com/LoGY0821/lane_following.git
```

## 2. Catkin make the lane following package
```bash
cd ..
catkin_make
```

## 3. Add course models
```bash
export GAZEBO_MODEL_PATH=${GAZEBO_MODEL_PATH}:~/catkin_ws/src/lane_following/models
```
   Or you can add it into the ~/.bashrc: 

```bash
echo "export GAZEBO_MODEL_PATH=${GAZEBO_MODEL_PATH}:~/catkin_ws/src/lane_following/models" >> ~/.bashrc
source ~/.bashrc
```

## 4. Launch the gazebo map
```bash
source ~/catkin_ws/devel/setup.bash
roslaunch lane_following race_track.launch 
```

## 5. Run provided lane following python node

```bash
cd ~/catkin_ws/src/lane_following/scripts/
chmod +x lane_following.py
cd ~/catkin_ws
source devel/setup.bash
rosrun lane_following lane_following.py
```

### Run part 1 (Fluently driving alone the lane)

```bash
cd ~/catkin_ws/src/lane_following/scripts/
chmod +x lane_following_part1.py
cd ~/catkin_ws
source devel/setup.bash
rosrun lane_following lane_following_part1.py
```

### Run part 2 (BEV)

```bash
cd ~/catkin_ws/src/lane_following/scripts/
chmod +x lane_following_part2.py
cd ~/catkin_ws
source devel/setup.bash
rosrun lane_following lane_following_part2.py
```

### Run part 3 (Stop)

```bash
cd ~/catkin_ws/src/lane_following/scripts/
chmod +x lane_following_part3.py
cd ~/catkin_ws
source devel/setup.bash
rosrun lane_following lane_following_part3.py
```
