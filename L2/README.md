## Build
### odom_ws
```
sudo apt-get install ros-kinetic-csm
cd odom_ws/src
catkin_init_workspace
cd ..
catkin_make
```
## File explanation
### odom.bag
```
roscore
rostopic list
```
you can find the odom.bag has the odom data and scan data

## Run
### odom_ws
```
source odom_ws/devel/setup.bash
roslaunch calib_odom odomCalib.launch
cd path_to_bag
rosbag play --clock odom.bag
```
when the data is enough, publish calibration flag
`rostopic pub /calib_flag std_msgs/Empty "{}"`