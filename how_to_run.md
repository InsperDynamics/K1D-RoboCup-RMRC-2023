# How to run the K1D robot

## Starting robot

```cd interface```
```npm start```
```cd ..```
```cd 'Main Software'```
```cmake -S . -B build```
```cd build```
```make```
```./K1D```

## Launching Autonomous

```cd catkin_ws/src/k1d/scripts```
```roscore```
```roslaunch turtlebot3_bringup turtlebot3_remote.launch```
```roslaunch hector_mapping mapping_default.launch odom_frame:=odom```
```rviz```
