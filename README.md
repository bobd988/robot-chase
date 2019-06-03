# Project Go Chase It

This project shows how to write ROS plugins to analysis sensor data and perform customized action from gazebo ROS simulation environment. There are two ROS projects needed to be run in seperate process. 

## 1. `my_robot` package:

The `my_robot` ROS package holds a robot, a white ball, and the world.

The robot model has two sensors:

 - [lidar hokuyo](http://gazebosim.org/tutorials?tut=ros_gzplugins#GPULaser)
 - [camera](http://gazebosim.org/tutorials?tut=ros_gzplugins#Camera)

To steer the robot a Gazebo plugin is added for robot’s differential drive, lidar, and camera. 

A white-colored ball is added to the Gazebo world which can be moved around and the robot will chase the ball. 
```
roslaunch my_robot world.launch
```

## 2. `ball_chaser` package:

* The `ball_chaser/command_robot` service to drive the robot by controlling its `linear x` and `angular z` velocities. 
The service publishes to the wheel joints and returns back the requested velocities.
* The `process_image` reads the robot’s camera image and analyzes it to determine the presence and position of a white ball. 
If a white ball exists in the image, the node request a service via a client to drive the robot.
* The `ball_chaser.launch` will start both the `drive_bot` and the `process_image` nodes.


## File Structure

```
.Project                          
    ├── my_robot                       
    │   ├── launch                     
    │   │   ├── robot_description.launch
    │   │   ├── world.launch
    │   ├── meshes                     
    │   │   ├── hokuyo.dae
    │   ├── urdf                      
    │   │   ├── rc_robot.gazebo
    │   │   ├── rc_robot.xacro
    │   ├── world                     
    │   │   ├── hello.world
    │   ├── CMakeLists.txt            
    │   ├── package.xml               
    ├── ball_chaser                    
    │   ├── launch                      
    │   │   ├── ball_chaser.launch
    │   ├── src                        
    │   │   ├── drive_bot.cpp
    │   │   ├── process_images.cpp
    │   ├── srv                        
    │   │   ├── DriveToTarget.srv
    │   ├── CMakeLists.txt              
    │   ├── package.xml               
    └──
```

```
roslaunch ball_chaser ball_chaser.launch
```
