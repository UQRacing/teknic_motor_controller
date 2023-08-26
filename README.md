# Teknic Motor Controller

## Table of Contents
1. [Teknic Motor Controller](#teknic-motor-controller)  
   1. [Requirements](#requirements)  
   1. [Setup](#setup)  
   2. [Utiliztion](#utilization)  
   3. [Configuration Files Breakdown](#configuration-files-breakdown)  
   4. [Teknic Documentation](#teknic-documentation)
   5. [Helpful Links](#helpful-links)  
2. [Trash Bot Motor Controller](./src/trash_bot_motor_controller)  
3. [Teknic Examples](./src/SDK_Examples)  
   1. [Example GPIO](./src/SDK_Examples/Example-GPIO)  
   2. [Example Homing](./src/SDK_Examples/Example-Homing)  
   3. [Example Motion](./src/SDK_Examples/Example-Motion)  
   4. [Example MultiThreaded](./src/SDK_Examples/Example-MultiThreaded)  
   5. [Example SingleThreaded](./src/SDK_Examples/Example-SingleThreaded)  
   6. [Example StatusAlerts](./src/SDK_Examples/Example-StatusAlerts)  
   7. [HelloWorld](./src/SDK_Examples/HelloWorld)  

This library by jcook3701 is being used for the integration of ros and the sFoundation library used for controlling ClearPath motors.  

Below are the instructions for the Trash Bot Motor Controller by jcook3701. It may be broken from the port between ros-melodic to ros-noetic.  

## Requirements
The following files are recommended for installation:  
```
$ sudo apt-get install ros-noetic-teleop-twist-keyboard  
```

## Setup
1. Create ROS Workspace.  
```
$ mdir -p ~/Documents/ros_workspace/trash_bot_workspace/src  
```

2. Move to workspace.  
```
$ cd ~/Documents/ros_workspace/trash_bot_workspace/src  
```

3. Clone this repository into newly created Trash Bot Workspace.  
```
$ git clone git@github.com:jcook3701/teknic_motor_controller.git  
```

4. Move to the head of the project workspace and build the ROS Teknic Motor Controller.  
```
$ cd .. && catkin_make  
```

## Utilization
1. Source the environment file from the head of the project workspace.  
```
$ source devel/setup.bash  
```

2. Use roslaunch to start the motor controller.  
```
$ roslaunch teknic_motor_controller trash_bot_motor_controller.launch  
```

## Configuration Files Breakdown

__Note:__  The following assumptions are being made when using this motor controller.  
1. Each motor is driving two wheels.  Each wheel has its own chain that connects it to either the left or the right motor.  
2. Motor orientation is the following:  
```
  (L)  (R)  
---[]  []---  
```
3. There are exactly four wheels.  This could be modified in the future with an additional parameter to the vehicle configuration file.  

__Vehicle Parameters:__  
1. Number of Motors  
   - __Description:__ Number of motors being used on vehicle.  It is assumed that this number will be even with a minimum of two motors.  
2. Gear Ratio  
   - __Description:__ Gear ratio between motor and single tire rotation.  
3. Wheel Base:
   - __Description:__ The length of the wheel base in meters.  
4. Wheel Radius:
   - __Description:__ The length of the wheel radius in meters.  
5. Tire Deflection
   - __Description:__ Deformation of tire at the point of contact  
6. Diameter Mod
   - __Description:__ 

__Teknic Motor Parameters:__  
1. Encoder Resolution
   - __Description:__ Encoder tics per wheel revolution     
2. Max Motor Velocity  
   - __Description:__ Max velocity that the motor may reach during operations.  
3. Max Motor Acceleration  
   - __Description:__ Max acceleration that the motor may reach when ramping to the specified velocity.  
4. Max Motor Torque Percentage  
   - __Description:__ Max percentage of available torque that the motor may use when operating.  

## Teknic Documentation

[Clearpath SC User Manual](./docs/Clearpath-SC-User-Manual.pdf)  
__Description:__ This goes over the general setup and configuration of ClearPath Motors.  

[S-Foundation Reference](./docs/S-FoundationRef.chm)  
__Description:__ This goes over the ClearPath SDK.  


## Helpful Links
[Differential Drive Kinematics](http://www.cs.columbia.edu/~allen/F15/NOTES/icckinematics.pdf)  
[Control of Mobile Robots- 2.2 Differential Drive Robots](https://www.youtube.com/watch?v=aE7RQNhwnPQ&ab_channel=mouhknowsbest)  
[Kinematics of a 4 wheeled differential drive robots](https://robotics.stackexchange.com/questions/2324/kinematics-of-a-4-wheeled-differential-drive-robots)  
[rosbot kernel headers](https://community.husarion.com/t/linux-header-files-linux-headers-4-4-71husarion5/657/4)  
