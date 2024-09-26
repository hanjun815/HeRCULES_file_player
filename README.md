# File player for HeRCULES dataset

Maintainer: Hanjun Kim (hanjun815@snu.ac.kr)


## News
- March 2025: Our dataset is now available via [HeRCULES dataset](https://sites.google.com/view/herculesdataset).

## What is File player?
This program is a file player for the complex urban data set. If a user installs the ROS using "Desktop-Full version", there is only one additional dependent package, except for the ROS default package. First, clone this package into the src folder of your desired ROS workspace.

## How to install
```
$ mkdir -p catkin_ws/src
$ cd catkin_ws/src
$ git clone https://github.com/hanjun815/HeRCULES_file_player.git
$ cd ../..
$ catkin_make
```

## How to use 

```
$ source devel/setup.bash
$ roslaunch file_player file_player.launch
```
- Then, you need to select a sequence directory via GUI.
- For the correct load, first you need to place the GPS, IMU, LiDAR, and radar files in a directory following this structure (see this [guide video](https://youtu.be/uU-FC-GmHXA?t=45)) 
- IMU and GPS files (.csv) must be located at the same directory of "data_stamp.csv"




