# File player for HeRCULES dataset

Maintainer: Hanjun Kim (hanjun815@snu.ac.kr)


## News
- March 2025: Our dataset will be available via [To be updated](https://sites.google.com/view/hercules_dataset).

## What is File player?
This program is a file player for the complex urban data set. If a user installs the ROS using "Desktop-Full version", there is only one additional dependent package, except for the ROS default package. First, clone this package into the src folder of your desired ROS workspace.

## 1. Pre-requisites
Before utilizing the file player, it's crucial to have both the novatel-gps-msgs and livox custom messages. Ensure you install these drivers:

Novatel GPS Driver Installation:
Replace 'version' with your appropriate ROS version (e.g., melodic, noetic).
```
$ sudo apt-get install ros-'version'-novatel-gps-driver
```

## 2. How to install
```
$ mkdir -p catkin_ws/src
$ cd catkin_ws/src
$ git clone https://github.com/hanjun815/HeRCULES_file_player.git
$ cd ../..
$ catkin_make
```

## 3. How to Execute the File Player

```
$ source devel/setup.bash
$ roslaunch file_player file_player.launch
```
- Then, you need to select a sequence directory via GUI.
- For the correct load, first you need to place the GPS, IMU, LiDAR, and radar files in a directory following this structure (see this [guide video](https://youtu.be/uU-FC-GmHXA?t=45)) 
- IMU and GPS files (.csv) must be located at the same directory of "datastamp.csv"

## 4. Load Data Files and Play
Here's a step-by-step guide:

1. Click the 'Load' button.
2. Navigate and select the desired dataset folder.
3. Hit the player button to commence publishing data as ROS messages.
4. Use the 'Stop skip' button to skip intervals when the vehicle remains stationary. This feature enhances the user experience by focusing on significant data.
5. The loop button ensures that the data resumes playback from the beginning once completed


## Contributors
- Jinyong Jeong: The original author
- Minwoo Jung: made the player system compatible with LIO-SAM input (i.e., supports ring information of a lidar scan)


