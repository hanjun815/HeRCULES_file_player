# File Player for HeRCULES Dataset

 <div align="center">
    
  ![overview](https://github.com/user-attachments/assets/c3b71b0e-3a5f-4c9c-91e3-bc6c23870f03)

 </div>

## News
- 2025/02/05: Our dataset is available via [https://sites.google.com/view/herculesdataset](https://sites.google.com/view/herculesdataset).
- 2025/02/05 : ROS based fileplayer of the HeRCULES dataset is released.
- 2025/03/08 : PR_GT has been updated with a unified local coordinate system.
- 2025/03/10 : Stereo Camera Data is uploaded.
- 2025/05/30 : HeRCULES_Pointcloud_Toolbox is released [https://github.com/hanjun815/HeRCULES_Pointcloud_Toolbox](https://github.com/hanjun815/HeRCULES_Pointcloud_Toolbox).
  
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
$ cd ~/catkin_ws
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

## License and Citation
- When using the dataset or code, please cite our paper:
```
@INPROCEEDINGS { hjkim-2025-icra,
    AUTHOR = { Hanjun Kim and Minwoo Jung and Chiyun Noh and Sangwoo Jung and Hyunho Song and Wooseong Yang and Hyesu Jang and Ayoung Kim },
    TITLE = { HeRCULES: Heterogeneous Radar Dataset in Complex Urban Environment for Multi-session Radar SLAM },
    BOOKTITLE = { Proceedings of the IEEE International Conference on Robotics and Automation (ICRA) },
    YEAR = { 2025 },
    MONTH = { May. },
    ADDRESS = { Atlanta },
}
```

## Radar Development Tools
- Our Radar development tools are available via [Polar2X](https://github.com/hanjun815/Polar2X).

## Contributors
- Maintainer: Hanjun Kim (hanjun815@snu.ac.kr)
- Jinyong Jeong: The original author
- Minwoo Jung: made the player system compatible with LIO-SAM input (i.e., supports ring information of a lidar scan)

## 5. ROS2 Usage

### Prerequisites
- ROS2 (tested on Humble/Iron; Desktop-Full recommended)
- Install required dependencies:
  - `novatel_gps_msgs`, `pcl_conversions`, `tf2`, `tf2_eigen`, `std_srvs`, etc. (use `rosdep` to install dependencies)
  - Example:
    ```
    rosdep install --from-paths src --ignore-src -r -y
    ```

### Build Instructions
```
$ mkdir -p ~/ros2_ws/src
$ cd ~/ros2_ws/src
$ git clone https://github.com/hanjun815/HeRCULES_file_player.git
$ cd ~/ros2_ws
$ rosdep install --from-paths src --ignore-src -r -y
$ colcon build --symlink-install
$ source install/setup.bash
```

### How to Launch the File Player (ROS2)
```
$ ros2 launch hercules_file_player hercules_file_player.launch.py
```
- You can enable/disable specific topics using launch arguments, e.g.:
  ```
  $ ros2 launch hercules_file_player hercules_file_player.launch.py enable_aeva:=true enable_stereo:=false
  ```
- The GUI will appear for sequence selection as in ROS1.

### Notes
- The ROS2 version does not require rosbag or dynamic_reconfigure.
- All topic publishers are managed via launch parameters.
- If you encounter missing plugin errors in RViz, ensure you do not have broken/unused plugins installed.
- For rqt and RViz2, use the standard ROS2 tools (`rqt`, `rviz2`).

### Prepare the data and timestamps

If your data directory is represented as follows, you are now ready to enjoy the HeRCULES dataset!
```
📂 Sequence_name/
├── 📂 LiDAR/
│   └── 📂 Aeva/
│       └── 📝 timestamp.bin
│── 📂 Radar/
│   │── 📂 Continental/
│   │   └── 📝 timestamp.bin
│   │── 📂 Continentalobject/
│   │   └── 📝 timestamp.bin
│   └── 📂 Navtech/
│       └── 📝 timestamp.bin
│── 📂 .../
└── 📂 sensor_data/
       │── 📝 aeva_stamp.csv
       │── 📝 continentalobject_stamp.csv
       │── 📝 continental_stamp.csv
       │── 📝 datastamp.csv     
       │── 📝 gps.csv     
       │── 📝 inspva.csv     
       │── 📝 navtech_stamp.csv     
       │── 📝 stereo_stamp.csv     
       └── 📝 xsens_imu.csv     

```

### Load data files and play

1. Click the "Load" button.
2. Choose Sequence_name folder including sensor_data folder and data_stamp.csv.
3. The "Play" button starts publishing data in the ROS message.
4. The "Pause/Resume" button pauses and resumes publishing data.
5. The "Save" button saves all topics into the rosbag file.
6. The "Loop" checkbox resumes when playback is finished.

Enjoy it:) 


