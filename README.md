# feihu_sentry

### 项目介绍
本项目是北部湾大学飞虎战队的哨兵导航包，开发过程中从深圳北理莫斯科大学北极熊战队开源的导航包[链接](http://gitee.com/SMBU-POLARBEAR/pb_rm_simulation)中得到了很多启发。

### 硬件设备
1.  Mini Pc:13代 intel酷睿i9-13900H BQM6
   
2.  传感器：Livox-Mid360激光雷达、Intel-d435深度相机 

### 开发环境
Ubuntu22.04 ROS2 Humble

### 第三方依赖库
fast_lio:https://github.com/hku-mars/FAST_LIO.git

livox_ros_driver2:https://github.com/Livox-SDK/livox_ros_driver2

rtabmap:https://github.com/introlab/rtabmap_ros.git

rtabmap_ros:https://github.com/introlab/rtabmap_ros.git

### 配置安装
1.安装livox SDK (https://github.com/Livox-SDK/Livox-SDK2)

```
git clone https://github.com/Livox-SDK/Livox-SDK2.git
cd ./Livox-SDK2/
mkdir build
cd build
cmake .. && make -j
sudo make install
```

2.安装RealSense SDK （https://dev.intelrealsense.com/docs/compiling-librealsense-for-linux-ubuntu-guide）

2.1安装依赖项

```
sudo apt-get update && sudo apt-get upgrade && sudo apt-get dist-upgrade
sudo apt-get install libssl-dev libusb-1.0-0-dev libudev-dev pkg-config libgtk-3-dev
sudo apt-get install git wget cmake build-essential
sudo apt-get install libglfw3-dev libgl1-mesa-dev libglu1-mesa-dev at
```
2.2安装和构建SDK

```
git clone https://github.com/IntelRealSense/librealsense.git
cd librealsense 
./scripts/setup_udev_rules.sh
mkdir build && cd build
cmake ../ -DBUILD_EXAMPLES=true
sudo make uninstall && make clean && make && sudo make install
```

3.克隆仓库到本地：

```
git clone https://github.com/bbguhdm/feihu.git`
```

4.安装依赖

```
rosdep update && rosdep install --from-paths src --ignore-src -r -y
```

5.编译

```
export MAKEFLAGS="-j6" # Can be ignored if you have a lot of RAM (>16GB)

colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
```

### 运行

#### 建图：


![image](https://github.com/user-attachments/assets/cd6837fa-8c1a-419b-b8d2-85c0ac9a53c6)



（建图后可通过运行  ```ros2 run nav2_map_server map_saver_cli -f 保存路径/地图名称``` 保存栅格地图）

（pcd地图的保存可以通过运行`ros2 service call /map_save std_srvs/srv/Trigger`实现，保存的路径在mid360.yaml文件中的`map_file_path`中指定）

rtabmap建图:

```
./mapping_rtab.sh
```

cartographer建图：

```
./mapping_cartographer.sh
```

slam_mapping建图：

```
./mapping_slam_mapping.sh
```



#### 导航：


![image](https://github.com/user-attachments/assets/b9512b46-0114-4233-bc3f-da8ff1f1ac5b)



rtabmap定位模式：

```
./nav_rtab.sh
```

icp定位模式（需要pcd地图）：

```
./nav_icp.sh
```

amcl定位模式：

```
./nav_amcl.sh
```
