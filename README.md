---
typora-root-url: ./
---

# panda_hand_eye_calibrate
基于easy_handeye开源项目，对Franka_panda进行手眼标定（Kinect v2）；里面主要是修改了一些easy_handeye中的一些launch文件，实现手眼转换矩阵的计算与发布。

## 依赖

1. 需要安装easy_handeye包;参看https://github.com/IFL-CAMP/easy_handeye；
2. 安装了Panda相关的ROS包；



## 标定

1. 自行安装好相机和标定板，标定板使用 [ar_track_alvar](http://wiki.ros.org/ar_track_alvar/)生成的10cm*10cm的二维码标签，id为7

   <img src="/pic/image-20210312174138027.png" alt="image-20210312174138027" style="zoom: 25%;" />

2. 解锁panda机械臂，并启动Moveit

   ```bash
   cd ~/catkin_ws #cd where you put your "panda_client.sh"
   source panda_client.sh -r
   # ctrl+c
   roslaunch panda_moveit_config zzu_panda_move.launch
   ```

3. 另一个命令窗口，打开相机

   ```bash
   cd ~/catkin_ws #cd where you put your "panda_client.sh"
   source panda_client.sh
   roslaunch kinect2_bridge kinect2_bridge.launch publish_tf:=true
   ```

4. 打开窗口进行监视

   ```bash
   cd ~/catkin_ws #cd where you put your "panda_client.sh"
   source panda_client.sh 
   rosrun kinect2_viewer kinect2_viewer kinect2 sd cloud
   ```

   

5. 另一个命令窗口，进行标定，按照提示进行标定

   ```bash
   cd ~/catkin_ws #cd where you put your "panda_client.sh"
   source panda_client.sh 
   roslaunch panda_hand_eye_calibrate panda_eob.launch
   ```

## 已标定&使用

1. 发布手眼姿态矩阵

   ```bash
   cd ~/catkin_ws #cd where you put your "panda_client.sh"
   source panda_client.sh 
   roslaunch panda_hand_eye_calibrate publish_panda_eob.launch
   ```

2. 解锁panda机械臂，并启动Moveit

   ```bash
   cd ~/catkin_ws #cd where you put your "panda_client.sh"
   source panda_client.sh -r
   # ctrl+c
   roslaunch panda_moveit_config zzu_panda_move.launch
   ```

## TroubleShooting

   如果tf出现问题，就先重新启动Moveit步骤；

<img src="/pic/image-20210312164554656.png" alt="image-20210312164554656" style="zoom: 50%;" />

<img src="/pic/image-20210312164627240.png" alt="image-20210312164627240" style="zoom: 80%;" />