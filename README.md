# Mobile Robot with VR : Meta Mobility

<br/>

## Equipments
- Oculus Quest 2

![oculus](https://user-images.githubusercontent.com/68265609/174251242-36cc5d86-006e-43ec-a0e5-6c46ebeed3bd.jpeg)
- C920 Camera
- VLP-16 or YD-LiDAR ... any 3D or 2D LiDAR is ok...
- WIFI router
- Any Mobile Robot
- ~Ricoh Theta V~

## Description

1. Navigation Package is based on ROS move_base.
<p align='left'>
    <img src="/gif/rviz.gif" width="400"/>
    <img src="/gif/nav.gif" width="400"/>
</p>

2. This repo supports teleoperation with Oculus Quest 2 (Manual Control Driving + VR).
<p align='left'>
    <img src="/gif/ocu.gif" width="500"/>
</p>

3. Combined two camera scenes are connected to Oculus Quest 2.
<p align='left'>
    <img src="/gif/cam.gif" width="500"/>
</p>

### VR Environments
- If you want to make an application of Oculus Quest 2 or any other VR devices. You have to develop it yourself by UNITY.
- We also make a [manual](https://github.com/kws6081/Mobile-Robot-with-VR/blob/main/vr_unity.md) how to make an VR application. It supports ROS communication with VR devices.
- There is a additional [manul](https://github.com/kws6081/Mobile-Robot-with-VR/blob/main/360.md) how to use 360 degree camera in UNITY, such as RICOH Theta V.


![1655452078848](https://user-images.githubusercontent.com/68265609/174252208-a428a4f7-ea34-4154-8f5d-e6786b6166d2.gif)


## How to use
### Build
```bash
cd ~/catkin_ws/src
git clone {this_repo}
cd ..
catkin_make
```

### WIFI settings
- First of all, check your IP address as follows:
```bash
hostname -I
```

- Then modify your IP address parameter of rosbridge_websocket.launch file in rosbridge_server package.

### Map settings
- Modify velodyne's frame_id from `velodyne` to `base_scan`.
- Modify navigation output topic message's name from `cmd_vel` to `navigation_cmd_vel`.

### Launch file
```bash
roslaunch usb_cam usb_cam_double.launch
roslaunch turtlebot3_navigation turtlebot3_navigation.launch
roslaunch rosbridge_server rosbridge_websocket.launch
roslaunch {your robot launch file}
```

- In our repo, the robot launch file as follows:
```bash
roslaunch md md.launch
```
