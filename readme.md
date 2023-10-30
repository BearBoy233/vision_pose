### Vision_pose

将**外部视觉里程计**信息作为**定位基准**通过**MAVROS**发送给飞控.

[REF px4/external_position_estimation](https://docs.px4.io/main/en/ros/external_position_estimation.html)

### Quick Start

**Requirements**: Ubuntu LTS (16.04/18.04/20.04) with ros-desktop-full installation.

**Dependency package**: vrpn-client-ros

```
sudo apt install ros-noetic-vrpn-client-ros
```

**Run&Roslaunch**:

```
# VRPN (If necessary)
roslaunch vision_pose vrpn_client_ros_sample.launch
# vision_pose
roslaunch vision_pose vision_pose.launch
```

### Param definition

**flag_1vrpn_2vio_3both**: FLAG 选择要发给飞控的数据
```
1 -只用 VRPN 定位
2 -只用 机载视觉里程计 定位
3 -优先使用 机载视觉里程计 定位，当误差较大时切换为 VRPN 定位
```

**my_id**: ??? 无人机编号
```
订阅的 VRPN 话题  "/vrpn_client_node/px4_uav???/pose"
```

**vio_odomTopic**: 订阅的 视觉里程计 话题
```
订阅的 视觉里程计 话题
[t265]		/camera/odom/sample
[vins]		/vins_fusion/odometry
```

