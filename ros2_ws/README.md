# ROS 2 workspace for HTS -> JAKA hand teleoperation

本工作空间已包含全部 ROS 2 依赖包，可独立交付、编译：

- `hand_tracking_robot`：HTS 数据接收与话题发布。
- `jaka_driver`：JAKA 双臂 `rc_ctrl_node`。
- `jaka_msgs`：JAKA 消息与服务定义。

控制机械臂前，先把 JAKA SDK 动态库放入：

```bash
cp /path/to/libjakaAPI.so src/jaka_driver/sdk_lib/libjakaAPI.so
```

然后编译：

```bash
cd ros2_ws
colcon build --symlink-install
source install/setup.bash
```

安全测试步骤、运行方式和参数说明详见 `src/hand_tracking_robot/README.md`。
