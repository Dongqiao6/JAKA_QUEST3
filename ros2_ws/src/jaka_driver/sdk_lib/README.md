# JAKA SDK 动态库

将 JAKA 官方提供的 `libjakaAPI.so` 放到本目录：

```text
ros2_ws/src/jaka_driver/sdk_lib/libjakaAPI.so
```

然后重新编译：

```bash
cd ros2_ws
colcon build --symlink-install --packages-up-to jaka_driver
```

没有该动态库时，整个工作空间仍然可以编译，
`jaka_driver` 只安装 launch 文件、不生成 `rc_ctrl_node`；
此时可先完成 `hand_tracking_bridge` 的无机械臂测试。
