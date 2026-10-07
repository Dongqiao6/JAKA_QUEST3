# JAKA SDK 动态库

`libjakaAPI.so` 是 JAKA 官方提供的**编译好的动态链接库**（类似 Windows 的 DLL），
里面包含登录、上电、使能、`servo_p` 等机械臂控制的实现。我们仓库里的
`JAKAZuRobot.h` 只是函数声明，**没有对应的实现源码**，所以无法通过编译生成这个
`.so`，必须从 JAKA 官方 SDK 安装包中复制。

获取方式：

- JAKA 官网/技术支持的“二次开发 SDK”（Readme 中版本为 `2.2.2`）安装包。
- 已安装过 JAKA SDK 的电脑，可用下面命令查找：

```bash
find /usr /opt /home -name 'libjakaAPI.so*' 2>/dev/null
```

找到后，把该文件复制到本目录：

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
