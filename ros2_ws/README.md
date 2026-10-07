# ROS 2 workspace：Quest3 手部骨骼点 -> JAKA 双臂遥操作

本工作空间接收 Meta Quest 3 上 `Hand Tracking Streamer`（HTS）通过 UDP 发来的
手部骨骼数据，把它转换成 JAKA 双臂的**增量笛卡尔伺服**指令。

默认配置已按“先慢后快、先单臂后双臂”的安全原则调整：

- 手部位移到机械臂位移的比例 `motion_scale` 默认是 `0.40`，不是 1:1。
- 位置 EMA 滤波 `smooth_alpha` 默认是 `0.45`，兼顾跟手和抑制手抖。
- 机械臂笛卡尔速度上限 `80 mm/s`、加速度上限 `80 mm/s²`、加加速度上限 `160 mm/s³`。
- 每次 `servo_p` 的最大平移增量 `0.75 mm`、最大姿态增量 `0.005 rad`。
- 姿态随动 `use_rpy_ctrl` 默认关闭，第一次测试只做平移。

## 目录

- `hand_tracking_robot`：HTS UDP 接收、骨骼解析、`PoseStamped`/`Joy` 发布。
- `jaka_driver`：双臂 `rc_ctrl_node`，登录/上电/使能 JAKA 并执行 `servo_p`。
- `jaka_msgs`：JAKA 消息与服务定义。

## 0. 安装 Quest3 端 APK（只做一次）

`hand_tracking_streamer.apk` 位于仓库根目录。安装这一步骤只和 Quest3 有关，
不需要机械臂上电，可以在任何空闲时间完成：

```bash
# 1. Quest3 开启开发者模式，USB 连接电脑并授权
adb devices

# 2. 安装 APK
adb install hand_tracking_streamer.apk
```

安装后**先不要打开 HTS App**。App 会在你点击“开始传输”后持续发 UDP 包；
应在宿主机监听和机械臂安全流程都准备好之后再开始传输。

## 1. 准备 JAKA SDK 并编译

`libjakaAPI.so` 是 JAKA 官方闭源动态库，需要自行补齐：

```bash
# 在已装 JAKA SDK 的机器上查找
find /usr /opt /home -name 'libjakaAPI.so*' 2>/dev/null

# 放到指定位置
cp /path/to/libjakaAPI.so \
  ros2_ws/src/jaka_driver/sdk_lib/libjakaAPI.so

cd ros2_ws
colcon build --symlink-install
source install/setup.bash
```

没有 SDK 也能编译：此时 `jaka_driver` 只安装 launch 文件、不生成 `rc_ctrl_node`，
可先做无机械臂测试。

## 2. 先做无机械臂测试（不接臂、不打开 Quest）

```bash
cd ros2_ws
source install/setup.bash
python3 -m pytest src/hand_tracking_robot/test/test_hts_parser.py
```

终端 A 启动桥接节点：

```bash
source install/setup.bash
ros2 launch hand_tracking_robot hand_tracking_bridge.launch.py \
  enable_left:=true enable_right:=true motion_scale:=0.40
```

终端 B 发送假手数据：

```bash
source install/setup.bash
python3 src/hand_tracking_robot/test/fake_hand_stream.py \
  --host 127.0.0.1 --port 9000 --pinch --both-hands
```

终端 C 观察：

```bash
source install/setup.bash
ros2 topic echo /rc_ctrl/button
ros2 topic echo /rc_ctrl/left_target
ros2 topic echo /rc_ctrl/right_target
```

此阶段没有 `rc_ctrl_node`，不会给机械臂发任何运动命令。

如果想用真 Quest 代替上面的假数据做一次无臂验证，也请**先启动桥接节点**，确认日志
出现 `listening for HTS UDP` 后，再打开 HTS App、选 `Dual/Both` 并开始传输；手指
保持张开即可，此阶段只观察 `left_fresh/right_fresh` 和话题，不会驱动机械臂。

## 3. 真机安全顺序：先左臂、再右臂、最后双臂

每次真机测试都必须遵守：

1. 工作范围内无人、无障碍；示教器或外接急停随时可及。
2. 一人操作，另一人监护急停。
3. 先保持手指**完全张开**，只有准备移动机械臂时才做“拇指尖碰食指尖”的捏合。

### 3.1 单左臂

终端 A 启动桥接，只允许左臂目标：

```bash
source ros2_ws/install/setup.bash
ros2 launch hand_tracking_robot hand_tracking_bridge.launch.py \
  enable_left:=true enable_right:=false motion_scale:=0.40
```

终端 B 启动 JAKA 控制：

```bash
source ros2_ws/install/setup.bash
ros2 launch jaka_driver RC_ctrl.launch.py \
  robot_left_ip:=192.168.71.37 robot_right_ip:=192.168.71.36 \
  use_rpy_ctrl:=false enable_y_limit:=true \
  max_cart_vel:=80.0 max_cart_acc:=80.0
```

确认日志依次出现 `left login OK`、`left power_on OK`、`left enable_robot OK`、
`left servo enabled`，并且 `servo NLF limits` 显示低速参数。

此时再操作 Quest3：

1. 打开 HTS App，手模式选 **Left**。
2. 协议选 **UDP**，IP 填 Ubuntu 有线网卡 IP，端口 `9000`。
3. 建议关闭 Debug Info。
4. 双手/左手保持张开，点“开始传输”。
5. 小幅度、缓慢捏合并移动左手，确认机械臂方向、比例和松手保持都正确。

### 3.2 单右臂

关闭上一组进程后，桥接只允许右臂，Quest App 手模式改 **Right**：

```bash
ros2 launch hand_tracking_robot hand_tracking_bridge.launch.py \
  enable_left:=false enable_right:=true motion_scale:=0.40
```

`RC_ctrl.launch.py` 仍同时登录左右臂；只有右臂能收到目标并随动。

### 3.3 双臂

Quest App 手模式改 **Dual/Both**，两个桥接都打开：

```bash
ros2 launch hand_tracking_robot hand_tracking_bridge.launch.py \
  enable_left:=true enable_right:=true motion_scale:=0.40
```

或用一键启动：

```bash
ros2 launch hand_tracking_robot dual_arm_hand_teleop.launch.py \
  robot_left_ip:=192.168.71.37 robot_right_ip:=192.168.71.36 \
  enable_left:=true enable_right:=true motion_scale:=0.40 \
  use_rpy_ctrl:=false enable_y_limit:=true
```

确认 `left_pinch`/`right_pinch` 分别只在自己捏合时变为 `1`，且双臂不会越过中线。

## 遥操作映射

- **输入量**：HTS 每帧报告的腕部位姿（默认控制点 `wrist`）以及 21 个手部骨骼点。
- **映射到机械臂的东西**：机械臂末端移动的是**“捏合开始时刻”之后手部控制点位移的映射值**，
  不是 Quest 世界坐标中的绝对位置。默认把 Unity 的 `(x,y,z)` 映射为
  `(-z,-x,y)`，再乘 `motion_scale`，最后加静态 `tf_translation`/`tf_rotation`。
- **开始遥操作**：同一只手的拇指尖和食指尖距离小于 `pinch_threshold`（默认 0.035 m）
  时，`rc_ctrl_node` 记录当前机械臂 TCP 和当前手部目标为原点，随后开始增量伺服。
- **停止/保持**：松开捏合（两指尖距离大于 `pinch_threshold * pinch_release_ratio`），
  `rc_ctrl_node` 立即发送一次零增量 `servo_p` 并停止发增量指令，机械臂保持原位。
- **丢失保护**：手部丢失超过 `tracking_timeout`，桥接自动把捏合信号置 0，机械臂保持。

姿态 `use_rpy_ctrl=false` 时，只有手部**位移**驱动机械臂平移，手部旋转不会驱动姿态。

## 绝对停止

- 松手捏合：保持原位（不是下电）。
- 关闭 `rc_ctrl_node`：`Ctrl-C` 会退出伺服、断开登录；确认示教器状态。
- 紧急情况：按机械臂急停/安全门，不要依赖软件。

更完整的参数表、Quest 网络排错和分步检查见
`src/hand_tracking_robot/README.md`。
