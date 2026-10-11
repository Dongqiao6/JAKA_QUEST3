# Quest3 手部骨骼点 -> JAKA 双臂遥操作

本工程用 Meta Quest 3 上的 `Hand Tracking Streamer`（HTS）识别手部骨骼点，通过
UDP 发送到 Ubuntu，再由 ROS 2 把**手部位移**转换成 JAKA 双臂的增量笛卡尔伺服。

默认参数按“先慢后快、先单臂后双臂”配置。实际遥操作链路：

```text
Quest3 HTS App
  -> UDP:9000
  -> hand_tracking_bridge（解析手腕/骨骼点，发布 PoseStamped + Joy）
  -> rc_ctrl_left / rc_ctrl_right（登录/上电/使能 JAKA，执行 servo_p）
  -> JAKA 双臂
```

## 目录结构

```text
hand-tracking-streamer-main/
├── hand_tracking_streamer.apk          # Quest3 安装包
├── run_quest_teleop.sh                 # 自动启动/停止 Quest APK 的遥操作包装脚本
├── hand_tracking_streamer/             # Unity 工程（如不需要可只交付 APK）
└── ros2_ws/
    └── src/
        ├── hand_tracking_robot/        # HTS 解析、话题发布
        ├── jaka_driver/                # rc_ctrl_node + PGI 夹爪驱动
        └── jaka_msgs/                  # JAKA 消息/服务定义
```

## 环境要求

- Ubuntu 22.04 + ROS 2 Humble。
- Python `numpy`、`pytest`。
- JAKA SDK 动态库 `libjakaAPI.so`。
- Quest3 已安装 `hand_tracking_streamer.apk`，并通过 `adb` 可访问。

## 1. Quest3 APK：安装、启动、停止

### 安装

```bash
adb install hand_tracking_streamer.apk
```

### 手动启动 APK

```bash
adb shell monkey -p com.wengmister.handtrackingstreamer 1
```

### 查找包名

```bash
adb shell pm list packages | grep handtracking
```

### 强制停止 APK

```bash
adb shell am force-stop com.wengmister.handtrackingstreamer
```

### 自动启动/停止

本工程提供了 `run_quest_teleop.sh`。它会先启动 Quest APK，运行你传入的命令；
命令正常结束或按 `Ctrl-C` 中断时，自动强制停止 Quest APK。

```bash
./run_quest_teleop.sh ros2 launch hand_tracking_robot dual_arm_hand_teleop.launch.py \
  robot_left_ip:=192.168.71.37 \
  robot_right_ip:=192.168.71.36 \
  enable_left:=true enable_right:=true \
  motion_scale:=0.40 \
  use_rpy_ctrl:=false \
  enable_y_limit:=true
```

也可以把它包在任意命令外面：

```bash
./run_quest_teleop.sh ros2 launch jaka_driver RC_ctrl.launch.py ...
```

如果你不想使用脚本，也可以手动执行上面的“启动 APK”和“强制停止 APK”命令。

## 2. 准备 JAKA SDK 并编译

先补齐 JAKA 官方动态库：

```bash
# 在已装 JAKA SDK 的机器上查找
find /usr /opt /home -name 'libjakaAPI.so*' 2>/dev/null

# 复制到指定位置
cp /path/to/libjakaAPI.so \
  ros2_ws/src/jaka_driver/sdk_lib/libjakaAPI.so
```

然后编译：

```bash
source /opt/ros/humble/setup.bash
cd ros2_ws
colcon build --symlink-install
source install/setup.bash
```

没有 `libjakaAPI.so` 时也能编译，但不会生成 `rc_ctrl_node`；要控制机械臂必须补上
SDK 并重新编译。每次重新编译后都要重新 `source install/setup.bash`。

如果只改了 `hand_tracking_robot` 里的 Python 代码或 launch 文件，可以只编译这个包：

```bash
cd ros2_ws
colcon build --symlink-install --packages-select hand_tracking_robot
source install/setup.bash
```

## 3. 先做无机械臂测试

```bash
cd ros2_ws
source install/setup.bash
python3 -m pytest src/hand_tracking_robot/test/test_hts_parser.py
```

终端 A 启动桥接：

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

确认 `left_fresh/right_fresh`、`left_pinch/right_pinch` 和两个位姿话题正常。
这一步没有启动 `rc_ctrl_node`，不会驱动机械臂。

注意：`fake_hand_stream.py` 目前不发 `Head pose`，所以此时日志会显示 `head=--`；
要验证 head pose 日志需要用 Quest 真机并打开 Head Pose / TrackHeadPose。

### 日志文件

`hand_tracking_bridge` 的日志会同时输出到终端和本地文件，默认路径：

```text
ros2_ws/logs/hand_tracking_bridge.log
```

每次进程启动都会**覆盖**上一次的日志内容。也可以启动时指定其他路径：

```bash
ros2 launch hand_tracking_robot hand_tracking_bridge.launch.py \
  log_file:=logs/my_hand_bridge.log
```

## 4. 真机顺序：先左臂，再右臂，最后双臂

真机测试必须遵守：

1. 机械臂工作范围内无人、无障碍，急停随时可及。
2. 一人操作，另一人监护急停。
3. 手指保持完全张开，只有准备移动机械臂时才捏合。
4. 机械臂进入低速伺服前，不要开始 Quest HTS 传输。

### 4.0 修改后的映射模式怎么运行

`hand_tracking_bridge` 现在支持 `mapping_mode`，默认仍是 `world_wrist`，所以不传
参数时行为和以前完全一样：

```text
world_wrist                 # 默认：腕部 Unity 世界坐标增量
head_relative_translation   # 减去头显位置，保留 Unity 世界轴
head_relative_local         # 再旋转到头显局部系，前后/左右更直观
```

桥接节点单独运行时切换：

```bash
ros2 launch hand_tracking_robot hand_tracking_bridge.launch.py \
  enable_left:=true enable_right:=true \
  mapping_mode:=head_relative_translation \
  head_timeout:=1.0 \
  motion_scale:=0.40 smooth_alpha:=0.45
```

一键双臂运行时切换：

```bash
./run_quest_teleop.sh ros2 launch hand_tracking_robot dual_arm_hand_teleop.launch.py \
  robot_left_ip:=192.168.71.37 \
  robot_right_ip:=192.168.71.36 \
  enable_left:=true enable_right:=true \
  mapping_mode:=head_relative_local \
  head_timeout:=1.0 \
  motion_scale:=0.40 smooth_alpha:=0.45 \
  use_rpy_ctrl:=false \
  enable_y_limit:=true
```

使用 `head_relative_translation` 或 `head_relative_local` 时，必须在 HTS App 中
打开 **Head Pose / TrackHeadPose**。头部位姿约 30 Hz，桥接节点会打印：

```text
mapping=head_relative_translation head=(x,y,z)
```

如果没收到头部位姿，会显示 `head=--`，并自动回退到 `world_wrist`。

### 4.1 单左臂

终端 A：

```bash
source ros2_ws/install/setup.bash
ros2 launch hand_tracking_robot hand_tracking_bridge.launch.py \
  enable_left:=true enable_right:=false \
  motion_scale:=0.40 smooth_alpha:=0.45
```

终端 B：

```bash
source ros2_ws/install/setup.bash
ros2 launch jaka_driver RC_ctrl.launch.py \
  robot_left_ip:=192.168.71.37 \
  robot_right_ip:=192.168.71.36 \
  use_rpy_ctrl:=false \
  enable_y_limit:=true \
  servo_filter_mode:=joint \
  max_joint_vel:=40.0 \
  max_joint_acc:=80.0
```

确认日志出现：

```text
left login OK
left power_on OK
left enable_robot OK
left servo filter=joint vj=40.0 deg/s aj=80.0 deg/s^2 ...
left servo enabled
```

然后打开 Quest HTS App：

1. 手模式选 `Left`。
2. 协议选 `UDP`，IP 填 Ubuntu 有线网卡 IP，端口 `9000`。
3. 建议关闭 Debug Info。
4. 如果使用 head-relative 映射，打开 **Head Pose / TrackHeadPose**。
5. 手指完全张开，开始传输。
6. 小幅度捏合并移动手腕，确认方向和急停。

### 4.2 单右臂

关闭上一组进程，桥接改成：

```bash
ros2 launch hand_tracking_robot hand_tracking_bridge.launch.py \
  enable_left:=false enable_right:=true \
  motion_scale:=0.40 smooth_alpha:=0.45
```

Quest HTS App 手模式选 `Right`，其余相同。

### 4.3 双臂

```bash
./run_quest_teleop.sh ros2 launch hand_tracking_robot dual_arm_hand_teleop.launch.py \
  robot_left_ip:=192.168.71.37 \
  robot_right_ip:=192.168.71.36 \
  enable_left:=true enable_right:=true \
  motion_scale:=0.40 smooth_alpha:=0.45 \
  use_rpy_ctrl:=false \
  enable_y_limit:=true \
  servo_filter_mode:=joint \
  max_joint_vel:=40.0 \
  max_joint_acc:=80.0
```

Quest HTS App 手模式选 `Dual/Both`。确认左右 `pinch` 只在自己捏合时变 `1`。

## 5. 遥操作映射：手怎么动，机械臂怎么动

### 5.1 控制点是腕部

默认 `position_source:=wrist`，所以驱动机械臂的是 **腕部的三维位置**，不是手指的
21 个关节角。21 个手部骨骼点中：

- 腕部 `wrist`：作为机械臂末端控制点。
- 拇指尖和食指尖：只用于判断是否捏合。
- 其他手指关节：默认不参与运动。

也可以把控制点改成食指指尖或捏合中点：

```bash
position_source:=index_tip
# 或
position_source:=pinch_midpoint
```

### 5.2 是三维坐标映射，不是关节角映射

HTS 的 `wrist` 是 Unity 左手系世界坐标，记为 `(x_u, y_u, z_u)`。桥接节点把它
映射到 JAKA/ROS 右手系：

```text
默认 unity_to_robot:
robot_x = -z_u
robot_y = -x_u
robot_z =  y_u
```

再乘 `motion_scale`，最后叠加静态 `tf_rotation` / `tf_translation`。

因此**机械臂移动的是映射后的腕部“位置变化量”**，不是 Quest 世界里的绝对坐标。
`rc_ctrl_node` 在捏合开始时刻记录原点，之后每个周期只发送：

```text
当前映射位置 - 上一次已执行映射位置
```

再经过死区、单步限幅和 JAKA 伺服滤波。

### 5.3 实际操作时手怎么移动

默认 `motion_scale=0.40`，即手移动 10 cm，机械臂移动约 4 cm。

默认 `use_rpy_ctrl=false`，所以：

- 手的**旋转不控制**机械臂姿态；
- 只移动手腕做**平移**，机械臂末端做对应方向的平移。

按照上面默认映射公式，在 Quest 世界轴不变的情况下：

| 手/Quest 方向 | 机械臂方向 |
| --- | --- |
| Unity +Z（通常前方） | 机械臂 -X |
| Unity +X（通常右方） | 机械臂 -Y |
| Unity +Y（向上） | 机械臂 +Z |

实际方向还会受 `tf_rotation`、机械臂安装方向和 Quest 标定影响，第一次真机先用
极小幅度确认方向，再逐步增大动作。

### 5.4 可切换的映射模式

桥接节点新增了 `mapping_mode`，当前默认仍是 `world_wrist`，不会改变现有行为：

```text
world_wrist                 # 原行为：腕部 Unity 世界坐标增量
head_relative_translation   # 只减去头显位置，仍保持 Unity 世界轴
head_relative_local         # 旋转到头显局部系，前后/左右更直观
```

切换示例：

```bash
mapping_mode:=head_relative_translation
# 或
mapping_mode:=head_relative_local
```

使用后两种模式前，需要在 HTS App 中打开 **Head Pose / TrackHeadPose**，否则
桥接节点会因头部位姿缺失自动回退到 `world_wrist`。头部位姿当前约 30 Hz，手部约
100 Hz；实验阶段建议先只记录数据，确认 `head=(x,y,z)` 和相对向量稳定后再真机。

### 5.5 左右臂镜像安装的基座旋转

桥接节点会先把 Quest 手部位姿转换到统一的机器人世界坐标，再按每只臂自己的旋转
转到该臂基座坐标。默认与 RemoteCtrl 的 quest_reader 一致：左右手使用同一套坐标
映射，不额外做镜像。

新增参数：

```text
left_arm_rotation   # 左臂基座旋转 [w,x,y,z]，默认 [1,0,0,0] = 不旋转
right_arm_rotation  # 右臂基座旋转 [w,x,y,z]，默认 [1,0,0,0] = 不旋转
```

只启动桥接节点时可以这样显式指定：

```bash
ros2 launch hand_tracking_robot hand_tracking_bridge.launch.py \
  enable_left:=true enable_right:=true \
  left_arm_rotation:='[1.0,0.0,0.0,0.0]' \
  right_arm_rotation:='[1.0,0.0,0.0,0.0]'
```

一键双臂运行时同样可以覆盖：

```bash
./run_quest_teleop.sh ros2 launch hand_tracking_robot dual_arm_hand_teleop.launch.py \
  robot_left_ip:=192.168.71.37 \
  robot_right_ip:=192.168.71.36 \
  enable_left:=true enable_right:=true \
  left_arm_rotation:='[1.0,0.0,0.0,0.0]' \
  right_arm_rotation:='[1.0,0.0,0.0,0.0]'
```

如果左臂方向仍然不对，可以尝试绕 Z 轴 180°：

```bash
left_arm_rotation:='[0.0,0.0,0.0,1.0]'
```

或绕 Y 轴 180°：

```bash
left_arm_rotation:='[0.0,0.0,1.0,0.0]'
```

验证时建议先只开左臂、用极小的 `motion_scale` 做慢速测试：

```bash
ros2 launch hand_tracking_robot hand_tracking_bridge.launch.py \
  enable_left:=true enable_right:=false \
  motion_scale:=0.20 \
  left_arm_rotation:='[1.0,0.0,0.0,0.0]'
```

## 6. 关节 4 大幅度摆动的原因和修改

当前遥操作只发送笛卡尔**平移增量**，姿态增量为零。JAKA 每个周期在内部求解逆运动学；
当 TCP 经过奇异点附近、或腕部/肘部处于某些构型时，一个很小的末端平移就可能要求
某个关节大幅旋转，最明显就是关节 4。

之前代码只做了**笛卡尔**速度/加速度限制，没有限制**关节**速度。因此 TCP 速度虽然
不高，但关节 4 仍可能突然快速摆动。

现在默认改成：

```text
servo_filter_mode := joint
max_joint_vel    := 40.0 deg/s
max_joint_acc    := 80.0 deg/s^2
max_joint_jerk   := 160.0 deg/s^3
```

这样每个关节的速度、加速度、加加速度都被限制，关节 4 不会突然猛甩。

如果仍觉得不自然，可以继续调低：

```bash
servo_filter_mode:=joint \
max_joint_vel:=20.0 \
max_joint_acc:=40.0 \
max_joint_jerk:=80.0
```

不建议在遥操作中改成 `servo_filter_mode:=cartesian`，因为它只限制 TCP，无法抑制
奇异点附近的关节突变。

## 7. 常用参数

### 桥接参数

| 参数 | 默认值 | 说明 |
| --- | --- | --- |
| `protocol` | `udp` | `udp` 或 `tcp` |
| `host` | `0.0.0.0` | 监听地址 |
| `port` | `9000` | 监听端口 |
| `publish_rate` | `100.0` | 发布频率 Hz |
| `position_source` | `wrist` | `wrist` / `index_tip` / `pinch_midpoint` |
| `motion_scale` | `0.40` | 手部位移到机械臂位移比例 |
| `smooth_alpha` | `0.45` | EMA 滤波系数，0 关闭 |
| `pinch_threshold` | `0.035` | 捏合判定距离 m |
| `pinch_release_ratio` | `1.5` | 松开滞回倍数 |
| `tracking_timeout` | `1.0` | 手部丢失判定时间 s |
| `enable_left` / `enable_right` | `true` | 是否发布左/右臂目标 |
| `mapping_mode` | `world_wrist` | `world_wrist` / `head_relative_translation` / `head_relative_local` |
| `head_timeout` | `1.0` | 头部位姿超时后 head-relative 模式自动回退 |
| `log_file` | `logs/hand_tracking_bridge.log` | 本地日志文件；每次启动覆盖上一次内容 |
| `tf_translation` | `[0,0,0]` | Quest 原点在机器人基座下位置 |
| `tf_rotation` | `[1,0,0,0]` | Quest 原点在机器人基座下姿态 `[w,x,y,z]` |
| `left_arm_rotation` | `[1,0,0,0]` | 左臂基座旋转 `[w,x,y,z]`；默认无旋转，与 RemoteCtrl 一致 |
| `right_arm_rotation` | `[1,0,0,0]` | 右臂基座旋转 `[w,x,y,z]`；默认无旋转 |
| `unity_to_robot` | `[0,0,-1,-1,0,0,0,1,0]` | Unity 左手系到 ROS 右手系 3x3 映射 |

### 机械臂运动参数

| 参数 | 默认值 | 说明 |
| --- | --- | --- |
| `servo_filter_mode` | `joint` | `joint` 推荐；`cartesian` 只限制 TCP |
| `servo_tran_deadband` | `0.25` | 平移死区 mm |
| `servo_rot_deadband` | `0.005` | 姿态死区 rad |
| `servo_max_tran_step` | `0.75` | 每个 `servo_p` 周期最大平移增量 mm |
| `servo_max_rot_step` | `0.005` | 每个 `servo_p` 周期最大姿态增量 rad |
| `max_joint_vel` | `40.0` | 关节速度上限 deg/s |
| `max_joint_acc` | `80.0` | 关节加速度上限 deg/s² |
| `max_joint_jerk` | `160.0` | 关节加加速度上限 deg/s³ |
| `max_cart_vel` | `80.0` | `cartesian` 模式下 TCP 速度上限 mm/s |
| `max_cart_acc` | `80.0` | `cartesian` 模式下 TCP 加速度上限 mm/s² |
| `max_cart_jerk` | `160.0` | `cartesian` 模式下 TCP 加加速度上限 mm/s³ |

## 8. 安全提示

- `rc_ctrl_node` 启动成功后会自动登录、上电、使能并进入伺服模式。
- 软件限速、捏合死停、手部丢失保持、双臂中线限制都只是辅助保护，不能替代
  物理急停、安全围栏和人员监护。
- 第一次真机从 `motion_scale:=0.40`、`servo_filter_mode:=joint` 开始，确认方向后再调整。
- 松手捏合只是保持原位；彻底退出请 `Ctrl-C` 关闭 `rc_ctrl_node`，并确认示教器状态。

## 9. 常见问题

### `Package 'jaka_driver' not found`

通常是没有 source overlay：

```bash
source /opt/ros/humble/setup.bash
cd ros2_ws
colcon build --symlink-install
source install/setup.bash
ros2 pkg prefix jaka_driver
```

### `rc_ctrl_node` 不存在

说明 `libjakaAPI.so` 没有放入 `ros2_ws/src/jaka_driver/sdk_lib/`，补齐后重新编译。

### Quest 数据收不到

确认 Quest 与 Ubuntu 在同一网段，防火墙放行：

```bash
sudo ufw allow 9000/udp
```

确认桥接日志出现 `listening for HTS UDP on 0.0.0.0:9000`。
