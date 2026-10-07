# hand_tracking_robot

把 Meta Quest 3 的 **视觉手部骨骼点**（Hand Tracking Streamer，HTS）转换成 JAKA
双臂机械臂的笛卡尔伺服目标。

本项目已把遥操作所需的 ROS 2 包整合到同一个交付文件夹中，不再依赖外部的
`Jaka_VR-main`：

- `hand_tracking_robot`：UDP 接收 HTS 数据，解析手部骨骼并发布
  `PoseStamped` / `Joy`。
- `jaka_driver`：复用 JAKA 已验证的 `rc_ctrl_node`，负责登录、上电、使能机械臂
  并调用 JAKA SDK 的 `servo_p`。
- `jaka_msgs`：`rc_ctrl_node` 需要的 ROS 2 消息/服务定义。

## 交付目录

```text
hand-tracking-streamer-main/
├── hand_tracking_streamer.apk          # Quest 3 安装包
├── hand_tracking_streamer/             # Unity 工程（如不需要可只交付 APK）
└── ros2_ws/
    ├── README.md
    └── src/
        ├── hand_tracking_robot/        # HTS 桥接节点
        │   ├── hand_tracking_robot/
        │   ├── launch/
        │   ├── config/
        │   └── test/
        ├── jaka_driver/                # rc_ctrl_node + PGI 夹爪
        │   ├── include/                # JAKA SDK 头文件
        │   ├── launch/
        │   ├── sdk_lib/                # 放 libjakaAPI.so（需手动补齐）
        │   └── src/
        └── jaka_msgs/                  # JAKA 消息/服务
```

## 环境要求

- Ubuntu 22.04 + ROS 2 Humble。
- Python `numpy` 与 `pytest`。
- JAKA SDK 动态库 `libjakaAPI.so`（JAKA 官方闭源库，需自行放入
  `jaka_driver/sdk_lib/`，见 `jaka_driver/sdk_lib/README.md`）。
- Quest 3 已通过 `adb install hand_tracking_streamer.apk` 安装 HTS，并与 Ubuntu
  主机有线网络连通（实际使用 UDP 有线稳定通信）。HTS App 应在宿主机监听和机械臂
  安全流程准备好之后才打开/开始传输，不要在机械臂上电前提前点击开始。

## 首次编译

先把 JAKA SDK 动态库放到指定位置。如果手头没有，先在 JAKA 主机上查找：

```bash
find /usr /opt /home -name 'libjakaAPI.so*' 2>/dev/null
```

找到后复制到目标位置：

```bash
# 把 JAKA 官方提供的 libjakaAPI.so 复制进来
cp /path/to/libjakaAPI.so \
  ros2_ws/src/jaka_driver/sdk_lib/libjakaAPI.so
```

然后编译整个工作空间：

```bash
cd ros2_ws
colcon build --symlink-install
source install/setup.bash
```

没有 `libjakaAPI.so` 时也可以编译：`jaka_driver` 只安装 launch 文件、不生成
`rc_ctrl_node`，因此可以先完成桥接节点测试；要控制机械臂前必须补上 SDK 并重新编译。

## 原理

HTS 每个 UDP 报文里包含两类行：

- `Left/Right wrist:` 7 个浮点：`x, y, z, qx, qy, qz, qw`（Unity 世界坐标）。
- `Left/Right landmarks:` 63 个浮点：21 个关节的 `x, y, z`，相对腕部。

桥接节点做的事情：

1. 监听 UDP（默认 `0.0.0.0:9000`），解析左右手。
2. 用腕部位姿（也可配置成食指指尖/捏合中点）作为机械臂末端目标点。
3. 用 **拇指尖-食指尖距离** 判断是否“捏合”。捏合相当于 JAKA 遥控里的“按住侧键”，
   机械臂进入随动；松开即保持原位。
4. 把 Unity 左手系坐标映射到 JAKA/ROS 右手系，默认 `(x,y,z) -> (-z,-x,y)`。
5. 以 `100 Hz` 发布：
   - `geometry_msgs/PoseStamped` 到 `/rc_ctrl/left_target`、`/rc_ctrl/right_target`
   - `sensor_msgs/Joy` 到 `/rc_ctrl/button`（`axes[0]` 左手捏合、`axes[1]` 右手捏合）

JAKA 侧由 `rc_ctrl_node` 处理：捏合上升沿记录机械臂当前位置为原点，之后按手部位移
做增量笛卡尔伺服；松手后停止。默认 `use_rpy_ctrl=false`，只做三维平移。

## 安全测试流程

### 第 0 步：先测试代码，不接机械臂

在没有 Quest、没有机械臂时，先验证解析、话题、参数和 UDP 链路。

```bash
cd ros2_ws
source install/setup.bash

# 1. 单元测试（无 ROS 节点，也不会让机械臂运动）
python3 -m pytest src/hand_tracking_robot/test/test_hts_parser.py
```

启动桥接节点：

```bash
ros2 launch hand_tracking_robot hand_tracking_bridge.launch.py
```

在第二个终端运行假数据发送器：

```bash
cd ros2_ws
source install/setup.bash
python3 src/hand_tracking_robot/test/fake_hand_stream.py \
  --host 127.0.0.1 --port 9000 --pinch --both-hands
```

第三个终端检查输出：

```bash
source ros2_ws/install/setup.bash
ros2 topic echo /rc_ctrl/button
ros2 topic echo /rc_ctrl/left_target
ros2 topic echo /rc_ctrl/right_target
```

确认标准：

- 桥接日志出现 `listening for HTS UDP on 0.0.0.0:9000`。
- `left_fresh=True`、`right_fresh=True`。
- `left_pinch` / `right_pinch` 随 `--pinch` 或假数据在 0/1 间变化。
- 两个目标位姿话题持续发布，且没有异常退出。

这一步完全不启动 `rc_ctrl_node`，不会给机械臂发任何运动指令。

如果这一步想用真 Quest 代替假数据，先启动桥接节点，确认日志出现
`listening for HTS UDP` 后，再打开 HTS App 并开始传输；手指保持张开，只观察
`left_fresh/right_fresh` 和 `/rc_ctrl/left_target` 等话题即可。

### 第 1 步：安装并准备 Quest 3（机械臂不上电）

先把 APK 装进 Quest 3；这一步随时可以做：

```bash
adb install hand_tracking_streamer.apk
```

确认 Quest 与 Ubuntu 在同一网段，且防火墙放行 UDP 9000：

```bash
sudo ufw allow 9000/udp
```

**此时不要打开 HTS App，也不要开始传输。** App 只有在后面需要真机数据时才打开。

### 第 2 步：单臂低速空载测试（先左臂，再右臂）

只有第 0 步全部通过、并已补齐 `libjakaAPI.so` 重新编译后，再进行这一步。开始前：

- 双臂处于**空载、低速、急停可及**状态，工作范围内无人、无障碍物。
- 一人操作，另一人随时按急停。
- 机械臂未下电/未使能前，先不要打开 HTS App。

先只启动桥接节点，并**关闭右臂输出、保留低速比例和滤波**：

```bash
source ros2_ws/install/setup.bash
ros2 launch hand_tracking_robot hand_tracking_bridge.launch.py \
  enable_left:=true enable_right:=false \
  motion_scale:=0.30 smooth_alpha:=0.30
```

再启动 JAKA 控制，并显式使用低速限值：

```bash
ros2 launch jaka_driver RC_ctrl.launch.py \
  robot_left_ip:=192.168.71.37 \
  robot_right_ip:=192.168.71.36 \
  use_rpy_ctrl:=false \
  enable_y_limit:=true \
  servo_max_tran_step:=0.5 \
  max_cart_vel:=60.0 \
  max_cart_acc:=60.0
```

观察日志确认左臂依次出现：

```text
left login OK
left power_on OK
left enable_robot OK
left servo NLF limits: vp=60.0 mm/s ap=60.0 mm/s^2 ...
left servo enabled
```

确认机械臂已进入低速伺服、但尚未接收目标后，再操作 Quest：

1. 打开 HTS App，手模式选 **Left**。
2. 协议选 **UDP**，IP 填 Ubuntu 有线网卡 IP，端口 `9000`，建议关闭 Debug Info。
3. 手指**完全张开**，点击开始传输。
4. 缓慢做“拇指尖碰食指尖”捏合，并做很小幅度移动。
5. 确认方向、比例、松手保持和急停反应都正确后，再逐级增大 `motion_scale`
   和速度参数，不要一开始就调快。

右臂测试把桥接参数换成 `enable_left:=false enable_right:=true`，HTS App 手模式
选 **Right**，其余相同。

### 第 3 步：双臂联动测试

左右单臂都验证通过后，再进入双臂。HTS App 手模式选 **Dual/Both**，先保持双手
张开，等两个 `rc_ctrl` 进程都进入伺服后再捏合。

分开启动，便于查看日志：

```bash
# 终端 1：桥接，左右臂都发布目标
ros2 launch hand_tracking_robot hand_tracking_bridge.launch.py \
  enable_left:=true enable_right:=true \
  motion_scale:=0.30 smooth_alpha:=0.30

# 终端 2：JAKA 双臂，显式低速
ros2 launch jaka_driver RC_ctrl.launch.py \
  robot_left_ip:=192.168.71.37 \
  robot_right_ip:=192.168.71.36 \
  use_rpy_ctrl:=false \
  enable_y_limit:=true \
  servo_max_tran_step:=0.5 \
  max_cart_vel:=60.0 \
  max_cart_acc:=60.0
```

也可以一键启动：

```bash
source ros2_ws/install/setup.bash
ros2 launch hand_tracking_robot dual_arm_hand_teleop.launch.py \
  robot_left_ip:=192.168.71.37 \
  robot_right_ip:=192.168.71.36 \
  enable_left:=true enable_right:=true \
  motion_scale:=0.30 smooth_alpha:=0.30 \
  use_rpy_ctrl:=false \
  enable_y_limit:=true \
  servo_max_tran_step:=0.5 \
  max_cart_vel:=60.0 \
  max_cart_acc:=60.0
```

确认 `left_pinch` / `right_pinch` 分别只在自己捏合时变为 `1`，且双臂不会越过中线。

## 操作方式

- **左手控制左臂，右手控制右臂。**
- 做“捏合”（拇指尖碰食指）**进入控制**，移动手部，机械臂末端做三维平移随动。
- **松开捏合**，机械臂保持当前位置。
- 手部丢失跟踪超过 `tracking_timeout` 会自动释放捏合信号，机械臂保持。
- 想改变手的敏感度：调 `motion_scale`；想更平滑：调 `smooth_alpha`。

## 常用参数

| 参数 | 默认值 | 说明 |
| --- | --- | --- |
| `protocol` | `udp` | `udp` 或 `tcp` |
| `host` | `0.0.0.0` | 监听地址 |
| `port` | `9000` | 监听端口 |
| `publish_rate` | `100.0` | 发布频率 Hz |
| `position_source` | `wrist` | `wrist` / `index_tip` / `pinch_midpoint` |
| `motion_scale` | `0.30` | 手部位移到机械臂位移的比例，第一次真机保持低速 |
| `pinch_threshold` | `0.035` | 捏合判定距离（米） |
| `pinch_release_ratio` | `1.5` | 松开滞回倍数 |
| `tracking_timeout` | `1.0` | 多久无数据判定丢失（秒） |
| `smooth_alpha` | `0.30` | 一阶低通系数，0 关闭 |
| `tf_translation` | `[0,0,0]` | Quest 原点在机器人基座下位置 |
| `tf_rotation` | `[1,0,0,0]` | Quest 原点在机器人基座下姿态 `[w,x,y,z]` |
| `unity_to_robot` | `[0.0, 0.0, -1.0, -1.0, 0.0, 0.0, 0.0, 1.0, 0.0]` | Unity 左手系到 ROS 右手系 3x3 映射 |
| `enable_left` / `enable_right` | `true` | 是否发布左/右臂目标，单臂测试时关闭另一侧 |

`RC_ctrl.launch.py` 的机械臂运动限值：

| 参数 | 默认值 | 说明 |
| --- | --- | --- |
| `servo_tran_deadband` | `0.5` | 平移死区 mm，过滤手抖 |
| `servo_rot_deadband` | `0.005` | 姿态死区 rad |
| `servo_max_tran_step` | `0.5` | 每个 `servo_p` 周期最大平移增量 mm |
| `servo_max_rot_step` | `0.005` | 每个 `servo_p` 周期最大姿态增量 rad |
| `max_cart_vel` | `60.0` | 笛卡尔速度上限 mm/s |
| `max_cart_acc` | `60.0` | 笛卡尔加速度上限 mm/s² |
| `max_cart_jerk` | `120.0` | 笛卡尔加加速度上限 mm/s³ |
| `max_rot_vel` | `20.0` | 姿态速度上限 deg/s |
| `max_rot_acc` | `40.0` | 姿态加速度上限 deg/s² |
| `max_rot_jerk` | `80.0` | 姿态加加速度上限 deg/s³ |

## 安全提示

- `rc_ctrl_node` 启动成功后会**自动登录、上电、使能并进入伺服模式**；启动前必须确保
  机械臂工作范围内无人、无障碍。
- 第一次真机测试必须从 `motion_scale:=0.3` 开始，逐级确认方向后再增大。
- 代码默认已经把机械臂速度限制为 `60 mm/s`、加速度限制为 `60 mm/s²`，同时每次
  `servo_p` 平移增量不超过 `0.5 mm`；单臂测试通过前不要调大这些值。
- 默认 `enable_y_limit:=true` 只防止双臂越过中线，不能替代安全围栏、碰撞检测和急停。
- 松开捏合只会停止随动；若需要完全退出伺服/下电，请关闭 `rc_ctrl_node` 进程并
  使用机械臂示教器确认状态。
