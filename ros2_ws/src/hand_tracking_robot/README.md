# hand_tracking_robot

把 Meta Quest 3 的 **视觉手部骨骼点**（Hand Tracking Streamer，HTS）转换成 JAKA
双臂机械臂的笛卡尔伺服目标。

本项目只新增在 `hand-tracking-streamer-main` 内，复用了
`Jaka_VR-main` 中已经验证过的 `rc_ctrl_node`：Quest 侧只负责发手部骨骼数据，
Ubuntu 侧由本包解析并发布 `PoseStamped` / `Joy`，`rc_ctrl_node` 负责真正登录、
上电、使能机械臂并调用 JAKA SDK 的 `servo_p`。

## 目录结构

```text
hand-tracking-streamer-main/ros2_ws/src/hand_tracking_robot/
├── hand_tracking_robot/
│   ├── __init__.py
│   ├── hts_parser.py              # HTS 数据解析、四元数/坐标变换
│   └── hand_tracking_bridge_node.py  # ROS2 节点：UDP 接收 -> PoseStamped/Joy
├── launch/
│   ├── hand_tracking_bridge.launch.py        # 只启动桥接节点
│   └── dual_arm_hand_teleop.launch.py        # 桥接 + JAKA 双臂 rc_ctrl
├── config/
│   └── hand_tracking_bridge.yaml             # 参数示例
├── test/
│   ├── fake_hand_stream.py                   # 无 Quest 时的假数据发送器
│   └── test_hts_parser.py                    # 解析器单元测试
├── package.xml
├── setup.py
└── setup.cfg
```

## 原理

HTS 每个 UDP 报文里包含两类行：

- `Left/Right wrist:` 7 个浮点：`x, y, z, qx, qy, qz, qw`（Unity 世界坐标）。
- `Left/Right landmarks:` 63 个浮点：21 个关节的 `x, y, z`，相对腕部。

桥接节点做的事情：

1. 监听 UDP（默认 `0.0.0.0:9000`），解析左右手。
2. 用腕部位姿（也可配置成食指指尖/捏合中点）作为机械臂末端目标点。
3. 用 **拇指尖-食指尖距离** 判断是否“捏合”。捏合相当于 JAKA 遥控里的“按住侧键”，
   机械臂进入随动；松开即保持原位。
4. 把 Unity 左手系坐标映射到 JAKA/ROS 右手系，默认与
   `Jaka_VR-main/src/quest_vr/quest_reader.py` 一致：`(x,y,z) -> (-z,-x,y)`。
5. 以 `100 Hz` 发布：
   - `geometry_msgs/PoseStamped` 到 `/rc_ctrl/left_target`、`/rc_ctrl/right_target`
   - `sensor_msgs/Joy` 到 `/rc_ctrl/button`（`axes[0]` 左手捏合、`axes[1]` 右手捏合）

JAKA 侧沿用现有 `rc_ctrl_node`：捏合上升沿记录机械臂当前位置为原点，之后按手柄/手部
位移做增量笛卡尔伺服；松手后停止。默认 `use_rpy_ctrl=false`，只做三维平移。

## 环境要求

- Ubuntu 22.04 + ROS 2 Humble。
- 已按 `Jaka_VR-main/Readme.md` 编译并安装 `jaka_driver`、`jaka_msgs` 等包。
- Python `numpy`。
- Quest 3 已安装 HTS（`hand_tracking_streamer.apk`），并与 Ubuntu 主机有线网络
  连通（本工程实际使用 UDP 有线稳定通信）。

## 编译

```bash
cd /home/carrie/my_project/hand-tracking-streamer-main/ros2_ws
colcon build --symlink-install
source install/setup.bash

# 再 source JAKA 工作空间（按你的实际路径）
source /home/carrie/my_project/Jaka_VR-main/install/setup.bash
```

> 两个工作空间都要 source；本包的 `dual_arm_hand_teleop.launch.py` 需要找到
> `jaka_driver` 的 launch 文件。若没有编译过 JAKA 工作空间，先按
> `Jaka_VR-main/Readme.md` 编译。

## 运行

### 方式一：只启动桥接节点（推荐先测试）

```bash
ros2 launch hand_tracking_robot hand_tracking_bridge.launch.py
```

然后在另一个终端启动 JAKA 双臂控制（IP 参考 Jaka_VR-main，默认左 `192.168.71.37`、
右 `192.168.71.36`）：

```bash
ros2 launch jaka_driver RC_ctrl.launch.py \
  robot_left_ip:=192.168.71.37 \
  robot_right_ip:=192.168.71.36 \
  use_rpy_ctrl:=false \
  enable_y_limit:=true
```

### 方式二：一键启动（桥接 + 双臂）

```bash
ros2 launch hand_tracking_robot dual_arm_hand_teleop.launch.py \
  robot_left_ip:=192.168.71.37 \
  robot_right_ip:=192.168.71.36
```

## Quest 3 上的 HTS 设置

1. 打开 HTS App，网络协议选 **UDP**。
2. 目标 IP 填 Ubuntu 主机的有线网卡 IP。
3. 端口填 `9000`（与启动参数一致）。
4. 手模式选 **Dual/Both**（双臂都控制）。
5. 建议关闭 Debug Info 避免报文头变化（本包两种格式都能解析，但普通格式更省带宽）。
6. 点击开始传输。

> 有线场景下应确认 Quest 与 Ubuntu 在同一网段，且防火墙放行 UDP 9000：
> `sudo ufw allow 9000/udp`。

## 操作方式

- **左手控制左臂，右手控制右臂。**
- 做“捏合”（拇指尖碰食指）**进入控制**，移动手部，机械臂末端做三维平移随动。
- **松开捏合**，机械臂保持当前位置。
- 手部丢失跟踪超过 `tracking_timeout` 会自动释放捏合信号，机械臂保持。
- 想改变手的敏感度：调 `motion_scale`；想更平滑：调大/调小 `smooth_alpha`。

## 常用参数

| 参数 | 默认值 | 说明 |
| --- | --- | --- |
| `protocol` | `udp` | `udp` 或 `tcp` |
| `host` | `0.0.0.0` | 监听地址 |
| `port` | `9000` | 监听端口 |
| `publish_rate` | `100.0` | 发布频率 Hz |
| `position_source` | `wrist` | `wrist` / `index_tip` / `pinch_midpoint` |
| `motion_scale` | `1.0` | 手部位移到机械臂位移的比例 |
| `pinch_threshold` | `0.035` | 捏合判定距离（米） |
| `pinch_release_ratio` | `1.5` | 松开滞回倍数 |
| `tracking_timeout` | `1.0` | 多久无数据判定丢失（秒） |
| `smooth_alpha` | `0.45` | 一阶低通系数，0 关闭 |
| `tf_translation` | `[0,0,0]` | Quest 原点在机器人基座下位置 |
| `tf_rotation` | `[1,0,0,0]` | Quest 原点在机器人基座下姿态 `[w,x,y,z]` |
| `unity_to_robot` | `[0.0, 0.0, -1.0, -1.0, 0.0, 0.0, 0.0, 1.0, 0.0]` | Unity 左手系到 ROS 右手系 3x3 映射 |

例子：手移动 10 cm 只让机械臂移动 5 cm，且捏合更灵敏：

```bash
ros2 launch hand_tracking_robot hand_tracking_bridge.launch.py \
  motion_scale:=0.5 pinch_threshold:=0.05
```

## 无 Quest 的离线测试

先启动桥接节点，再运行假数据发送器：

```bash
# 终端 1
ros2 launch hand_tracking_robot hand_tracking_bridge.launch.py

# 终端 2
python3 ./test/fake_hand_stream.py \
  --host 127.0.0.1 --port 9000 --pinch --both-hands
```

然后可以用 `ros2 topic echo /rc_ctrl/left_target` 查看目标位姿（不启动真机械臂时
`rc_ctrl_node` 会登录失败，但桥接发布不受影响）。

## 安全提示

- 第一次测试请让机械臂处于低速、空载、急停可及的状态。
- 建议先用 `motion_scale:=0.3` 等较小比例验证方向后再逐步增大。
- 默认 `enable_y_limit:=true` 只防止双臂越中线，不能替代安全围栏和碰撞检测。
