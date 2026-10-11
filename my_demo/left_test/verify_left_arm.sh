#!/usr/bin/env bash
set -euo pipefail

# 左臂单臂验证脚本：Quest APK + hand_tracking_bridge + 左臂 rc_ctrl_node。
# 使用和 RemoteCtrl 类似的笛卡尔伺服参数，验证左臂姿态是否恢复正常。
#
# 运行前请确认：
#   - 没有其他 rc_ctrl_node 占用左臂
#   - 机械臂工作范围内无人、无障碍，急停随时可及
# 启动后按 Ctrl-C 会停止 APK 和两个 ROS 节点。

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/../.." && pwd)"

ADB="${ADB:-adb}"
PKG="com.wengmister.handtrackingstreamer"
LEFT_IP="${LEFT_IP:-192.168.71.37}"
MOTION_SCALE="${MOTION_SCALE:-0.40}"

BRIDGE_PID=""
CTRL_PID=""

cleanup() {
  echo "[verify-left] stopping ..."
  if [ -n "${CTRL_PID}" ]; then kill "${CTRL_PID}" 2>/dev/null || true; fi
  if [ -n "${BRIDGE_PID}" ]; then kill "${BRIDGE_PID}" 2>/dev/null || true; fi
  "${ADB}" shell am force-stop "${PKG}" >/dev/null 2>&1 || true
}

trap cleanup EXIT
trap 'exit 130' INT TERM

source /opt/ros/humble/setup.bash
source "${REPO_ROOT}/ros2_ws/install/setup.bash"

echo "[verify-left] launching Quest APK ..."
"${ADB}" shell monkey -p "${PKG}" 1 >/dev/null 2>&1 || {
  echo "[verify-left] could not launch APK; check adb" >&2
  exit 1
}

echo "[verify-left] starting hand_tracking_bridge ..."
ros2 launch hand_tracking_robot hand_tracking_bridge.launch.py \
  enable_left:=true enable_right:=false \
  motion_scale:=${MOTION_SCALE} \
  left_arm_rotation:='[1.0,0.0,0.0,0.0]' \
  right_arm_rotation:='[1.0,0.0,0.0,0.0]' &
BRIDGE_PID=$!

sleep 1

echo "[verify-left] starting left rc_ctrl_node (cartesian filter, RemoteCtrl-like limits) ..."
ros2 run jaka_driver rc_ctrl_node --ros-args \
  -p arm_name:=left \
  -p robot_ip:=${LEFT_IP} \
  -p target_topic:=/rc_ctrl/left_target \
  -p button_topic:=/rc_ctrl/button \
  -p grip_axis:=0 \
  -p enable_y_limit:=false \
  -p servo_filter_mode:=cartesian \
  -p servo_tran_deadband_mm:=0.5 \
  -p servo_max_tran_step_mm:=5.0 \
  -p max_cart_vel_mm_s:=1500.0 \
  -p max_cart_acc_mm_s2:=2500.0 \
  -p max_cart_jerk_mm_s3:=2800.0 \
  -p max_rot_vel_deg_s:=1120.0 \
  -p max_rot_acc_deg_s2:=1180.0 \
  -p max_rot_jerk_deg_s3:=1720.0 &
CTRL_PID=$!

echo "[verify-left] running. In Quest: hand mode=Left, UDP to this host:9000, then pinch to move."
echo "[verify-left] press Ctrl-C to stop everything."

wait "${BRIDGE_PID}" "${CTRL_PID}"
