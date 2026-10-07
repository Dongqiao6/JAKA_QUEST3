#!/usr/bin/env bash
set -euo pipefail

PKG="com.wengmister.handtrackingstreamer"
ADB="${ADB:-adb}"

cleanup() {
  echo "[quest-apk] force-stopping ${PKG} ..."
  "${ADB}" shell am force-stop "${PKG}" >/dev/null 2>&1 || true
}

trap cleanup EXIT
trap 'exit 130' INT TERM

if [ "$#" -eq 0 ]; then
  echo "usage: $0 <command...>" >&2
  echo "example: $0 ros2 launch hand_tracking_robot dual_arm_hand_teleop.launch.py" >&2
  exit 2
fi

echo "[quest-apk] launching ${PKG} ..."
"${ADB}" shell monkey -p "${PKG}" 1 >/dev/null 2>&1 || {
  echo "[quest-apk] could not launch ${PKG}; check adb connection" >&2
  exit 1
}

"$@"
