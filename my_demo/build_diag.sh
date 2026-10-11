#!/usr/bin/env bash
set -euo pipefail

# 编译临时诊断程序，生成 my_demo/print_left_install_angle
SDK="ros2_ws/src/jaka_driver/sdk_lib/libjakaAPI.so"
INCLUDE="ros2_ws/src/jaka_driver/include"
SRC="my_demo/print_left_install_angle.cpp"
OUT="my_demo/print_left_install_angle"

arch="$(uname -m)"
if [ "${arch}" != "aarch64" ]; then
  echo "error: libjakaAPI.so is an aarch64 library, but this host is ${arch}." >&2
  echo "run this script on the robot/Jetson (aarch64) host, or use an aarch64 cross toolchain." >&2
  exit 1
fi

g++ -std=c++17 -I"${INCLUDE}" "${SRC}" -L"$(dirname "${SDK}")" \
  -ljakaAPI -Wl,-rpath,"$(cd "$(dirname "${SDK}")" && pwd)" \
  -o "${OUT}"

echo "built ${OUT}"
echo "run: ./${OUT} 192.168.71.37"
