#!/usr/bin/env bash
set -euo pipefail

# 编译临时诊断程序，生成 my_demo/left_test/print_left_install_angle
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/../.." && pwd)"

SDK="${REPO_ROOT}/ros2_ws/src/jaka_driver/sdk_lib/libjakaAPI.so"
INCLUDE="${REPO_ROOT}/ros2_ws/src/jaka_driver/include"
SRC="${SCRIPT_DIR}/print_left_install_angle.cpp"
OUT="${SCRIPT_DIR}/print_left_install_angle"

arch="$(uname -m)"
if [ "${arch}" != "aarch64" ]; then
  echo "error: libjakaAPI.so is an aarch64 library, but this host is ${arch}." >&2
  echo "run this script on the robot/Jetson (aarch64) host, or use an aarch64 cross toolchain." >&2
  exit 1
fi

g++ -std=c++17 -I"${INCLUDE}" "${SRC}" \
  -L"$(dirname "${SDK}")" -ljakaAPI \
  -Wl,-rpath,"$(dirname "${SDK}")" \
  -o "${OUT}"

echo "built ${OUT}"
echo "run: ${OUT} 192.168.71.37"
