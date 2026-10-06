"""Bridge HTS hand landmarks to JAKA dual-arm Cartesian servo targets.

The Quest side runs the Hand Tracking Streamer (HTS) app and sends UDP packets
containing the wrist pose and 21 local landmarks for each tracked hand. This
node:

1. Listens for the HTS UDP/TCP stream.
2. Reconstructs a control point (wrist by default) and a pinch signal from the
   thumb-tip / index-tip distance.
3. Converts Unity's left-handed coordinates into the JAKA/ROS frame.
4. Publishes ``geometry_msgs/PoseStamped`` targets and a ``sensor_msgs/Joy``
   message on the topics consumed by ``jaka_driver``'s ``rc_ctrl_node``.

The existing ``rc_ctrl_node`` implements dead-man tracking: it captures the arm
pose at the rising edge of the grip axis and then servos the arm by the
controller displacement. Pinching therefore engages tracking, releasing holds
the arm in place. Only translation is commanded by default (``use_rpy_ctrl`` in
the JAKA launch file remains false).
"""

from __future__ import annotations

import json
import socket
import threading
import time
from typing import Dict, Optional, Tuple

import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from geometry_msgs.msg import PoseStamped, Point, Quaternion
from sensor_msgs.msg import Joy

from hand_tracking_robot.hts_parser import (
    HandState,
    matrix_to_quat,
    parse_hts_line,
    quat_to_matrix,
)

# Joy axis layout expected by jaka_driver's rc_ctrl_node:
# axes[0]=leftGrip, axes[1]=rightGrip, axes[2]=leftTrig, axes[3]=rightTrig,
# axes[4..5]=leftJS, axes[6..7]=rightJS.
JOY_AXIS_COUNT = 8
JOY_BUTTON_COUNT = 12


def _parse_float_list(value, expected: Optional[int] = None) -> np.ndarray:
    """Accept either a ROS list or a JSON-style string for list parameters."""
    if isinstance(value, str):
        try:
            value = json.loads(value)
        except json.JSONDecodeError:
            value = [float(part) for part in value.replace(",", " ").split()]
    arr = np.asarray(value, dtype=float)
    if expected is not None and arr.size != expected:
        raise ValueError("expected %d floats, got %d" % (expected, arr.size))
    return arr


class HandTrackingBridge(Node):
    """ROS2 bridge from HTS to JAKA rc_ctrl topics."""

    def __init__(self) -> None:
        super().__init__("hand_tracking_bridge")

        # --- Network input -------------------------------------------------
        self.protocol = str(self.declare_parameter("protocol", "udp").value).lower()
        self.host = str(self.declare_parameter("host", "0.0.0.0").value)
        self.port = int(self.declare_parameter("port", 9000).value)

        # --- Output topics -------------------------------------------------
        left_topic = str(
            self.declare_parameter("left_target_topic", "/rc_ctrl/left_target").value
        )
        right_topic = str(
            self.declare_parameter("right_target_topic", "/rc_ctrl/right_target").value
        )
        button_topic = str(
            self.declare_parameter("button_topic", "/rc_ctrl/button").value
        )
        self.enable_left = bool(self.declare_parameter("enable_left", True).value)
        self.enable_right = bool(self.declare_parameter("enable_right", True).value)

        # --- Processing parameters ----------------------------------------
        self.publish_rate = float(self.declare_parameter("publish_rate", 100.0).value)
        self.smooth_alpha = float(self.declare_parameter("smooth_alpha", 0.45).value)
        self.motion_scale = float(self.declare_parameter("motion_scale", 1.0).value)
        self.pinch_threshold = float(self.declare_parameter("pinch_threshold", 0.035).value)
        self.pinch_release_ratio = float(
            self.declare_parameter("pinch_release_ratio", 1.5).value
        )
        self.tracking_timeout = float(self.declare_parameter("tracking_timeout", 1.0).value)
        self.position_source = str(self.declare_parameter("position_source", "wrist").value).lower()
        if self.position_source not in ("wrist", "index_tip", "pinch_midpoint"):
            self.get_logger().warn(
                "unsupported position_source '%s', falling back to 'wrist'",
                self.position_source,
            )
            self.position_source = "wrist"

        # --- Coordinate transforms ----------------------------------------
        self.glasses_frame = str(self.declare_parameter("glasses_frame", "quest").value)
        self.robot_base_frame = str(self.declare_parameter("robot_base_frame", "world").value)

        # Unity LH -> RH transform. The default matches the JAKA VR teleop
        # mapping used in quest_reader.matrix_to_pose: (x,y,z) -> (-z,-x,y).
        unity_to_robot = _parse_float_list(
            self.declare_parameter(
                "unity_to_robot",
                [0.0, 0.0, -1.0, -1.0, 0.0, 0.0, 0.0, 1.0, 0.0],
            ).value,
            expected=9,
        )
        self.unity_to_robot = unity_to_robot.reshape(3, 3)

        tf_translation = _parse_float_list(
            self.declare_parameter("tf_translation", [0.0, 0.0, 0.0]).value,
            expected=3,
        )
        tf_rotation = _parse_float_list(
            self.declare_parameter("tf_rotation", [1.0, 0.0, 0.0, 0.0]).value,
            expected=4,
        )
        self.tf_translation = tf_translation
        # tf_rotation is [w, x, y, z]; quat_to_matrix wants [x, y, z, w].
        self.rot_base = quat_to_matrix(tf_rotation[[1, 2, 3, 0]])

        # --- State and threads --------------------------------------------
        self.hands: Dict[str, HandState] = {
            "left": HandState("left"),
            "right": HandState("right"),
        }
        self._filter_pos = {"left": None, "right": None}
        self._filter_quat = {"left": None, "right": None}
        self._pinched = {"left": False, "right": False}
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._conn_threads = []

        # Publishers: depth=1 keeps only the newest target, matching rc_ctrl's
        # QoS(1) subscription.
        qos = QoSProfile(depth=1)
        self.left_pub = self.create_publisher(PoseStamped, left_topic, qos)
        self.right_pub = self.create_publisher(PoseStamped, right_topic, qos)
        self.button_pub = self.create_publisher(Joy, button_topic, qos)

        self._publish_count = 0
        self.timer = self.create_timer(1.0 / max(self.publish_rate, 1.0), self._publish_callback)

        self._start_network()
        self.get_logger().info(
            "hand_tracking_bridge started: protocol=%s %s:%d, source=%s, "
            "left=%s, right=%s, buttons=%s, rate=%.1f Hz",
            self.protocol,
            self.host,
            self.port,
            self.position_source,
            left_topic,
            right_topic,
            button_topic,
            self.publish_rate,
        )

    # ------------------------------------------------------------------
    # Networking
    # ------------------------------------------------------------------
    def _start_network(self) -> None:
        if self.protocol == "udp":
            thread = threading.Thread(target=self._udp_loop, daemon=True)
        elif self.protocol == "tcp":
            thread = threading.Thread(target=self._tcp_server_loop, daemon=True)
        else:
            raise ValueError("protocol must be 'udp' or 'tcp'")
        thread.start()

    def _ingest_bytes(self, data: bytes) -> None:
        try:
            message = data.decode("utf-8")
        except UnicodeDecodeError:
            return
        for line in message.splitlines():
            self._handle_line(line.strip())

    def _handle_line(self, line: str) -> None:
        parsed = parse_hts_line(line)
        if parsed is None:
            return
        side, kind, floats = parsed
        if side not in self.hands:
            return
        with self._lock:
            if kind == "wrist":
                self.hands[side].update_wrist(floats)
            else:
                self.hands[side].update_landmarks(floats)

    def _udp_loop(self) -> None:
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        try:
            sock.bind((self.host, self.port))
        except OSError as exc:
            self.get_logger().error("cannot bind UDP %s:%d: %s", self.host, self.port, exc)
            return
        sock.settimeout(0.5)
        self.get_logger().info("listening for HTS UDP on %s:%d", self.host, self.port)
        while not self._stop.is_set():
            try:
                data, _addr = sock.recvfrom(65536)
            except socket.timeout:
                continue
            except OSError:
                break
            self._ingest_bytes(data)
        sock.close()

    def _tcp_server_loop(self) -> None:
        server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        try:
            server.bind((self.host, self.port))
        except OSError as exc:
            self.get_logger().error("cannot bind TCP %s:%d: %s", self.host, self.port, exc)
            return
        server.listen(5)
        server.settimeout(0.5)
        self.get_logger().info("listening for HTS TCP on %s:%d", self.host, self.port)
        while not self._stop.is_set():
            try:
                conn, addr = server.accept()
            except socket.timeout:
                continue
            except OSError:
                break
            thread = threading.Thread(target=self._tcp_conn_loop, args=(conn, addr), daemon=True)
            thread.start()
            self._conn_threads.append(thread)
        server.close()

    def _tcp_conn_loop(self, conn: socket.socket, addr) -> None:
        self.get_logger().info("HTS TCP connection from %s", addr)
        conn.settimeout(0.5)
        buffer = ""
        with conn:
            while not self._stop.is_set():
                try:
                    data = conn.recv(4096)
                except socket.timeout:
                    continue
                except OSError:
                    break
                if not data:
                    break
                buffer += data.decode("utf-8", errors="ignore")
                while "\n" in buffer:
                    line, buffer = buffer.split("\n", 1)
                    self._handle_line(line.strip())

    # ------------------------------------------------------------------
    # Signal processing
    # ------------------------------------------------------------------
    @staticmethod
    def _ema_vec(prev: Optional[np.ndarray], new: np.ndarray, alpha: float) -> np.ndarray:
        if prev is None or alpha <= 0.0:
            return new.copy()
        return prev + alpha * (new - prev)

    @staticmethod
    def _ema_quat(prev: Optional[np.ndarray], new: np.ndarray, alpha: float) -> np.ndarray:
        new = np.asarray(new, dtype=float)
        if prev is None or alpha <= 0.0:
            return new.copy()
        if float(np.dot(prev, new)) < 0.0:
            new = -new
        q = prev + alpha * (new - prev)
        norm = float(np.linalg.norm(q))
        if norm > 1e-12:
            q = q / norm
        return q

    def _pinch_value(self, side: str, hand: HandState, fresh: bool) -> float:
        if not fresh:
            self._pinched[side] = False
            return 0.0
        distance = hand.pinch_distance()
        if distance is None:
            self._pinched[side] = False
            return 0.0

        if self._pinched[side]:
            if distance > self.pinch_threshold * self.pinch_release_ratio:
                self._pinched[side] = False
        else:
            if distance < self.pinch_threshold:
                self._pinched[side] = True
        return 1.0 if self._pinched[side] else 0.0

    def _to_robot_pose(
        self, point_unity: np.ndarray, quat_unity: np.ndarray
    ) -> Tuple[np.ndarray, np.ndarray]:
        """Map a raw Unity point/quaternion to robot-base frame.

        Returns ``(position, quaternion)`` with quaternion in ``(x, y, z, w)``
        order. The static ``tf_translation``/``tf_rotation`` parameters express
        the glasses frame origin in robot-base coordinates.
        """
        glasses_pos = self.unity_to_robot @ point_unity
        glasses_pos = glasses_pos * self.motion_scale
        robot_pos = self.rot_base @ glasses_pos + self.tf_translation

        r_glasses = self.unity_to_robot @ quat_to_matrix(quat_unity) @ self.unity_to_robot.T
        q_glasses = matrix_to_quat(r_glasses)
        r_robot = self.rot_base @ quat_to_matrix(q_glasses)
        q_robot = matrix_to_quat(r_robot)
        return robot_pos, q_robot

    @staticmethod
    def _pose_msg(stamp, frame_id: str, position: np.ndarray, quat: np.ndarray) -> PoseStamped:
        msg = PoseStamped()
        msg.header.stamp = stamp
        msg.header.frame_id = frame_id
        msg.pose.position = Point(
            x=float(position[0]), y=float(position[1]), z=float(position[2])
        )
        msg.pose.orientation = Quaternion(
            x=float(quat[0]), y=float(quat[1]), z=float(quat[2]), w=float(quat[3])
        )
        return msg

    # ------------------------------------------------------------------
    # Publishing
    # ------------------------------------------------------------------
    def _publish_callback(self) -> None:
        now = time.monotonic()
        stamp = self.get_clock().now().to_msg()

        with self._lock:
            left = self.hands["left"]
            right = self.hands["right"]
            left_fresh = left.has_pose and (now - left.last_update) <= self.tracking_timeout
            right_fresh = right.has_pose and (now - right.last_update) <= self.tracking_timeout
            left_point = left.control_point(self.position_source) if left_fresh else None
            right_point = right.control_point(self.position_source) if right_fresh else None
            left_quat = left.wrist_quat.copy() if left_fresh else None
            right_quat = right.wrist_quat.copy() if right_fresh else None
            left_pinch = self._pinch_value("left", left, left_fresh and left_point is not None)
            right_pinch = self._pinch_value("right", right, right_fresh and right_point is not None)

        if self.enable_left and left_point is not None and left_quat is not None:
            pos = self._ema_vec(self._filter_pos["left"], left_point, self.smooth_alpha)
            quat = self._ema_quat(self._filter_quat["left"], left_quat, self.smooth_alpha)
            self._filter_pos["left"] = pos
            self._filter_quat["left"] = quat
            robot_pos, robot_quat = self._to_robot_pose(pos, quat)
            self.left_pub.publish(self._pose_msg(stamp, self.robot_base_frame, robot_pos, robot_quat))

        if self.enable_right and right_point is not None and right_quat is not None:
            pos = self._ema_vec(self._filter_pos["right"], right_point, self.smooth_alpha)
            quat = self._ema_quat(self._filter_quat["right"], right_quat, self.smooth_alpha)
            self._filter_pos["right"] = pos
            self._filter_quat["right"] = quat
            robot_pos, robot_quat = self._to_robot_pose(pos, quat)
            self.right_pub.publish(self._pose_msg(stamp, self.robot_base_frame, robot_pos, robot_quat))

        joy = Joy()
        joy.header.stamp = stamp
        joy.header.frame_id = self.glasses_frame
        joy.axes = [
            left_pinch,
            right_pinch,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
        ][:JOY_AXIS_COUNT]
        joy.buttons = [0] * JOY_BUTTON_COUNT
        self.button_pub.publish(joy)

        self._publish_count += 1
        if self._publish_count % 50 == 0:
            self.get_logger().info(
                "bridge active: left_pinch=%.0f right_pinch=%.0f "
                "(left_fresh=%s right_fresh=%s)",
                left_pinch,
                right_pinch,
                left_fresh,
                right_fresh,
            )

    def shutdown(self) -> None:
        self._stop.set()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = HandTrackingBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
