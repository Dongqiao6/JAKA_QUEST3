"""Parser and geometry helpers for the Hand Tracking Streamer (HTS) packet format.

HTS emits UTF-8 CSV lines such as::

    Right wrist:, 0.2502, 1.0635, 0.2540, 0.194, -0.116, 0.094, -0.970
    Right landmarks:, 0.0, 0.0, 0.0, -0.0275, ...

``wrist`` carries 7 floats (x, y, z, qx, qy, qz, qw) in Unity's left-handed
world frame. ``landmarks`` carries 63 floats (21 joints x 3) relative to the
wrist, in the joint order documented by HTS. With the debug HUD option enabled
HTS prepends frame/timestamp metadata to the label; that is also accepted here.

This module deliberately has no ROS dependencies so it can be unit-tested on a
plain Python environment.
"""

from __future__ import annotations

import time
from dataclasses import dataclass, field
from typing import List, Optional, Tuple

import numpy as np

# Streamed landmark order from HTS (OpenXR joint index -> local streamed index).
THUMB_TIP = 4
INDEX_TIP = 8
MIDDLE_TIP = 12
LANDMARK_COUNT = 21


def parse_hts_line(line: str) -> Optional[Tuple[str, str, Tuple[float, ...]]]:
    """Parse one HTS CSV line into ``(side, kind, floats)``.

    ``side`` is ``"left"`` or ``"right"`` and ``kind`` is ``"wrist"`` or
    ``"landmarks"``. Returns ``None`` for unrelated lines (for example head
    pose or empty lines).
    """
    if not line:
        return None

    parts = [part.strip() for part in line.split(",")]
    if not parts:
        return None

    label = parts[0].lower()
    if "wrist" not in label and "landmarks" not in label:
        return None

    side = "right" if "right" in label else "left" if "left" in label else ""
    if not side:
        return None

    kind = "wrist" if "wrist" in label else "landmarks"
    floats: List[float] = []
    for part in parts[1:]:
        if not part:
            continue
        try:
            floats.append(float(part))
        except ValueError:
            continue

    if kind == "wrist":
        if len(floats) < 7:
            return None
        return side, kind, tuple(floats[:7])

    # Landmarks must be a flat xyz array. HTS sends 21 joints, but trim cleanly
    # to a multiple of three just in case a malformed packet arrives.
    floats = floats[: (len(floats) // 3) * 3]
    if len(floats) < 3:
        return None
    return side, kind, tuple(floats)


def parse_head_pose_line(line: str) -> Optional[Tuple[float, ...]]:
    """Parse one HTS head-pose CSV line into 7 floats ``(x, y, z, qx, qy, qz, qw)``.

    ``HandLandmarkStreamer`` and ``HeadPoseStreamer`` deliberately use separate
    labels, so the hand parser above ignores head lines. This helper exists for
    the bridge when a head-relative mapping mode is requested.
    """
    if not line:
        return None

    parts = [part.strip() for part in line.split(",")]
    if not parts:
        return None

    label = parts[0].lower()
    if "head" not in label or "pose" not in label:
        return None

    floats: List[float] = []
    for part in parts[1:]:
        if not part:
            continue
        try:
            floats.append(float(part))
        except ValueError:
            continue

    if len(floats) < 7:
        return None
    return tuple(floats[:7])


def _normalize_quat(quat: np.ndarray) -> np.ndarray:
    """Normalize a quaternion in ``(x, y, z, w)`` order."""
    quat = np.asarray(quat, dtype=float)
    norm = float(np.linalg.norm(quat))
    if norm <= 1e-12:
        return np.array([0.0, 0.0, 0.0, 1.0], dtype=float)
    return quat / norm


def quat_rotate(points: np.ndarray, quat: np.ndarray) -> np.ndarray:
    """Rotate an ``(N, 3)`` array by quaternion ``(x, y, z, w)``."""
    quat = _normalize_quat(quat)
    q_xyz = quat[:3]
    q_w = quat[3]
    t = 2.0 * np.cross(q_xyz, points)
    return points + q_w * t + np.cross(q_xyz, t)


def quat_to_matrix(quat: np.ndarray) -> np.ndarray:
    """Convert quaternion ``(x, y, z, w)`` to a 3x3 rotation matrix."""
    quat = _normalize_quat(quat)
    x, y, z, w = quat
    xx, yy, zz = x * x, y * y, z * z
    xy, xz, yz = x * y, x * z, y * z
    wx, wy, wz = w * x, w * y, w * z
    return np.array(
        [
            [1.0 - 2.0 * (yy + zz), 2.0 * (xy - wz), 2.0 * (xz + wy)],
            [2.0 * (xy + wz), 1.0 - 2.0 * (xx + zz), 2.0 * (yz - wx)],
            [2.0 * (xz - wy), 2.0 * (yz + wx), 1.0 - 2.0 * (xx + yy)],
        ],
        dtype=float,
    )


def matrix_to_quat(mat: np.ndarray) -> np.ndarray:
    """Convert a 3x3 rotation matrix to quaternion ``(x, y, z, w)``."""
    mat = np.asarray(mat, dtype=float)
    trace = mat[0, 0] + mat[1, 1] + mat[2, 2]
    if trace > 0.0:
        s = 0.5 / np.sqrt(trace + 1.0)
        w = 0.25 / s
        x = (mat[2, 1] - mat[1, 2]) * s
        y = (mat[0, 2] - mat[2, 0]) * s
        z = (mat[1, 0] - mat[0, 1]) * s
    elif mat[0, 0] > mat[1, 1] and mat[0, 0] > mat[2, 2]:
        s = 2.0 * np.sqrt(1.0 + mat[0, 0] - mat[1, 1] - mat[2, 2])
        w = (mat[2, 1] - mat[1, 2]) / s
        x = 0.25 * s
        y = (mat[0, 1] + mat[1, 0]) / s
        z = (mat[0, 2] + mat[2, 0]) / s
    elif mat[1, 1] > mat[2, 2]:
        s = 2.0 * np.sqrt(1.0 + mat[1, 1] - mat[0, 0] - mat[2, 2])
        w = (mat[0, 2] - mat[2, 0]) / s
        x = (mat[0, 1] + mat[1, 0]) / s
        y = 0.25 * s
        z = (mat[1, 2] + mat[2, 1]) / s
    else:
        s = 2.0 * np.sqrt(1.0 + mat[2, 2] - mat[0, 0] - mat[1, 1])
        w = (mat[1, 0] - mat[0, 1]) / s
        x = (mat[0, 2] + mat[2, 0]) / s
        y = (mat[1, 2] + mat[2, 1]) / s
        z = 0.25 * s
    return _normalize_quat(np.array([x, y, z, w], dtype=float))


@dataclass
class HandState:
    """Latest raw HTS data for one hand (still in Unity coordinates)."""

    side: str
    wrist_position: Optional[np.ndarray] = None
    wrist_quat: Optional[np.ndarray] = None
    landmarks_local: Optional[np.ndarray] = None
    last_update: float = field(default_factory=time.monotonic)

    def update_wrist(self, values: Tuple[float, ...]) -> None:
        if len(values) < 7:
            return
        arr = np.asarray(values, dtype=float)
        self.wrist_position = arr[:3].copy()
        self.wrist_quat = _normalize_quat(arr[3:7])
        self.last_update = time.monotonic()

    def update_landmarks(self, values: Tuple[float, ...]) -> None:
        arr = np.asarray(values, dtype=float)
        if arr.size < 3:
            return
        arr = arr[: (arr.size // 3) * 3]
        self.landmarks_local = arr.reshape((-1, 3))
        self.last_update = time.monotonic()

    @property
    def has_pose(self) -> bool:
        return self.wrist_position is not None and self.wrist_quat is not None

    def world_landmark(self, index: int) -> Optional[np.ndarray]:
        """Return a landmark expressed in Unity world coordinates."""
        if self.landmarks_local is None or index >= self.landmarks_local.shape[0]:
            return None
        point = self.landmarks_local[index]
        if self.has_pose:
            point = quat_rotate(point.reshape(1, 3), self.wrist_quat)[0] + self.wrist_position
        return point

    def pinch_distance(self) -> Optional[float]:
        """Return thumb-tip to index-tip distance in meters."""
        thumb = self.world_landmark(THUMB_TIP)
        index = self.world_landmark(INDEX_TIP)
        if thumb is None or index is None:
            return None
        return float(np.linalg.norm(thumb - index))

    def control_point(self, source: str) -> Optional[np.ndarray]:
        """Return the raw Unity-space point used as the robot TCP target."""
        if source == "wrist":
            return self.wrist_position.copy() if self.wrist_position is not None else None
        if source == "index_tip":
            return self.world_landmark(INDEX_TIP)
        if source == "pinch_midpoint":
            thumb = self.world_landmark(THUMB_TIP)
            index = self.world_landmark(INDEX_TIP)
            if thumb is None or index is None:
                return None
            return (thumb + index) * 0.5
        raise ValueError("unsupported position_source: %s" % source)


@dataclass
class HeadPoseState:
    """Latest HTS head pose, still in Unity tracking-space coordinates."""

    position: Optional[np.ndarray] = None
    quat: Optional[np.ndarray] = None
    last_update: float = field(default_factory=time.monotonic)

    def update(self, values: Tuple[float, ...]) -> None:
        if len(values) < 7:
            return
        arr = np.asarray(values, dtype=float)
        self.position = arr[:3].copy()
        self.quat = _normalize_quat(arr[3:7])
        self.last_update = time.monotonic()

    @property
    def has_pose(self) -> bool:
        return self.position is not None and self.quat is not None
