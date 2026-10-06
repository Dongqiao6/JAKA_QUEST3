"""Send synthetic HTS UDP packets for testing the bridge without a Quest.

Example::

    python fake_hand_stream.py --host 127.0.0.1 --port 9000 --pinch

The bridge node should log ``left_pinch``/``right_pinch`` and publish
PoseStamped targets.
"""

import argparse
import math
import socket
import time

import numpy as np

THUMB_TIP = 4
INDEX_TIP = 8


def make_landmarks(pinch: bool) -> np.ndarray:
    landmarks = np.zeros((21, 3), dtype=float)
    landmarks[THUMB_TIP] = [0.025, -0.02, 0.01]
    if pinch:
        landmarks[INDEX_TIP] = [0.015, -0.018, 0.012]
    else:
        landmarks[INDEX_TIP] = [0.015, -0.018, 0.07]
    return landmarks


def fmt(values) -> str:
    return ", ".join("%.4f" % v for v in values)


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=9000)
    parser.add_argument("--pinch", action="store_true")
    parser.add_argument("--both-hands", action="store_true")
    parser.add_argument("--rate", type=float, default=90.0)
    args = parser.parse_args()

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    start = time.monotonic()
    frame = 0
    print("sending HTS-like UDP to %s:%d (Ctrl+C to stop)" % (args.host, args.port))

    try:
        while True:
            t = time.monotonic() - start
            x = 0.08 * math.sin(t * 0.5)
            y = 1.0 + 0.05 * math.sin(t * 0.3)
            z = 0.2 + 0.05 * math.cos(t * 0.5)

            hands = ["Right"] if not args.both_hands else ["Left", "Right"]
            packet = []
            for side in hands:
                packet.append("%s wrist:, %s, 0.0, 0.0, 0.0, 1.0" % (side, fmt([x, y, z])))
                landmarks = make_landmarks(args.pinch)
                packet.append("%s landmarks:, %s" % (side, fmt(landmarks.flatten())))

            sock.sendto("\n".join(packet).encode("utf-8"), (args.host, args.port))
            frame += 1
            time.sleep(max(0.0, 1.0 / args.rate - (time.monotonic() - start - frame / args.rate)))
    except KeyboardInterrupt:
        print("\nstopped")


if __name__ == "__main__":
    main()
