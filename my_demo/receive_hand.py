from hand_tracking_sdk import (
    HTSClient,
    HTSClientConfig,
    StreamOutput,
    HeadFrame,
    JointName,
)

# 配置 UDP 客户端（沿用你当前可用的配置）
config = HTSClientConfig(
    output=StreamOutput.FRAMES,
    transport_mode="udp",   # 请确认这是你当前可用的参数名
    host="0.0.0.0",
    port=9000,
)

client = HTSClient(config)
print("Waiting for hand tracking data... Press Ctrl+C to stop.")

try:
    for frame in client.iter_events():
        # 跳过头部帧
        if isinstance(frame, HeadFrame):
            continue

        side = frame.side.value.upper()
        print(f"\n===== {side} HAND =====")

        # 遍历所有关节并打印
        for joint in JointName:
            # 跳过不属于当前帧的关节（例如，某些枚举可能包含非21关节的项）
            try:
                x, y, z = frame.get_joint(joint)
                print(f"{joint.value:20s}: ({x:7.4f}, {y:7.4f}, {z:7.4f})")
            except (KeyError, AttributeError):
                # 如果该关节在此帧中不存在，则跳过
                continue

except KeyboardInterrupt:
    print("\nStopped.")