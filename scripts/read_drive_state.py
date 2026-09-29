"""Read drive telemetry; never creates command publishers or calls services."""
import json
import time

from lingxi_x2.backends.ros_environment import reexec_with_ros_environment
from lingxi_x2.client import X2Client


def main():
    reexec_with_ros_environment()
    from aimdk_msgs.msg import DcuMonitorData, JointNoRealTimeStateArray
    from rosidl_runtime_py.convert import message_to_ordereddict
    samples = {}
    counts = {}
    with X2Client("config/x2.yaml") as client:
        def receive(key, message):
            samples[key] = {"received_monotonic_ns": time.monotonic_ns(),
                            "message": message_to_ordereddict(message)}
            counts[key] = counts.get(key, 0)+1
        backend = client._backend
        subscriptions = []
        try:
            for key, topic, msgtype in (
                ("udcu", "/aima/hal/monitor/udcu", DcuMonitorData),
                ("joints", "/aima/hal/joint/norealtime/udcu_joint/state", JointNoRealTimeStateArray),
            ):
                subscriptions.append(backend._node.create_subscription(
                    msgtype, topic, lambda msg, selected=key: receive(selected, msg), backend._qos))
            time.sleep(4.)
        finally:
            for subscription in subscriptions:
                backend._node.destroy_subscription(subscription)
    print(json.dumps({"kind": "drive_readonly", "counts": counts, "samples": samples}, indent=2), flush=True)


if __name__ == "__main__":
    main()
