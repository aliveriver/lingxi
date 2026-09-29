"""Bounded latest-sample storage for read-only gravity diagnostics."""
from copy import deepcopy
import math
import threading
import time


TOPICS = {
    "chest_imu": ("/aima/hal/imu/chest/state", "sensor_msgs/msg/Imu"),
    "pelvis_imu": ("/aima/hal/imu/torso/state", "sensor_msgs/msg/Imu"),
    "waist_state": ("/aima/hal/joint/waist/state", "aimdk_msgs/msg/JointStateArray"),
    "arm_state": ("/aima/hal/joint/arm/state", "aimdk_msgs/msg/JointStateArray"),
}


def gravity_sample(kind, message, received_monotonic_ns: int, received_ros_ns: int) -> dict:
    if kind not in TOPICS:
        raise ValueError(f"Unknown sample kind: {kind}")
    header = message.header
    stamp = header.stamp
    if stamp.sec < 0 or not 0 <= stamp.nanosec < 1_000_000_000:
        raise ValueError("Invalid source timestamp")
    sample = {"received_monotonic_ns": received_monotonic_ns, "received_ros_ns": received_ros_ns,
              "stamp_ns": int(stamp.sec) * 10**9 + int(stamp.nanosec), "frame_id": str(header.frame_id)}
    if kind.endswith("imu"):
        q, a = message.orientation, message.linear_acceleration
        sample.update(orientation_xyzw=[float(q.x), float(q.y), float(q.z), float(q.w)],
                      orientation_covariance=list(map(float, message.orientation_covariance)),
                      linear_acceleration_m_s2=[float(a.x), float(a.y), float(a.z)])
        values = sample["orientation_xyzw"] + sample["orientation_covariance"] + sample["linear_acceleration_m_s2"]
    else:
        sample["joints"] = [{"name": str(j.name), "position": float(j.position),
                             "velocity": float(j.velocity), "error_code": int(j.error_code)}
                            for j in message.joints]
        values = [v for j in sample["joints"] for v in (j["position"], j["velocity"])]
    if not all(math.isfinite(v) for v in values):
        raise ValueError("Nonfinite telemetry")
    return sample


class GravityCapture:
    """Store one latest sample per topic plus bounded errors and counters.

    Invalid latest frames invalidate the retained frame for that topic. This
    recorder is independent of ROS and never creates transport endpoints.
    """

    def __init__(self):
        self.samples = {}
        self.counts = dict.fromkeys(TOPICS, 0)
        self.errors = []
        self.error_count = 0
        self.lock = threading.Lock()

    def receive(self, kind, message, *, received_monotonic_ns: int, received_ros_ns: int):
        with self.lock:
            try:
                if kind not in TOPICS:
                    raise ValueError(f"Unknown sample kind: {kind}")
                self.counts[kind] += 1
                sample = gravity_sample(kind, message, received_monotonic_ns, received_ros_ns)
                previous = self.samples.get(kind)
                if previous and (sample["stamp_ns"] <= previous["stamp_ns"]
                                 or received_monotonic_ns <= previous["received_monotonic_ns"]
                                 or received_ros_ns < previous["received_ros_ns"]):
                    raise ValueError("Duplicate/backward source or receive timestamp")
                self.samples[kind] = sample
            except Exception as exc:
                self.samples.pop(kind, None)
                self.error_count += 1
                if len(self.errors) < 20:
                    self.errors.append(f"{kind}: {exc}")

    def snapshot(self, *, captured_monotonic_ns: int, captured_ros_ns: int) -> dict:
        with self.lock:
            return {"kind": "gravity_readonly_snapshot", "schema_version": 1,
                    "captured_monotonic_ns": captured_monotonic_ns, "captured_ros_ns": captured_ros_ns,
                    "topics": {kind: {"topic": topic, "type": msgtype} for kind, (topic, msgtype) in TOPICS.items()},
                    "samples": deepcopy(self.samples), "counts": dict(self.counts),
                    "error_count": self.error_count, "errors": list(self.errors)}


def capture_readonly(duration_s: float) -> dict:
    """Standalone ROS subscriptions; no client, command publisher or service call."""
    if not math.isfinite(duration_s) or not .5 <= duration_s <= 30:
        raise ValueError("duration_s must be between 0.5 and 30 seconds")
    import rclpy
    from rclpy.context import Context
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import Imu
    from aimdk_msgs.msg import JointStateArray

    context = Context()
    node = None
    executor = None
    capture = GravityCapture()
    try:
        rclpy.init(context=context)
        node = Node("lingxi_gravity_readonly", context=context, enable_rosout=False,
                    start_parameter_services=False)
        executor = SingleThreadedExecutor(context=context)
        executor.add_node(node)
        subscriptions = []
        for kind, (topic, _) in TOPICS.items():
            def receive(message, selected=kind):
                capture.receive(selected, message, received_monotonic_ns=time.monotonic_ns(),
                                received_ros_ns=node.get_clock().now().nanoseconds)
            subscriptions.append(node.create_subscription(
                Imu if kind.endswith("imu") else JointStateArray, topic, receive, qos_profile_sensor_data))
        stop = time.monotonic() + duration_s
        while time.monotonic() < stop:
            executor.spin_once(timeout_sec=min(.05, max(0., stop - time.monotonic())))
        snapshot = capture.snapshot(captured_monotonic_ns=time.monotonic_ns(),
                                    captured_ros_ns=node.get_clock().now().nanoseconds)
        snapshot["topic_publishers"] = {kind: node.count_publishers(topic) for kind, (topic, _) in TOPICS.items()}
        return snapshot
    finally:
        if executor is not None:
            executor.shutdown()
        if node is not None:
            node.destroy_node()
        if context.ok():
            context.shutdown()
