#!/usr/bin/env python3
"""
Minimal cloud-only ROS2 sender for a single SHT40 sensor.

- Fixed node and topic names (set below).
- Subscribes to temperature & humidity (std_msgs/Float32).
- Immediately forwards each reading to InfluxDB (no pairing, no local storage).
"""

import time
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from std_msgs.msg import Float32

# --- Fixed config (matches your snippet) ---
NODE_NAME       = "sht40_sensor_1"
TEMP_TOPIC_NAME = "sht40_sensor_1/temperature"
HUM_TOPIC_NAME  = "sht40_sensor_1/humidity"

# Influx integration from your project
from my_multi_plot.utils.influxdb import InfluxCollector
from my_multi_plot.components.secrets import influx_secret

MEASUREMENT = "sht40_readings"         # change if you want a different measurement
TAGS = {"device": NODE_NAME}           # per-point tags (kept extremely simple)


class SHT40CloudNode(Node):
    def __init__(self):
        # name the ROS node exactly as requested
        super().__init__(NODE_NAME)

        # Start the Influx sender thread once
        self.influx = InfluxCollector(TAGS, influx_secret)
        self.influx.start_send_thread()
        self.get_logger().info("Cloud upload: ENABLED")

        # Simple reliable QoS
        qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
        )

        # Subscribe to fixed topics
        self.create_subscription(Float32, TEMP_TOPIC_NAME, self._on_temp, qos)
        self.create_subscription(Float32, HUM_TOPIC_NAME,  self._on_hum,  qos)
        self.get_logger().info(f"Subscribed to: {TEMP_TOPIC_NAME}, {HUM_TOPIC_NAME}")

    # ---- Callbacks: forward immediately to Influx ----
    def _on_temp(self, msg: Float32):
        try:
            self.influx.record_fields_tags(
                measurement=MEASUREMENT,
                fields={"temperature": float(msg.data)},
                tags={},  # per-point extras (kept empty; base TAGS already applied)
            )
        except Exception as e:
            self.get_logger().warn(f"Influx send (temperature) failed: {e}")

    def _on_hum(self, msg: Float32):
        try:
            self.influx.record_fields_tags(
                measurement=MEASUREMENT,
                fields={"humidity": float(msg.data)},
                tags={},
            )
        except Exception as e:
            self.get_logger().warn(f"Influx send (humidity) failed: {e}")

    # ---- Clean shutdown ----
    def destroy_node(self):
        # Give a last chance to flush any queued points
        try:
            self.influx.stop_send_thread()
        except Exception:
            pass
        return super().destroy_node()


def main():
    rclpy.init()
    node = SHT40CloudNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.destroy_node()
        finally:
            if rclpy.ok():
                rclpy.shutdown()


if __name__ == "__main__":
    main()
