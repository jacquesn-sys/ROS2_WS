#!/usr/bin/env python3
"""
This script discovers topics named like "<device>/temperature" and "<device>/humidity",
subscribes to them automatically, keeps the most recent N seconds of data, and
(1) saves all incoming data to CSV and (2) saves a single static plot **on exit**.

No live plotting, no periodic PNG saves.
"""

# ---- Standard library imports ----
import argparse                      # Parse command-line flags like --window-seconds, --save-csv
import csv                           # CSV logging
import re                            # Topic name matching
import time                          # Timestamps (seconds since epoch)
import os                            # For DISPLAY check and filesystem ops
from collections import defaultdict, deque   # Fast FIFO buffers
from dataclasses import dataclass             # Tiny typed container for series
from typing import Dict, Deque, Optional      # Type hints only

# ---- ROS 2 client library imports ----
import rclpy                          # Core ROS 2 Python client
from rclpy.node import Node           # Base class for making a node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy  # Sensor QoS
from std_msgs.msg import Float32      # Expected message type for these topics

# ---- Matplotlib (only to generate a static plot at exit) ----
# Use a non-GUI backend on headless systems so saving PNG works without a display.
import matplotlib
if "DISPLAY" not in os.environ:
    matplotlib.use("Agg")
import matplotlib.pyplot as plt

# Allow an optional leading "/" (ROS topics commonly start with it).
# Group(1) = device name, group(2) true for "temperature", group(3) true for "humidity".
DEVICE_TOPIC_RE = re.compile(r"^/?([^/]+)/(?:(temperature)|(humidity))$")

@dataclass
class Series:
    """Small container for one time series with parallel time/value buffers."""
    times: Deque[float]
    values: Deque[float]


class MultiBoardPlotNode(Node):
    """
    A ROS 2 node that:
      - scans the graph for <device>/temperature and <device>/humidity topics,
      - subscribes dynamically (no restart needed),
      - buffers a sliding window of recent data,
      - appends every reading to CSV,
      - and writes one static PNG at shutdown (if --save-png is given).
    """

    def __init__(self, window_seconds: int,
                 save_csv: Optional[str],
                 save_png: Optional[str],
                 flush_every: int = 20):
        # Initialize the base node with a readable name.
        super().__init__('multi_board_plot_listener')

        # How much recent data to keep in memory (for the final plot axes).
        self.window_seconds = window_seconds

        # Where to save CSV/PNG (None means "don’t save").
        self.save_csv_path = save_csv
        self.save_png_path = save_png

        # Flush CSV after this many rows to avoid data loss on power cut.
        self.flush_every = max(1, flush_every)
        self._rows_written_since_flush = 0

        # QoS suitable for sensors: low-latency, OK to drop occasionally.
        self.qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10
        )

        # In-memory series buffers per device.
        self.series_temp: Dict[str, Series] = defaultdict(lambda: Series(deque(), deque()))
        self.series_hum:  Dict[str, Series] = defaultdict(lambda: Series(deque(), deque()))

        # Record topics we’ve already subscribed to.
        self.subscribed_topics = set()

        # Prepare CSV if requested.
        self.csv_file = None
        self.csv_writer = None
        if self.save_csv_path:
            # Ensure parent directory exists.
            os.makedirs(os.path.dirname(self.save_csv_path) or ".", exist_ok=True)
            # Open in append mode so you can stop/start without overwriting.
            self.csv_file = open(self.save_csv_path, "a", newline="")
            self.csv_writer = csv.writer(self.csv_file)
            # Write a header if the file was empty.
            if os.stat(self.save_csv_path).st_size == 0:
                self.csv_writer.writerow(["ts", "device", "type", "value"])

        # Timers:
        #  - Scan for new topics every 1s (dynamic discovery).
        #  - Prune old samples so memory stays bounded.
        self.scan_timer  = self.create_timer(1.0, self._scan_for_topics)
        self.prune_timer = self.create_timer(1.0, self._prune_old_samples)

        # Friendly startup log.
        self.get_logger().info(
            f"Ready. Watching topics '<device>/temperature' and '<device>/humidity'. "
            f"History window={self.window_seconds}s. "
            f"CSV={'ON' if self.save_csv_path else 'OFF'}, "
            f"PNG at exit={'ON' if self.save_png_path else 'OFF'}."
        )

    # ---------- Discovery & subscription ----------

    def _scan_for_topics(self):
        """Inspect all topics; subscribe to any new that match our pattern and type."""
        for topic_name, types in self.get_topic_names_and_types():
            # Only match names like [/]<device>/(temperature|humidity)
            m = DEVICE_TOPIC_RE.match(topic_name)
            if not m:
                continue
            # Avoid duplicate subscriptions.
            if topic_name in self.subscribed_topics:
                continue
            # Only accept std_msgs/Float32 for these.
            if "std_msgs/msg/Float32" not in types:
                self.get_logger().warn(f"Skipping {topic_name}: not Float32 (types={types})")
                continue

            # Extract device and kind from the regex groups.
            device = m.group(1)
            kind = "temperature" if m.group(2) else "humidity"

            # Create the subscriber; capture device/kind via default args in the lambda.
            self.create_subscription(
                Float32,
                topic_name,
                lambda msg, d=device, k=kind: self._on_value(d, k, msg),
                self.qos
            )
            self.subscribed_topics.add(topic_name)
            self.get_logger().info(f"Subscribed to {topic_name} (device={device}, kind={kind})")

    # ---------- Data handling ----------

    def _on_value(self, device: str, kind: str, msg: Float32):
        """On each message: buffer it, and append a CSV row if enabled."""
        now = time.time()  # float seconds since epoch
        # Choose the correct series map.
        series = self.series_temp[device] if kind == "temperature" else self.series_hum[device]
        # Append to in-memory buffers.
        series.times.append(now)
        series.values.append(float(msg.data))
        # Append to CSV (if enabled).
        if self.csv_writer:
            self.csv_writer.writerow([now, device, kind, float(msg.data)])
            self._rows_written_since_flush += 1
            # Flush every N rows to keep data safe.
            if self._rows_written_since_flush >= self.flush_every:
                self.csv_file.flush()
                self._rows_written_since_flush = 0

    def _prune_old_samples(self):
        """Keep only the last `window_seconds` of data in memory."""
        cutoff = time.time() - self.window_seconds
        # Prune both temperature and humidity series per device.
        for series in list(self.series_temp.values()) + list(self.series_hum.values()):
            while series.times and series.times[0] < cutoff:
                series.times.popleft()
                series.values.popleft()

    # ---------- Shutdown: save one static plot ----------

    def _save_plot_once(self):
        """Generate and save a single PNG with the latest buffered data (if requested)."""
        if not self.save_png_path:
            return  # Nothing to do if the user didn’t ask for PNG.

        # Create a 2-row figure: top=Temperature, bottom=Humidity.
        fig = plt.figure(constrained_layout=True, figsize=(9, 6))
        gs = fig.add_gridspec(2, 1)
        ax_temp = fig.add_subplot(gs[0, 0])
        ax_hum  = fig.add_subplot(gs[1, 0])

        # Draw temperature lines, one per device.
        now = time.time()
        any_data = False
        for device, series in self.series_temp.items():
            if series.times:
                x = [t - now for t in series.times]  # seconds relative to now
                y = list(series.values)
                ax_temp.plot(x, y, label=device)
                any_data = True
        ax_temp.set_title("Temperature (°C) — last window")
        ax_temp.set_xlabel("Time (s) (past → now)")
        ax_temp.set_ylabel("°C")
        if any_data:
            ax_temp.legend(loc="upper left", fontsize="small")
        ax_temp.set_xlim(-self.window_seconds, 0)
        ax_temp.relim(); ax_temp.autoscale_view()

        # Draw humidity lines, one per device.
        any_data = False
        for device, series in self.series_hum.items():
            if series.times:
                x = [t - now for t in series.times]
                y = list(series.values)
                ax_hum.plot(x, y, label=device)
                any_data = True
        ax_hum.set_title("Humidity (%RH) — last window")
        ax_hum.set_xlabel("Time (s) (past → now)")
        ax_hum.set_ylabel("%RH")
        if any_data:
            ax_hum.legend(loc="upper left", fontsize="small")
        ax_hum.set_xlim(-self.window_seconds, 0)
        ax_hum.relim(); ax_hum.autoscale_view()

        # Ensure parent directory exists and save.
        os.makedirs(os.path.dirname(self.save_png_path) or ".", exist_ok=True)
        try:
            fig.savefig(self.save_png_path, dpi=120)
            self.get_logger().info(f"Saved final plot to: {self.save_png_path}")
        except Exception as e:
            self.get_logger().warn(f"PNG save failed: {e}")
        finally:
            plt.close(fig)

    # ---------- Cleanup ----------

    def destroy_node(self):
        """Flush CSV, save the final plot, then close files and destroy the node."""
        # Save the final plot before tearing anything down.
        self._save_plot_once()

        # Flush/close CSV if needed.
        if self.csv_file:
            try:
                self.csv_file.flush()
                self.csv_file.close()
            except Exception:
                pass

        # Finish the base class cleanup.
        super().destroy_node()


def main():
    # Command-line flags: keep it simple.
    parser = argparse.ArgumentParser(description="Multi-board temp/hum logger with final PNG at exit.")
    parser.add_argument("--window-seconds", type=int, default=60,
                        help="How much recent data to keep in memory for the final plot (default: 60s).")
    parser.add_argument("--save-csv", type=str, default="/root/ros2_ws/data.csv",
                        help="CSV file path (default: /root/ros2_ws/data.csv).")
    parser.add_argument("--save-png", type=str, default="/root/ros2_ws/plot.png",
                        help="PNG file path saved once on exit (default: /root/ros2_ws/plot.png).")

    # Accept ROS 2’s extra CLI args without crashing (e.g., when launched).
    args, unknown = parser.parse_known_args()
    rclpy.init(args=unknown)

    node = MultiBoardPlotNode(
        window_seconds=args.window_seconds,
        save_csv=args.save_csv,
        save_png=args.save_png,
        flush_every=20,  # flush CSV every 20 rows
    )

    try:
        # Normal single-threaded spin; Ctrl+C breaks out.
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Ensure rclpy shuts down cleanly and our node runs its destroy logic (saves PNG/CSV).
        if rclpy.ok():
            rclpy.shutdown()
        node.destroy_node()


# Standard Python entry point
if __name__ == "__main__":
    main()
