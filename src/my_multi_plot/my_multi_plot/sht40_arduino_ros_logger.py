#!/usr/bin/env python3
"""
ROS 2 multi-board temperature/humidity logger.

Main features:
- Saves results into Results/data (CSV files) and Results/plots (PNG plots)
- Creates one CSV per sensor (device)
- Each CSV row contains: timestamp, humidity, temperature
- Pairs humidity and temperature messages that arrive close together
- Names files as data_<start_time>_<end_time>.csv and .png
"""

import argparse              # For command-line argument parsing
import csv                   # To write CSV files
import os                    # To handle file paths and folders
import re                    # For regular expression parsing of topic names
import time                  # For timestamps and timing
from collections import defaultdict, deque   # Efficient data storage for time-series
from dataclasses import dataclass             # Simple data structure definitions
from typing import Deque, Dict, Optional, Tuple

import rclpy                # ROS 2 Python client library
from rclpy.node import Node  # Base class for ROS 2 nodes
from rclpy.qos import QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy  # QoS for topics
from std_msgs.msg import Float32              # Message type for temperature/humidity

from my_multi_plot.utils.influxdb import InfluxCollector
from my_multi_plot.components.secrets import influx_secret

# Use non-GUI (headless) matplotlib backend for environments without display
import matplotlib
if "DISPLAY" not in os.environ:
    matplotlib.use("Agg")
import matplotlib.pyplot as plt               # For plotting graphs

# Regular expression to match topic names like /sensor1/temperature or /sensor1/humidity
DEVICE_TOPIC_RE = re.compile(r"^/?([^/]+)/(?:(temperature)|(humidity))$")

# ---------------------------------------------------------------------
# Data structures to organize readings
# ---------------------------------------------------------------------

@dataclass
class Series:
    """Stores timestamps and values for plotting."""
    times: Deque[float]
    values: Deque[float]

@dataclass
class CsvHandle:
    """Keeps a CSV file handle and writer for each device."""
    fh: any
    writer: csv.writer
    tmp_path: str
    rows_since_flush: int = 0   # Tracks how many rows since last flush to disk

# ---------------------------------------------------------------------
# Main Node class
# ---------------------------------------------------------------------

class MultiBoardPlotNode(Node):
    """
    ROS 2 Node that:
    - Subscribes dynamically to <device>/temperature and <device>/humidity topics
    - Logs readings to per-device CSVs
    - Generates a final plot on shutdown
    """

    def __init__(self,
                 window_seconds: int,
                 results_dir: str,
                 pair_tolerance_s: float,
                 stale_timeout_s: float,
                 csv_flush_every: int,
                 cloud: bool = False,
                 no_csv: bool = False,
                 no_png: bool = False):
        super().__init__('multi_board_plot_listener')

        self.no_csv = no_csv
        self.no_png = no_png

        # Cloud sender (optional)
        self.influx = None
        if cloud:
            tags = {"device": "ROS2_PI_SHT40"}  # adjust as needed
            self.influx = InfluxCollector(tags, influx_secret)
            self.influx.start_send_thread()
            self.get_logger().info("Cloud upload: ON")

        # ---- Folder setup ----
        self.results_root = results_dir
        self.data_dir = os.path.join(self.results_root, "data")
        self.plots_dir = os.path.join(self.results_root, "plots")
        if not self.no_csv:
            os.makedirs(self.data_dir, exist_ok=True) # Create Results/data folder
        if not self.no_png:
            os.makedirs(self.plots_dir, exist_ok=True) # Create Results/plots folder
        # os.makedirs(self.data_dir, exist_ok=True)   
        # os.makedirs(self.plots_dir, exist_ok=True)  

        # Record start time for naming output files later
        self.run_started_ts = time.time()
        self.start_tag = time.strftime("%Y%m%d-%H%M%S", time.localtime(self.run_started_ts))

        # ---- Configuration parameters ----
        self.window_seconds = window_seconds           # How much recent data to keep for plotting
        self.pair_tolerance_s = max(0.0, pair_tolerance_s)  # Time window to consider readings “simultaneous”
        self.stale_timeout_s = max(0.1, stale_timeout_s)    # Max wait before writing unpaired data
        self.csv_flush_every = max(1, csv_flush_every)      # Flush to disk every N rows

        # QoS (Quality of Service) for reliable topic subscription
        self.qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,  # Reliable delivery (no intentional drops)
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10
        )

        # ---- Data structures ----
        # Store time-series for plotting
        self.series_temp: Dict[str, Series] = defaultdict(lambda: Series(deque(), deque()))
        self.series_hum:  Dict[str, Series] = defaultdict(lambda: Series(deque(), deque()))

        # Keep track of what topics we’ve already subscribed to
        self.subscribed_topics = set()

        # Dictionary to hold CSV writers per device
        self.csv_handles: Dict[str, CsvHandle] = {}

        # Store most recent unpaired readings awaiting their counterpart
        self.unpaired: Dict[str, Dict[str, Optional[Tuple[float, float]]]] = defaultdict(
            lambda: {"temperature": None, "humidity": None}
        )

        # ---- Timers ----
        # Check every second for new topics
        self.scan_timer  = self.create_timer(1.0, self._scan_for_topics)
        # Periodically remove old samples from memory
        self.prune_timer = self.create_timer(1.0, self._prune_old_samples)
        # Flush stale (unpaired) samples every 0.5s
        self.flush_timer = self.create_timer(0.5, self._flush_stale_unpaired)

        # Startup log message
        csv_msg = (f"Saving CSVs in {self.data_dir}; " if not self.no_csv else "NOT saving CSVs; ")
        png_msg = (f"PNG in {self.plots_dir}. " if not self.no_png else "NOT saving PNG. ")
        cloud_msg = ("Cloud upload ON. " if self.influx else "")
        self.get_logger().info(
            "Ready. Watching '<device>/temperature' & '<device>/humidity'. "
            f"{csv_msg}{png_msg}{cloud_msg}"
            f"Pair tolerance={self.pair_tolerance_s}s; stale timeout={self.stale_timeout_s}s."
        )


    # ---------------------------------------------------------------------
    # Discovery — subscribe to any new temperature/humidity topics
    # ---------------------------------------------------------------------

    def _scan_for_topics(self):
        # Get all currently active ROS topics
        for topic_name, types in self.get_topic_names_and_types():
            m = DEVICE_TOPIC_RE.match(topic_name)
            if not m or topic_name in self.subscribed_topics:
                continue  # Skip if not matching or already subscribed

            # Ensure message type is Float32
            if "std_msgs/msg/Float32" not in types:
                self.get_logger().warn(f"Skipping {topic_name}: not Float32")
                continue

            device = m.group(1)                         # e.g. "sensor1"
            kind = "temperature" if m.group(2) else "humidity"

            # Subscribe dynamically with a lambda capturing device and kind
            self.create_subscription(
                Float32,
                topic_name,
                lambda msg, d=device, k=kind: self._on_value(d, k, msg),
                self.qos
            )
            self.subscribed_topics.add(topic_name)       # Remember this topic
            self.get_logger().info(f"Subscribed to {topic_name} (device={device}, kind={kind})")

            # Create CSV file for this device if not yet done
            self._ensure_csv_for_device(device)

    def _ensure_csv_for_device(self, device: str):
        """Optional -- Open a temporary CSV file for this device and write header row."""
        if device in self.csv_handles:
            return
        if self.no_csv:
            return
        tmp_path = os.path.join(self.data_dir, f"tmp_{device}.csv")
        fh = open(tmp_path, "w", newline="")
        writer = csv.writer(fh)
        writer.writerow(["ts", "humidity", "temperature"])  # Header
        self.csv_handles[device] = CsvHandle(fh=fh, writer=writer, tmp_path=tmp_path)

    # ---------------------------------------------------------------------
    # Data reception — handle each temperature/humidity message
    # ---------------------------------------------------------------------

    def _on_value(self, device: str, kind: str, msg: Float32):
        """Handle a new Float32 reading for given device and kind."""
        now = time.time()
        val = float(msg.data)

        # Add to plotting buffer
        series = self.series_temp[device] if kind == "temperature" else self.series_hum[device]
        series.times.append(now)
        series.values.append(val)

        # Try to pair with the opposite kind (humidity ↔ temperature)
        other_kind = "humidity" if kind == "temperature" else "temperature"
        my_sample = (now, val)
        other_sample = self.unpaired[device][other_kind]

        # If the other type arrived within the allowed time window, pair them
        if other_sample and abs(my_sample[0] - other_sample[0]) <= self.pair_tolerance_s:
            ts = max(my_sample[0], other_sample[0])   # Use latest timestamp
            temp = val if kind == "temperature" else other_sample[1]
            hum  = val if kind == "humidity" else other_sample[1]
            self._write_row(device, ts, hum, temp)
            self.unpaired[device][other_kind] = None  # Clear paired sample
        else:
            # Store this reading to await its counterpart
            self.unpaired[device][kind] = my_sample

    # ---------------------------------------------------------------------
    # Periodic flush of stale unpaired samples
    # ---------------------------------------------------------------------

    def _flush_stale_unpaired(self):
        """Write out any unpaired samples that waited too long."""
        cutoff = time.time() - self.stale_timeout_s
        for device, kinds in list(self.unpaired.items()):
            for kind in ("temperature", "humidity"):
                samp = kinds[kind]
                if samp and samp[0] < cutoff:
                    ts, val = samp
                    # Write single reading with missing partner blank
                    if kind == "temperature":
                        self._write_row(device, ts, hum=None, temp=val)
                    else:
                        self._write_row(device, ts, hum=val, temp=None)
                    kinds[kind] = None  # Clear entry after writing

    # ---------------------------------------------------------------------
    # CSV writing
    # ---------------------------------------------------------------------

    def _write_row(self, device: str, ts: float, hum: Optional[float], temp: Optional[float]):
        """Write a single row to the device CSV file and (optionally) upload."""
        # 1) Local CSV (optional)
        if not self.no_csv:
            handle = self.csv_handles.get(device)
            if not handle:
                self._ensure_csv_for_device(device)
                handle = self.csv_handles[device]
            handle.writer.writerow([ts,
                                    "" if hum is None else hum,
                                    "" if temp is None else temp])
            handle.rows_since_flush += 1
            if handle.rows_since_flush >= self.csv_flush_every:
                try:
                    handle.fh.flush()
                except Exception:
                    pass
                handle.rows_since_flush = 0

        # 2) Cloud upload (optional)
        if self.influx:
            fields = {}
            if hum is not None:
                fields["humidity"] = float(hum)
            if temp is not None:
                fields["temperature"] = float(temp)

            # Only send if we have at least one field
            if fields:
                self.influx.record_fields_tags(
                    measurement="sht40_readings",
                    fields=fields,
                    tags={"device": device}
                )

    # ---------------------------------------------------------------------
    # Data cleanup — remove old points from memory
    # ---------------------------------------------------------------------

    def _prune_old_samples(self):
        """Keep only recent samples within the window_seconds timeframe."""
        cutoff = time.time() - self.window_seconds
        for series in list(self.series_temp.values()) + list(self.series_hum.values()):
            while series.times and series.times[0] < cutoff:
                series.times.popleft()
                series.values.popleft()

    # ---------------------------------------------------------------------
    # Plot saving (once, on shutdown)
    # ---------------------------------------------------------------------

    def _save_plot_once(self, end_ts: float):
        """Generate and save one plot showing the latest data window."""
        end_tag = time.strftime("%Y%m%d-%H%M%S", time.localtime(end_ts))
        png_path = os.path.join(self.plots_dir, f"data_{self.start_tag}_{end_tag}.png")

        # Create a 2-row figure: top for temperature, bottom for humidity
        fig = plt.figure(constrained_layout=True, figsize=(9, 6))
        gs = fig.add_gridspec(2, 1)
        ax_temp = fig.add_subplot(gs[0, 0])
        ax_hum  = fig.add_subplot(gs[1, 0])

        # now = time.time()
        any_data = False

        # --- Temperature plot ---
        x_max_temp = 0.0
        any_data = False
        for device, series in self.series_temp.items():
            if series.times:
                x = [t - self.run_started_ts for t in series.times]  # seconds since start
                y = list(series.values)
                ax_temp.plot(x, y, label=device)
                x_max_temp = max(x_max_temp, x[-1])
                any_data = True
        ax_temp.set_title("Temperature (°C)")
        ax_temp.set_xlabel("Time (s) since start")
        ax_temp.set_ylabel("°C")
        if any_data:
            ax_temp.legend(loc="upper left", fontsize="small")
            ax_temp.set_xlim(0, x_max_temp)

        # --- Humidity plot ---
        x_max_hum = 0.0
        any_data = False
        for device, series in self.series_hum.items():
            if series.times:
                x = [t - self.run_started_ts for t in series.times]
                y = list(series.values)
                ax_hum.plot(x, y, label=device)
                x_max_hum = max(x_max_hum, x[-1])
                any_data = True
        ax_hum.set_title("Humidity (%RH)")
        ax_hum.set_xlabel("Time (s) since start")
        ax_hum.set_ylabel("%RH")
        if any_data:
            ax_hum.legend(loc="upper left", fontsize="small")
            ax_hum.set_xlim(0, x_max_hum)

        # Try saving PNG
        try:
            fig.savefig(png_path, dpi=120)
            self.get_logger().info(f"Saved plot: {png_path}")
        except Exception as e:
            self.get_logger().warn(f"Failed to save plot: {e}")
        finally:
            plt.close(fig)  # Free memory

    # ---------------------------------------------------------------------
    # Cleanup and file finalization on shutdown
    # ---------------------------------------------------------------------

    def destroy_node(self):
        """Called when shutting down; flush everything and rename files."""
        # Flush any leftover samples
        self._flush_stale_unpaired()

        end_ts = time.time()
        # Save final plot
        if not self.no_png:
            self._save_plot_once(end_ts)
        # Rename temporary CSV files with start and end timestamps
        end_tag = time.strftime("%Y%m%d-%H%M%S", time.localtime(end_ts))
        for device, handle in list(self.csv_handles.items()):
            try:
                handle.fh.flush()
                handle.fh.close()
            except Exception:
                pass
            final_name = f"{device}_data_{self.start_tag}_{end_tag}.csv"
            final_path = os.path.join(self.data_dir, final_name)
            try:
                os.replace(handle.tmp_path, final_path)   # Rename safely
                self.get_logger().info(f"Saved CSV: {final_path}")
            except Exception as e:
                self.get_logger().warn(f"Failed to rename CSV for {device}: {e}")

        # Stop Influx thread cleanly
        if self.influx:
            try:
                self.influx.stop_send_thread()
            except Exception:
                pass

        super().destroy_node()

# ---------------------------------------------------------------------
# Main entry point
# ---------------------------------------------------------------------

def main():
    """Parse arguments and start the ROS node."""
    parser = argparse.ArgumentParser(description="Multi-board temp/humidity logger.")
    parser.add_argument("--window-seconds", type=int, default=60, help="Plot window length (s).")
    parser.add_argument("--results-dir", type=str, default="Results", help="Base output folder.")
    parser.add_argument("--pair-tolerance-s", type=float, default=0.2, help="Pairing time tolerance (s).")
    parser.add_argument("--stale-timeout-s", type=float, default=2.0, help="How long to wait before writing unpaired sample (s).")
    parser.add_argument("--csv-flush-every", type=int, default=20, help="Flush CSV after this many rows.")
    parser.add_argument("--cloud", action="store_true",
                    help="Enable upload to InfluxDB")
    parser.add_argument("--no-csv", action="store_true",
                    help="Disable writing CSVs")
    parser.add_argument("--no-png", action="store_true",
                    help="Disable final PNG plot")


    args, unknown = parser.parse_known_args()
    rclpy.init(args=unknown)

    # Create and run the node
    node = MultiBoardPlotNode(
    window_seconds=args.window_seconds,
    results_dir=args.results_dir,
    pair_tolerance_s=args.pair_tolerance_s,
    stale_timeout_s=args.stale_timeout_s,
    csv_flush_every=args.csv_flush_every,
    cloud=args.cloud,
    no_csv=args.no_csv,
    no_png=args.no_png,
    )

    try:
        rclpy.spin(node)  # Keep running until Ctrl+C
    except KeyboardInterrupt:
        pass
    finally:
        # Graceful teardown: destroy node first, then shutdown ROS
        try:
            node.destroy_node()
        finally:
            if rclpy.ok():
                rclpy.shutdown()

if __name__ == "__main__":
    main()
