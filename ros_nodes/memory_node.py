#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import serial
import time
import os
import csv
import random
import math

# === original paths & constants (unchanged) ===
baseline_reflection_csv = "/home/qaim/Desktop/ros2_ws/baseline_reflection_log.csv"
MAX_LEN_DATA = 30
X_PNN = 50

# ---------- DUMMY HELPERS ----------
def _synth_rr_ms(t0, hr_bpm=72.0, wobble_amp_bpm=6.0, wobble_period_s=20.0, noise_ms=15.0):
    """
    Generate a plausible RR interval (ms).
    HR(t) = hr_bpm + wobble_amp*sin(2πt/T) + small noise
    RR = 60000 / HR
    """
    t = time.time() - t0
    hr = hr_bpm + wobble_amp_bpm * math.sin(2.0 * math.pi * t / max(1e-6, wobble_period_s))
    rr = 60000.0 / max(30.0, hr)  # clamp HR >= 30 bpm
    rr += random.gauss(0.0, noise_ms)
    # clamp RR to sane human range ~[300, 2000] ms
    return max(300.0, min(2000.0, rr))

class PnnxNode(Node):
    def __init__(self):
        super().__init__('pnnx2_node')

        # try to open the real sensor; if it fails, enable dummy mode
        self.dummy_mode = False
        try:
            self.sensor = serial.Serial('/dev/ttyACM0', 115200)
            self.get_logger().info("Connected to HRV sensor on /dev/ttyACM0 @115200.")
        except Exception as e:
            self.get_logger().warn(f"HRV sensor not found; switching to DUMMY mode: {e}")
            self.sensor = None
            self.dummy_mode = True
            self._dummy_t0 = time.time()
            self._dummy_last_emit = 0.0
            self._dummy_emit_period = 1.0  # <<< MATCH ORIGINAL SPEED (~1 msg/sec)

        # ROS pubs & state (unchanged)
        self.pub_pnnx = self.create_publisher(String, 'pnnx2', 10)
        self.pnnx = String()
        self.pnnx.data = "0.0"
        self.rri = 0.0
        self.rri_arr = []
        self.xx_count = 0

        # --- Experiment metadata (unchanged) ---
        self.phase = "Not set"
        self.pattern = "Not set"
        self.user_id = "Not set"
        self.session_id = "Not set"

        self.create_subscription(String, 'experiment_phase', self.phase_callback, 10)
        self.create_subscription(String, 'experiment_pattern', self.pattern_callback, 10)
        self.create_subscription(String, 'current_user_id', self.user_id_callback, 10)
        self.create_subscription(String, 'current_session_id', self.session_id_callback, 10)

    # --- control topic callbacks (unchanged) ---
    def phase_callback(self, msg):
        self.phase = msg.data
        self.get_logger().info(f"[PHASE] Updated to: {self.phase}")

    def pattern_callback(self, msg):
        self.pattern = msg.data
        self.get_logger().info(f"[PATTERN] Updated to: {self.pattern}")

    def user_id_callback(self, msg):
        self.user_id = msg.data
        self.get_logger().info(f"[USER_ID] Updated to: {self.user_id}")

    def session_id_callback(self, msg):
        self.session_id = msg.data
        self.get_logger().info(f"[SESSION_ID] Updated to: {self.session_id}")

    # --- gate: only publish after experiment control is set (kept as before) ---
    def ready_to_publish(self):
        fields = [
            self.phase.strip().lower(),
            self.pattern.strip().lower(),
            self.user_id.strip().lower(),
            self.session_id.strip().lower()
        ]
        if not all(x and x != "not set" for x in fields):
            self.get_logger().info(
                f"[WAIT] Not publishing: Waiting for experiment control input (phase, pattern, user, session). "
                f"Current: phase='{self.phase}', pattern='{self.pattern}', user_id='{self.user_id}', session_id='{self.session_id}'"
            )
            return False
        return True

    # ---------- Internal helper to process one RRI sample (shared by real+dummy) ----------
    def _process_rri_sample(self, rri_ms: float):
        self.rri = float(rri_ms)
        self.xx_count += 1

        if self.xx_count >= MAX_LEN_DATA + 2:
            self.rri_arr.pop(0)
            self.rri_arr.append(self.rri)
            count = 0
            for i in range(MAX_LEN_DATA):
                if abs(self.rri_arr[i] - self.rri_arr[i + 1]) > X_PNN:
                    count += 1

            pnnx_value = float(count / 30)  # SAME derivation as original
            timestamp = time.strftime('%Y-%m-%d %H:%M:%S')

            if not self.ready_to_publish():
                return

            # Publish (format preserved: plain float string)
            self.pnnx.data = str(pnnx_value)
            self.pub_pnnx.publish(self.pnnx)
            self.get_logger().info(f'Published: {pnnx_value}')

            # Log HRV to CSV (same file and headers as original)
            try:
                file_exists = os.path.isfile(baseline_reflection_csv)
                write_header = not file_exists or os.path.getsize(baseline_reflection_csv) == 0

                row = [
                    timestamp,
                    self.phase,
                    self.pattern,
                    self.user_id,
                    self.session_id,
                    pnnx_value
                ]
                with open(baseline_reflection_csv, "a", newline='') as f:
                    writer = csv.writer(f)
                    if write_header:
                        writer.writerow(["timestamp", "phase", "pattern", "user_id", "session", "valence"])
                    writer.writerow(row)
                self.get_logger().info(f"Sending to baseline_reflection_log.csv: {row}")
            except Exception as e:
                self.get_logger().error(f"[Baseline/Reflection][CSV ERROR] {e}")

        elif self.xx_count < MAX_LEN_DATA + 2:
            self.rri_arr.append(self.rri)
            self.get_logger().info(f'RRI_data_len: {len(self.rri_arr)}')

    def publish_pnnx(self):
        # REAL SENSOR PATH (unchanged)
        if not self.dummy_mode and self.sensor is not None:
            try:
                sensor_data = self.sensor.readline().decode(encoding='utf-8').strip()
            except Exception as e:
                self.get_logger().error(f"Serial error: {e}")
                return

            if sensor_data.startswith('Q'):  # Only process RRI lines
                self.get_logger().info(f'Sensor_data:{sensor_data}')
                try:
                    rri_ms = float(sensor_data[1:])
                except ValueError:
                    self.get_logger().warn(f"Could not parse RRI from: {sensor_data}")
                    return
                self._process_rri_sample(rri_ms)
            return

        # DUMMY PATH — emit exactly ~1 msg/sec (matches original feel)
        now = time.time()
        if now - self._dummy_last_emit >= self._dummy_emit_period:
            self._dummy_last_emit = now
            rri_ms = _synth_rr_ms(self._dummy_t0)
            self._process_rri_sample(rri_ms)

    def run(self):
        while rclpy.ok():
            self.publish_pnnx()
            rclpy.spin_once(self, timeout_sec=0.01)

def main(args=None):
    rclpy.init(args=args)
    talker = PnnxNode()
    try:
        talker.run()
    except KeyboardInterrupt:
        pass
    finally:
        talker.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
