"""Read-only by default. Explicit opt-in required for motion."""

import argparse
import csv
import math
import time
from pathlib import Path

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage, Image

from .estimator import FixedGainEstimator


def yaw_from_quaternion(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def wrap_angle(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


class Experiment(Node):
    def __init__(self, args):
        super().__init__("rm_odometry_phase3")
        self.args = args
        self.filter = FixedGainEstimator(args.initial_distance, args.gain)
        self.odom = None
        self.last_pose = None
        self.last_odom_stamp = None
        self.initial_yaw = None
        self.latest_camera = None
        self.last_camera_stamp = None
        self.last_correction_stamp = None
        self.camera_scale = None
        self.calibration_width = None
        self.calibration_distance = None
        self.camera_valid = False
        self.latest_width = float("nan")
        self.latest_offset = float("nan")
        self.latest_range = float("nan")
        self.latest_detection = 0.0
        self.start_wall = time.monotonic()
        self.drive_start = None
        self.stop_start = None
        self.mode = "WAIT_ODOM"
        self.output = Path(args.output).expanduser()
        self.output.parent.mkdir(parents=True, exist_ok=True)
        self.file = self.output.open("w", newline="")
        self.writer = csv.DictWriter(
            self.file,
            fieldnames=[
                "stamp_s",
                "mode",
                "odom_x_m",
                "odom_y_m",
                "odom_forward_m",
                "odom_remaining_m",
                "predicted_remaining_m",
                "camera_remaining_m",
                "corrected_remaining_m",
                "innovation_m",
                "correction_applied",
                "tag_width_px",
                "tag_offset",
                "yaw_error_rad",
                "wheel_v_mps",
                "camera_age_s",
                "correction_count",
            ],
        )
        self.writer.writeheader()
        self.pub = self.create_publisher(Twist, args.cmd_topic, 10)
        self.create_subscription(Odometry, args.odom_topic, self.on_odom, 10)
        if args.camera_topic:
            msg_type = Image if args.raw_image else CompressedImage
            self.create_subscription(msg_type, args.camera_topic, self.on_image, 5)
        self.timer = self.create_timer(0.05, self.tick)
        self.get_logger().info(
            f"Started in read-only mode={not args.enable_motion}; camera={args.camera_topic or 'disabled'}"
        )

    def stop(self):
        if self.args.enable_motion:
            for _ in range(3):
                self.pub.publish(Twist())

    def on_odom(self, msg):
        now = time.monotonic()
        p = msg.pose.pose.position
        yaw = yaw_from_quaternion(msg.pose.pose.orientation)
        if self.initial_yaw is None:
            self.initial_yaw = yaw
        if self.last_pose is None:
            self.last_pose = (p.x, p.y)
        else:
            dx = p.x - self.last_pose[0]
            dy = p.y - self.last_pose[1]
            forward_delta = dx * math.cos(self.initial_yaw) + dy * math.sin(
                self.initial_yaw
            )
            self.filter.predict(forward_delta)
            self.last_pose = (p.x, p.y)
        self.odom = msg
        self.last_odom_stamp = now

    def on_image(self, msg):
        try:
            import cv2
            import numpy as np

            if self.args.raw_image:
                if msg.encoding not in ("rgb8", "bgr8", "mono8"):
                    raise ValueError(f"Unsupported raw image encoding {msg.encoding}")
                channels = 1 if msg.encoding == "mono8" else 3
                arr = np.frombuffer(msg.data, dtype=np.uint8)
                img = arr.reshape((msg.height, msg.step))[:, : msg.width * channels]
                img = (
                    img.reshape((msg.height, msg.width, channels))
                    if channels == 3
                    else img
                )
                if msg.encoding == "rgb8":
                    img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
            else:
                img = cv2.imdecode(
                    np.frombuffer(msg.data, dtype=np.uint8), cv2.IMREAD_COLOR
                )
            if img is None:
                return
            detector = getattr(self, "detector", None)
            if detector is None:
                self.detector = cv2.QRCodeDetector()
                detector = self.detector
            text, corners, _ = detector.detectAndDecode(img)
            if corners is None or not text:
                return
            pts = corners.reshape(4, 2)
            width = (math.dist(pts[0], pts[1]) + math.dist(pts[2], pts[3])) / 2
            if width < 8:
                return
            center_x = float(pts[:, 0].mean())
            self.latest_offset = (center_x - img.shape[1] / 2) / (img.shape[1] / 2)
            self.latest_width = width
            self.latest_detection = time.monotonic()
            self.camera_valid = True
            if self.camera_scale is not None:
                self.latest_range = self.camera_scale / width
        except Exception as exc:
            if not getattr(self, "camera_error_reported", False):
                self.get_logger().error(f"Camera processing failed: {exc}")
                self.camera_error_reported = True

    def tick(self):
        now = time.monotonic()
        if self.odom is None:
            if now - self.start_wall > self.args.sensor_timeout:
                self.get_logger().error("No odometry received; stopping")
                self.mode = "ABORT"
                self.stop()
            return
        if now - self.last_odom_stamp > self.args.sensor_timeout:
            self.mode = "ABORT"
            self.stop()
            self.get_logger().error("Odometry timeout; stopping")
            self.timer.cancel()
            self.file.flush()
            return
        p = self.odom.pose.pose.position
        yaw = yaw_from_quaternion(self.odom.pose.pose.orientation)
        yaw_error = wrap_angle(yaw - self.initial_yaw)
        odom_forward = (
            (p.x - self.args_origin_x) * math.cos(self.initial_yaw)
            + (p.y - self.args_origin_y) * math.sin(self.initial_yaw)
            if hasattr(self, "args_origin_x")
            else 0.0
        )
        if not hasattr(self, "args_origin_x"):
            self.args_origin_x, self.args_origin_y = p.x, p.y
        age = now - self.latest_detection if self.latest_detection else float("inf")
        fresh = age <= self.args.camera_timeout
        correction_applied = 0
        self.filter.innovation = float("nan")
        if self.mode == "WAIT_ODOM":
            self.mode = "CALIBRATE" if self.args.camera_topic else "READY"
        if self.mode == "CALIBRATE":
            if fresh and math.isfinite(self.latest_width):
                # Width is measured in pixels; calibration requires known physical distance.
                self.camera_scale = self.args.initial_distance * self.latest_width
                self.calibration_width = self.latest_width
                self.calibration_distance = self.args.initial_distance
                self.latest_range = self.args.initial_distance
                self.mode = (
                    "ALIGN"
                    if self.args.enable_motion and not self.args.skip_alignment
                    else "READY"
                )
                self.get_logger().info(
                    f"QR calibrated: width={self.latest_width:.1f}px at {self.args.initial_distance:.3f}m"
                )
            elif now - self.start_wall > self.args.align_timeout:
                self.get_logger().error("No decodable QR code for calibration")
                self.mode = "ABORT"
                self.stop()
        if self.camera_scale is not None and fresh and math.isfinite(self.latest_range):
            if self.last_correction_stamp != self.latest_detection and self.mode in (
                "READY",
                "DRIVE",
                "STOPPING",
                "DONE",
            ):
                if (
                    abs(self.latest_range - self.filter.distance)
                    <= self.args.max_innovation
                ):
                    self.filter.correct(self.latest_range)
                    correction_applied = 1
                self.last_correction_stamp = self.latest_detection
        if self.args.enable_motion:
            if self.mode == "ALIGN":
                if now - self.start_wall > self.args.align_timeout:
                    self.get_logger().error("QR alignment timed out")
                    self.mode = "ABORT"
                    self.stop()
                elif fresh:
                    if abs(self.latest_offset) <= self.args.align_tolerance:
                        self.mode = "READY"
                        self.stop()
                    else:
                        cmd = Twist()
                        cmd.angular.z = max(
                            -self.args.align_speed,
                            min(
                                self.args.align_speed,
                                -self.args.align_kp * self.latest_offset,
                            ),
                        )
                        self.pub.publish(cmd)
                else:
                    self.stop()
            elif self.mode == "READY":
                if (
                    not self.args.skip_alignment
                    and self.args.camera_topic
                    and not fresh
                ):
                    self.stop()
                elif not self.args.camera_topic:
                    self.get_logger().error(
                        "Motion requires camera or explicit --skip-alignment"
                    )
                    self.mode = "ABORT"
                else:
                    self.mode = "DRIVE"
                    self.drive_start = now
            elif self.mode == "DRIVE":
                if now - self.drive_start > self.args.max_drive_time:
                    self.mode = "ABORT"
                    self.stop()
                elif self.camera_scale is not None and not fresh:
                    self.mode = "ABORT"
                    self.stop()
                    self.get_logger().error("Camera target lost; stopped")
                elif (
                    self.filter.distance
                    <= self.args.stop_distance + self.args.safety_margin
                ):
                    self.mode = "STOPPING"
                    self.stop_start = now
                    self.stop()
                else:
                    cmd = Twist()
                    cmd.linear.x = self.args.speed
                    cmd.angular.z = max(
                        -self.args.drive_yaw_limit,
                        min(
                            self.args.drive_yaw_limit,
                            -self.args.drive_yaw_kp * yaw_error,
                        ),
                    )
                    self.pub.publish(cmd)
            elif self.mode == "STOPPING":
                self.stop()
                if now - self.stop_start >= self.args.settle_time:
                    self.mode = "DONE"
                    self.get_logger().info(
                        f"STOPPED: fused={self.filter.distance:.4f}m odom={self.args.initial_distance - odom_forward:.4f}m"
                    )
        if not self.args.enable_motion and now - self.start_wall >= self.args.duration:
            self.mode = "DONE"
        self.writer.writerow(
            {
                "stamp_s": time.time(),
                "mode": self.mode,
                "odom_x_m": p.x,
                "odom_y_m": p.y,
                "odom_forward_m": odom_forward,
                "odom_remaining_m": self.args.initial_distance - odom_forward,
                "predicted_remaining_m": self.filter.prediction,
                "camera_remaining_m": self.latest_range if fresh else float("nan"),
                "corrected_remaining_m": self.filter.distance,
                "innovation_m": self.filter.innovation,
                "correction_applied": correction_applied,
                "tag_width_px": self.latest_width if fresh else float("nan"),
                "tag_offset": self.latest_offset if fresh else float("nan"),
                "yaw_error_rad": yaw_error,
                "wheel_v_mps": self.odom.twist.twist.linear.x,
                "camera_age_s": age if math.isfinite(age) else float("nan"),
                "correction_count": self.filter.corrections,
            }
        )
        self.file.flush()
        if self.mode in ("DONE", "ABORT"):
            self.stop()
            self.timer.cancel()

    def close(self):
        self.stop()
        self.file.flush()
        self.file.close()


def main():
    parser = argparse.ArgumentParser(
        description="QR range + odometry fixed-gain correction; no Kalman filter"
    )
    parser.add_argument("--enable-motion", action="store_true")
    parser.add_argument(
        "--skip-alignment",
        action="store_true",
        help="Only for supervised tests; QR still required for ranging",
    )
    parser.add_argument(
        "--camera-topic", default="", help="ROS Image or CompressedImage topic"
    )
    parser.add_argument("--raw-image", action="store_true")
    parser.add_argument("--odom-topic", default="/odom")
    parser.add_argument("--cmd-topic", default="/cmd_vel")
    parser.add_argument("--initial-distance", type=float, default=1.5)
    parser.add_argument("--stop-distance", type=float, default=0.4)
    parser.add_argument("--safety-margin", type=float, default=0.08)
    parser.add_argument("--speed", type=float, default=0.10)
    parser.add_argument("--gain", type=float, default=0.30)
    parser.add_argument("--max-innovation", type=float, default=0.30)
    parser.add_argument("--camera-timeout", type=float, default=0.75)
    parser.add_argument("--sensor-timeout", type=float, default=2.0)
    parser.add_argument("--align-timeout", type=float, default=20.0)
    parser.add_argument("--align-tolerance", type=float, default=0.05)
    parser.add_argument("--align-speed", type=float, default=0.12)
    parser.add_argument("--align-kp", type=float, default=0.5)
    parser.add_argument("--drive-yaw-kp", type=float, default=0.6)
    parser.add_argument("--drive-yaw-limit", type=float, default=0.12)
    parser.add_argument("--max-drive-time", type=float, default=18.0)
    parser.add_argument("--settle-time", type=float, default=1.0)
    parser.add_argument(
        "--duration", type=float, default=10.0, help="Read-only logging duration"
    )
    parser.add_argument("--output", default="~/robomaster_ws/phase3_data/trial_01.csv")
    args = parser.parse_args()
    if (
        not 0 <= args.gain <= 1
        or args.initial_distance <= args.stop_distance + args.safety_margin
    ):
        parser.error("Invalid gain or distance thresholds")
    if args.enable_motion and not args.camera_topic:
        parser.error("Motion requires camera topic for range correction")
    if args.speed <= 0 or args.speed > 0.16:
        parser.error("Speed must be in (0, 0.16] m/s")
    rclpy.init()
    node = Experiment(args)
    try:
        while rclpy.ok() and node.mode not in ("DONE", "ABORT"):
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        node.get_logger().warning("Interrupted by operator")
    finally:
        node.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
