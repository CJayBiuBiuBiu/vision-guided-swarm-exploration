"""Record run-isolated camera images and synchronized laser clearance labels."""

import argparse
import csv
import hashlib
import math
import re
import sys
import threading
from pathlib import Path

import cv2
import numpy as np
import rospy
from sensor_msgs.msg import Image, LaserScan


FIELDS = ("image_path", "image_sha256", "run_id", "layout", "experiment_seed",
          "robot", "camera_stamp_ns", "scan_stamp_ns", "camera_fov_rad",
          "sync_tolerance_ns", "left", "centre", "right")


class CalibrationCollector:
    def __init__(self, args, lidar_openness):
        self.args = args
        self.lidar_openness = lidar_openness
        self.interval_ns = round(args.interval * 1e9)
        self.tolerance_ns = round(args.sync_tolerance * 1e9)
        self.scans = {}
        self.last_saved = {}
        self.lock = threading.Lock()
        self.count = 0
        self.run = args.output / args.run_id
        self.run.mkdir(parents=True)
        (self.run / "images").mkdir()
        self.stream = (self.run / "manifest.csv").open("x", newline="")
        self.writer = csv.writer(self.stream)
        self.writer.writerow(FIELDS)
        self.stream.flush()
        rospy.on_shutdown(self.close)
        self.subscribers = []
        for robot in args.robots:
            self.subscribers.append(rospy.Subscriber(
                f"/{robot}/scan", LaserScan, self.on_scan, callback_args=robot, queue_size=2))
            self.subscribers.append(rospy.Subscriber(
                f"/{robot}/camera/rgb/image_raw", Image, self.on_image, callback_args=robot, queue_size=2))

    def close(self):
        with self.lock:
            self.stream.close()

    def on_scan(self, msg, robot):
        with self.lock:
            self.scans[robot] = msg

    def on_image(self, msg, robot):
        stamp = msg.header.stamp.to_nsec()
        if stamp <= 0 or msg.encoding not in ("rgb8", "bgr8") or msg.step < msg.width * 3:
            return
        with self.lock:
            scan = self.scans.get(robot)
            if (self.stream.closed or scan is None or self.count >= self.args.samples
                    or stamp - self.last_saved.get(robot, -self.interval_ns) < self.interval_ns):
                return
            scan_stamp = scan.header.stamp.to_nsec()
            if scan_stamp <= 0 or abs(stamp - scan_stamp) > self.tolerance_ns:
                return
            labels = self.lidar_openness(
                scan.ranges, scan.angle_min, scan.angle_increment,
                scan.range_min, scan.range_max, self.args.camera_fov)
            if labels is None:
                return
            try:
                rows = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.step)
                pixels = rows[:, :msg.width * 3].reshape(msg.height, msg.width, 3)
            except ValueError:
                return
            bgr = pixels[:, :, ::-1] if msg.encoding == "rgb8" else pixels
            ok, encoded = cv2.imencode(".png", bgr)
            if not ok:
                raise ValueError(f"Could not encode camera frame for {robot}")
            payload = encoded.tobytes()
            image = Path("images") / f"{robot}_{stamp:019d}.png"
            with (self.run / image).open("xb") as stream:
                stream.write(payload)
            self.writer.writerow((
                image.as_posix(), hashlib.sha256(payload).hexdigest(), self.args.run_id,
                self.args.layout, self.args.seed, robot, stamp, scan_stamp,
                self.args.camera_fov, self.tolerance_ns, *labels))
            self.stream.flush()
            self.count += 1
            self.last_saved[robot] = stamp
            finished = self.count == self.args.samples
        if finished:
            rospy.signal_shutdown("Requested calibration samples collected")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("ros_project", type=Path)
    parser.add_argument("output", type=Path)
    parser.add_argument("--run-id", required=True)
    parser.add_argument("--layout", required=True)
    parser.add_argument("--seed", type=int, required=True)
    parser.add_argument("--robots", nargs="+", default=["robot1", "robot2", "robot3"])
    parser.add_argument("--samples", type=int, default=120)
    parser.add_argument("--interval", type=float, default=0.25)
    parser.add_argument("--sync-tolerance", type=float, default=0.15)
    parser.add_argument("--camera-fov", type=float, default=1.085595)
    args = parser.parse_args(rospy.myargv()[1:])
    names = (args.run_id, args.layout, *args.robots)
    if not all(re.fullmatch(r"[A-Za-z0-9][A-Za-z0-9_-]*", name) for name in names):
        parser.error("Run, layout and robot names must be simple identifiers")
    if (args.seed < 0 or args.samples <= 0 or not 0 < args.interval < math.inf
            or not 0 < args.sync_tolerance <= args.interval
            or not 0 < args.camera_fov <= math.pi
            or len(set(args.robots)) != len(args.robots)):
        parser.error("Invalid seed, sampling interval, timestamp tolerance, field of view or robot list")
    package = args.ros_project.resolve() / "turtlebot_ddpg/scripts/voronoi_exploration"
    sys.path.insert(0, str(package))
    from directional_clearance import lidar_openness

    rospy.init_node("clearance_calibration_collector")
    collector = CalibrationCollector(args, lidar_openness)
    rospy.spin()
    if collector.count != args.samples:
        raise SystemExit(f"Collected {collector.count} of {args.samples} requested frames")


if __name__ == "__main__":
    main()
