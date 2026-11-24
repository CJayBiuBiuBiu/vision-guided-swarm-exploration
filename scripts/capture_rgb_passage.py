"""Save a stationary RGB frame, its pose and the paired planar scan in Gazebo."""

import argparse
from collections import deque
import json
import math
from pathlib import Path
import time

import cv2
import numpy as np
import rospy
from gazebo_msgs.msg import ModelStates
from sensor_msgs.msg import CameraInfo, Image, LaserScan

from passage_scene import DOOR_X, STAGING_Y
from rgb_passage import measure_passage, score_direction


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('output', type=Path)
    args = parser.parse_args()
    rospy.init_node('rgb_passage_stationary_probe')
    images, poses, scans, infos = (deque(maxlen=size) for size in (6, 60, 6, 2))

    def receive_pose(message):
        if 'robot2' in message.name:
            pose = message.pose[message.name.index('robot2')]
            poses.append((rospy.get_time(), (pose.position.x, pose.position.y, pose.position.z),
                          (pose.orientation.x, pose.orientation.y, pose.orientation.z,
                           pose.orientation.w)))

    subscribers = [
        rospy.Subscriber('/gazebo/model_states', ModelStates, receive_pose, queue_size=10),
        rospy.Subscriber('/robot2/camera/rgb/image_raw', Image, images.append,
                         queue_size=3, buff_size=4 * 1024 * 1024),
        rospy.Subscriber('/robot2/camera/rgb/camera_info', CameraInfo, infos.append,
                         queue_size=1),
        rospy.Subscriber('/robot2/scan', LaserScan, scans.append, queue_size=3),
    ]
    deadline = time.monotonic() + 85
    while time.monotonic() < deadline and not rospy.is_shutdown():
        time.sleep(0.04)
        if not all((images, poses, scans, infos)):
            continue
        frame = images[-1]
        stamp = frame.header.stamp.to_sec()
        if stamp < 2.0:
            continue
        scan = min(scans.copy(), key=lambda msg: abs(msg.header.stamp.to_sec() - stamp))
        scan_stamp = scan.header.stamp.to_sec()
        pose_stamp, position, quaternion = min(poses.copy(), key=lambda pose: abs(pose[0] - stamp))
        if abs(scan_stamp - stamp) > 0.15 or abs(pose_stamp - stamp) > 0.10:
            continue
        info = infos[-1]
        if (frame.encoding not in ('rgb8', 'bgr8')
                or (frame.width, frame.height) != (info.width, info.height)):
            raise ValueError('RGB encoding or dimensions do not match CameraInfo')
        rgb = np.ndarray((frame.height, frame.width, 3), dtype=np.uint8,
                         buffer=frame.data, strides=(frame.step, 3, 1))
        if frame.encoding == 'bgr8':
            rgb = rgb[..., ::-1]
        if math.hypot(position[0], position[1] - STAGING_Y) > 0.05:
            raise ValueError('Robot moved away from the stationary probe pose')
        intrinsics = [info.K[index] for index in (0, 4, 2, 5)]
        scores = measure_passage(rgb, intrinsics, (position, quaternion))
        targets = {'left': (DOOR_X, 0.0), 'right': (-DOOR_X, 0.0)}
        directions = {name: None if scores is None else score_direction((position, *scores), xy)
                      for name, xy in targets.items()}
        returns = [float(distance) if math.isfinite(distance) else None for distance in scan.ranges]
        report = dict(image_stamp_s=stamp, pose_stamp_s=pose_stamp, scan_stamp_s=scan_stamp,
                      position=position, quaternion=quaternion, intrinsics=intrinsics,
                      range_max_m=scan.range_max, range_min_m=scan.range_min,
                      scan_angle_min_rad=scan.angle_min, scan_step_rad=scan.angle_increment,
                      scan_ranges_m=returns, targets_xy=targets, scores=directions)
        with (args.output / 'sensors.json').open('x') as stream:
            json.dump(report, stream, indent=2, allow_nan=False)
            stream.write('\n')
        png = cv2.imencode('.png', cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR))
        if not png[0]:
            raise ValueError('Could not encode the RGB frame')
        with (args.output / 'rgb.png').open('xb') as stream:
            stream.write(png[1].tobytes())
        for subscriber in subscribers:
            subscriber.unregister()
        return
    raise TimeoutError('No synchronized image, pose, and scan at the stationary probe pose')


if __name__ == '__main__':
    main()
