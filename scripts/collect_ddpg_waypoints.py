"""Drive the production local tracker toward fixed targets in Gazebo."""

import argparse
import csv
import json
import math
import os
from pathlib import Path
import sys
import tempfile


def reset(agent, change_pose):
    from gazebo_msgs.msg import ModelState
    from geometry_msgs.msg import Twist
    import rospy

    agent.pub.publish(Twist())
    rospy.sleep(0.30)
    state = ModelState(model_name=agent.robot_name, reference_frame="world")
    state.pose.position.z = 0.01
    state.pose.orientation.w = 1.0
    if not change_pose(state).success:
        raise ValueError("Gazebo rejected the reset pose")
    agent.prev_linear = agent.prev_angular = 0.0
    agent.near_collision_active = False
    reset_stamp = rospy.get_time()
    rospy.sleep(0.65)
    x, y, yaw = agent.get_pose()
    scan = agent.laser_ig.get_msg(timeout=2.0)
    if (math.hypot(x, y) > 0.04 or abs(yaw) > 0.04 or scan is None
            or scan.header.stamp.to_sec() <= reset_stamp):
        raise ValueError("The robot or scan was not ready at the reset pose")


def trace_waypoint(agent, model, case, horizon, tolerance, dwell):
    from geometry_msgs.msg import Twist
    import rospy

    angle = math.radians(case["bearing_deg"])
    target = (case["distance_m"] * math.cos(angle), case["distance_m"] * math.sin(angle))
    agent.target_x, agent.target_y = target
    agent.local_tracking_mode = case["mode"]
    inside_since = None
    started = rospy.get_time()
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        now = rospy.get_time()
        elapsed = now - started
        x, y, yaw = agent.get_pose()
        distance = math.dist((x, y), target)
        bearing = (math.atan2(target[1] - y, target[0] - x) - yaw + math.pi) % (2 * math.pi) - math.pi
        if distance <= tolerance:
            if inside_since is None:
                inside_since = elapsed
        else:
            inside_since = None
        reached = inside_since is not None and elapsed - inside_since >= dwell
        finished = reached or elapsed >= horizon
        if finished:
            agent.pub.publish(Twist())
        else:
            agent.step(model)
        yield dict(elapsed_s=elapsed, x_m=x, y_m=y, yaw_rad=yaw, target_distance_m=distance,
                   target_bearing_rad=bearing, min_laser_m=agent.last_min_range,
                   linear_mps=0.0 if finished else agent.prev_linear,
                   angular_rps=0.0 if finished else agent.prev_angular,
                   tracker="end" if finished else agent.last_tracking_source,
                   threshold_active=int(agent.near_collision_active), reached=int(reached),
                   terminal=int(finished))
        if finished:
            return
        rate.sleep()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("plan", type=Path)
    parser.add_argument("output", type=Path)
    parser.add_argument("ros_package", type=Path)
    args = parser.parse_args()
    sys.path.insert(0, str(args.ros_package / "scripts/voronoi_exploration"))

    from gazebo_msgs.srv import SetModelState
    from geometry_msgs.msg import Twist
    import rospy
    from multi_robot_explorer import K, RobotAgent, WEIGHTS_PATH, build_actor_model, tf

    from run_ddpg_waypoint_benchmark import audit_case

    plan = json.loads(args.plan.read_text())
    rospy.init_node("ddpg_waypoint_benchmark", anonymous=True)
    session = tf.Session()
    K.set_session(session)
    model = build_actor_model()
    model.load_weights(WEIGHTS_PATH)
    agent = RobotAgent("robot1", experiment_seed=plan["gazebo_seed"], safety_shield_enabled=True)
    rospy.wait_for_service("/gazebo/set_model_state", timeout=25)
    change_pose = rospy.ServiceProxy("/gazebo/set_model_state", SetModelState)
    if agent.laser_ig.get_msg(timeout=15) is None or agent.model_states_ig.get_msg(timeout=15) is None:
        raise ValueError("Missing robot or laser before the benchmark")
    try:
        for case in plan["runs"]:
            output = args.output / f"{case['name']}_{case['mode']}.csv"
            if output.exists():
                audit_case(plan, case, output)
                continue
            reset(agent, change_pose)
            rows = trace_waypoint(agent, model, case, plan["horizon_s"],
                                  plan["tolerance_m"], plan["dwell_s"])
            descriptor, attempt = tempfile.mkstemp(prefix=output.stem + ".", suffix=".attempt.csv",
                                                    dir=output.parent)
            with os.fdopen(descriptor, "w", newline="") as stream:
                first = next(rows)
                writer = csv.DictWriter(stream, fieldnames=list(first), lineterminator="\n")
                writer.writeheader()
                writer.writerow(first)
                writer.writerows(rows)
            result = audit_case(plan, case, Path(attempt))
            Path(attempt).rename(output)
            print(f"VERIFIED {plan['layout']} {case['name']} {case['mode']} "
                  f"reached={result['reached']} final={result['final_distance_m']:.3f}m", flush=True)
    finally:
        agent.pub.publish(Twist())
        session.close()


if __name__ == "__main__":
    main()
