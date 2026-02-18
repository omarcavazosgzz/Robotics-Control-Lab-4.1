#!/usr/bin/env python3
import argparse
import csv
import math
from typing import List, Tuple

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint


def float_to_time_from_start(t: float) -> Tuple[int, int]:
    """seconds(float) -> (sec, nanosec) non-negative"""
    if t < 0:
        t = 0.0
    sec = int(math.floor(t))
    nanosec = int(round((t - sec) * 1e9))
    if nanosec >= 1_000_000_000:
        sec += 1
        nanosec -= 1_000_000_000
    return sec, nanosec


def load_csv_points(
    csv_path: str,
    speed: float,
    min_dt: float,
    max_points: int,
) -> Tuple[List[float], List[List[float]], List[float]]:
    """
    Returns:
      t_list, arm_pos_list (Nx5), gripper_pos_list (N)
    """
    rows = []
    with open(csv_path, "r", newline="") as f:
        reader = csv.DictReader(f)
        required = [
            "t",
            "shoulder_pan_cmd",
            "shoulder_lift_cmd",
            "elbow_flex_cmd",
            "wrist_flex_cmd",
            "wrist_roll_cmd",
            "gripper_cmd",
        ]
        for k in required:
            if k not in reader.fieldnames:
                raise RuntimeError(f"Missing column '{k}' in CSV. Found: {reader.fieldnames}")

        for r in reader:
            rows.append(r)

    if not rows:
        raise RuntimeError("CSV has no data rows.")

    # normalize time so it starts at 0
    t0 = float(rows[0]["t"])
    speed = max(speed, 1e-6)

    t_list: List[float] = []
    arm_list: List[List[float]] = []
    grip_list: List[float] = []

    last_t = None
    for r in rows:
        t = (float(r["t"]) - t0) / speed

        # downsample by min_dt
        if last_t is not None and (t - last_t) < min_dt:
            continue

        arm = [
            float(r["shoulder_pan_cmd"]),
            float(r["shoulder_lift_cmd"]),
            float(r["elbow_flex_cmd"]),
            float(r["wrist_flex_cmd"]),
            float(r["wrist_roll_cmd"]),
        ]
        gr = float(r["gripper_cmd"])

        t_list.append(t)
        arm_list.append(arm)
        grip_list.append(gr)
        last_t = t

        if max_points > 0 and len(t_list) >= max_points:
            break

    # Ensure strictly increasing times (controller expects sane timing)
    fixed_t = [t_list[0]]
    for i in range(1, len(t_list)):
        if t_list[i] <= fixed_t[-1]:
            fixed_t.append(fixed_t[-1] + max(min_dt, 1e-3))
        else:
            fixed_t.append(t_list[i])

    return fixed_t, arm_list, grip_list


class CsvReplayer(Node):
    def __init__(self, arm_action: str, gripper_action: str):
        super().__init__("csv_replayer")
        self.arm_client = ActionClient(self, FollowJointTrajectory, arm_action)
        self.gripper_client = ActionClient(self, FollowJointTrajectory, gripper_action)

    def send_goal(
        self,
        client: ActionClient,
        joint_names: List[str],
        times: List[float],
        positions: List[List[float]],
    ):
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = joint_names

        points: List[JointTrajectoryPoint] = []
        for t, pos in zip(times, positions):
            p = JointTrajectoryPoint()
            p.positions = pos
            sec, nsec = float_to_time_from_start(t)
            p.time_from_start.sec = sec
            p.time_from_start.nanosec = nsec
            points.append(p)

        goal.trajectory.points = points
        return client.send_goal_async(goal)

    def wait_for_servers(self, timeout_sec: float = 10.0):
        if not self.arm_client.wait_for_server(timeout_sec=timeout_sec):
            raise RuntimeError("Arm action server not available.")
        if not self.gripper_client.wait_for_server(timeout_sec=timeout_sec):
            raise RuntimeError("Gripper action server not available.")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--csv", required=True, help="Path to PID.csv (or P/PI/PD).")
    parser.add_argument("--speed", type=float, default=1.0, help=">1 faster, <1 slower.")
    parser.add_argument("--min-dt", type=float, default=0.02, help="Downsample: keep points at least this far apart (sec).")
    parser.add_argument("--max-points", type=int, default=2000, help="Cap number of points (0 = no cap).")
    parser.add_argument("--arm-action", default="/arm_controller/follow_joint_trajectory")
    parser.add_argument("--gripper-action", default="/gripper_controller/follow_joint_trajectory")
    parser.add_argument("--arm-joints", default="1,2,3,4,5", help="Comma list, default matches your setup.")
    parser.add_argument("--gripper-joints", default="6", help="Comma list, default matches your setup.")
    args = parser.parse_args()

    arm_joints = [j.strip() for j in args.arm_joints.split(",") if j.strip()]
    grip_joints = [j.strip() for j in args.gripper_joints.split(",") if j.strip()]

    times, arm_pos, grip_pos = load_csv_points(args.csv, args.speed, args.min_dt, args.max_points)

    # Convert grip to list-of-list so it fits the same send_goal signature
    grip_pos_ll = [[g] for g in grip_pos]

    rclpy.init()
    node = CsvReplayer(args.arm_action, args.gripper_action)

    try:
        node.get_logger().info("Waiting for action servers...")
        node.wait_for_servers(timeout_sec=15.0)

        node.get_logger().info(f"Sending ARM trajectory: {len(times)} points to {args.arm_action} joints={arm_joints}")
        arm_future = node.send_goal(node.arm_client, arm_joints, times, arm_pos)

        node.get_logger().info(f"Sending GRIPPER trajectory: {len(times)} points to {args.gripper_action} joints={grip_joints}")
        grip_future = node.send_goal(node.gripper_client, grip_joints, times, grip_pos_ll)

        # Spin until both goals accepted and finished
        rclpy.spin_until_future_complete(node, arm_future)
        rclpy.spin_until_future_complete(node, grip_future)

        arm_goal_handle = arm_future.result()
        grip_goal_handle = grip_future.result()

        if not arm_goal_handle.accepted:
            raise RuntimeError("Arm goal was rejected.")
        if not grip_goal_handle.accepted:
            raise RuntimeError("Gripper goal was rejected.")

        node.get_logger().info("Goals accepted. Waiting for results...")

        arm_result_future = arm_goal_handle.get_result_async()
        grip_result_future = grip_goal_handle.get_result_async()

        rclpy.spin_until_future_complete(node, arm_result_future)
        rclpy.spin_until_future_complete(node, grip_result_future)

        node.get_logger().info(f"ARM result: {arm_result_future.result().result.error_code} ({arm_result_future.result().result.error_string})")
        node.get_logger().info(f"GRIP result: {grip_result_future.result().result.error_code} ({grip_result_future.result().result.error_string})")

        node.get_logger().info("Done.")

    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
