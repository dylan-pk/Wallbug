#!/usr/bin/env python3

import os
import yaml
import rclpy
from geometry_msgs.msg import PoseStamped
from wallbot_node import Wallbot
from wallbot.srv import SetGoal
import threading
import time

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
CONFIG_DIRECTORY = os.path.join(SCRIPT_DIR, '..', 'config')

MOTOR_FILE = "Schneider_BCH2HF0733CAS5C.yml"
GEARBOX_FILE = "Apex_No:PA_2_090-S2.yml"
PATH_PLAN_FILE = "path_plan.yml"
WALLBOT_FILE = "wallbot_config.yml"

WALLBOT_STARTING_POSE = [0.0, 0.7, 1.5]


def load_yaml(file_path):
    with open(file_path, 'r') as f:
        return yaml.safe_load(f)


def run_demo_once(service_client, path_goals, node):
    """Send each goal in path_goals via service call to wallbot/set_goal.
       Keep retrying the same goal until accepted."""
    for goal in path_goals:
        try:
            x = float(goal[0])
            y = float(goal[1])
            z = float(goal[2]) if len(goal) > 2 else 0.0
        except (ValueError, TypeError):
            node.get_logger().error(f"Invalid goal: {goal} - skipping")
            continue

        msg = PoseStamped()
        msg.header.frame_id = "map"
        msg.pose.position.x = x
        msg.pose.position.y = y
        msg.pose.position.z = z
        msg.pose.orientation.w = 1.0

        request = SetGoal.Request()
        request.goal = msg

        accepted = False
        while not accepted and rclpy.ok():
            msg.header.stamp = node.get_clock().now().to_msg()  # update timestamp each try
            request.goal = msg
            node.get_logger().info(f"Sending goal via service: [{x}, {y}, {z}]")

            future = service_client.call_async(request)
            rclpy.spin_until_future_complete(node, future, timeout_sec=5.0)

            if future.result() is not None:
                if future.result().accepted:
                    node.get_logger().info("Goal accepted by Wallbot.")
                    accepted = True
                else:
                    node.get_logger().warn("Goal rejected by Wallbot. Retrying in 5s...")
                    time.sleep(5.0)
            else:
                node.get_logger().error("Service call failed or timed out. Retrying in 5s...")
                time.sleep(5.0)

    return True

def wallbot_spin(wallbot_node):
    """Run rclpy.spin_once in a separate thread for the Wallbot node."""
    while rclpy.ok():
        rclpy.spin_once(wallbot_node, timeout_sec=0.1)


def main():
    rclpy.init()

    print("Starting Wallbot Demo with Service...")

    motor_params = load_yaml(os.path.join(CONFIG_DIRECTORY, MOTOR_FILE))
    gearbox_params = load_yaml(os.path.join(CONFIG_DIRECTORY, GEARBOX_FILE))
    wallbot_specs = load_yaml(os.path.join(CONFIG_DIRECTORY, WALLBOT_FILE))

    path_plan = load_yaml(os.path.join(CONFIG_DIRECTORY, PATH_PLAN_FILE))
    if 'path' not in path_plan:
        raise ValueError("path_plan.yml must contain a top-level 'path' key")
    path_goals = path_plan['path']

    wallbot = Wallbot(wallbot_specs, gearbox_params, motor_params, WALLBOT_STARTING_POSE)

    # Start spinning the Wallbot node in the background
    wallbot_thread = threading.Thread(target=wallbot_spin, args=(wallbot,), daemon=True)
    wallbot_thread.start()

    # Wait until the service is available
    wallbot.get_logger().info("Waiting for 'wallbot/set_goal' service to become available...")
    client = wallbot.create_client(SetGoal, 'wallbot/set_goal')
    if not client.wait_for_service(timeout_sec=5.0):
        wallbot.get_logger().error("Service 'wallbot/set_goal' not available. Exiting.")
        rclpy.shutdown()
        return

    # Run the demo
    run_demo_once(client, path_goals, wallbot)

    # Shutdown
    wallbot.destroy_node()
    rclpy.shutdown()
    wallbot_thread.join()


if __name__ == '__main__':
    main()
