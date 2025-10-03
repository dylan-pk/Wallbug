#!/usr/bin/env python3

import os
import yaml
import rclpy
from geometry_msgs.msg import PoseStamped
from wallbot import Wallbot
import threading
import time

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
CONFIG_DIRECTORY = os.path.join(SCRIPT_DIR, '..', 'config')

MOTOR_FILE = "Schneider_BCH2HF0733CAS5C.yml"
GEARBOX_FILE = "Apex_No:PA_2_090-S2.yml"
PATH_PLAN_FILE = "path_plan.yml"
WALLBOT_FILE = "wallbot_config.yml"

WALLBOT_STARTING_POSE = [2.0, 2.0]


def load_yaml(file_path):
    with open(file_path, 'r') as f:
        return yaml.safe_load(f)


def run_demo_once(publisher, path_goals, node):
    """Publish each goal in path_goals to the wallbot/goal topic."""
    for goal in path_goals:
        #if wallbot pose == goal pose send next goal else do nothing - check to maker sure first goal has been sent 
        try:
            x = float(goal[0])
            y = float(goal[1])
            z = float(goal[2]) if len(goal) > 2 else 0.0
        except (ValueError, TypeError):
            node.get_logger().error(f"Invalid goal: {goal} - skipping")
            continue

        msg = PoseStamped()
        msg.header.frame_id = "map"
        msg.header.stamp = node.get_clock().now().to_msg()
        msg.pose.position.x = x
        msg.pose.position.y = y
        msg.pose.position.z = z
        msg.pose.orientation.w = 1.0

        node.get_logger().info(f"Publishing goal: [{x}, {y}, {z}]")
        publisher.publish(msg)
        time.sleep(1.0)

    return True


def wallbot_spin(wallbot_node):
    """Run rclpy.spin in a separate thread for Wallbot subscriber callbacks."""
    while rclpy.ok():
        rclpy.spin_once(wallbot_node, timeout_sec=0.1)


def main():
    rclpy.init()

    print("Starting Wallbot Demo...")

    motor_params = load_yaml(os.path.join(CONFIG_DIRECTORY, MOTOR_FILE))
    gearbox_params = load_yaml(os.path.join(CONFIG_DIRECTORY, GEARBOX_FILE))
    wallbot_specs = load_yaml(os.path.join(CONFIG_DIRECTORY, WALLBOT_FILE))

    path_plan = load_yaml(os.path.join(CONFIG_DIRECTORY, PATH_PLAN_FILE))
    if 'path' not in path_plan:
        raise ValueError("path_plan.yml must contain a top-level 'path' key")
    path_goals = path_plan['path']

    wallbot = Wallbot(wallbot_specs, gearbox_params, motor_params, WALLBOT_STARTING_POSE)

    # Create publisher
    publisher = wallbot.create_publisher(PoseStamped, 'wallbot/goal', 10)

    wallbot_thread = threading.Thread(target=wallbot_spin, args=(wallbot,), daemon=True)
    wallbot_thread.start()

    run_demo_once(publisher, path_goals, wallbot)

    # Shutdown
    wallbot.destroy_node()
    rclpy.shutdown()
    wallbot_thread.join()


if __name__ == '__main__':
    main()
