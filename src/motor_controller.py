import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from winch import Winch, WinchProfile
from calcUtils import (
    motor_output_power,
    gearbox_output_speed,
    gearbox_max_speed,
    gearbox_output_torque,
    torque_at_max_current,
    torque_at_max_current_2,
    gearbox_output_power,
    reflected_inertia,
    angular_velocity,
    max_angular_velocity,
    linear_velocity,
    max_linear_velocity,
    max_linear_acceleration
)


class MotorController:
    def __init__(self, wallbot_specs, gearbox_specs, motor_specs, wallbot_node, num_winches=4):
        self.wallbot_specs = wallbot_specs
        self.gearbox_specs = gearbox_specs['gearbox']
        self.motor_specs = motor_specs['motor']
        self.wallbot_node = wallbot_node
        self.num_winches = num_winches
        self.winches = []
        self.winch_profile = self.generate_winch_profile()

        # Publisher for combined joint trajectory
        self.joint_traj_pub = wallbot_node.create_publisher(
            JointTrajectory, '/joint_trajectory_controller/joint_trajectory', 10
        )

        self.generate_winches()

    def generate_winch_profile(self):
        profile = WinchProfile()

        profile.motor_voltage = self.motor_specs['voltage']
        profile.motor_current = self.motor_specs['current']
        profile.motor_max_irms_current = self.motor_specs['max_current_irms']
        profile.nominal_motor_speed = self.motor_specs['nominal_speed']
        profile.motor_nominal_torque = self.motor_specs['nominal_torque']
        profile.motor_rotor_inertia = self.motor_specs['rotor_inertia']

        profile.gear_ratio = self.gearbox_specs['gear_ratio']
        profile.gearbox_efficiency = self.gearbox_specs['efficiency'] / 100.0
        profile.gearbox_nominal_output_torque = self.gearbox_specs['nominal_output_torque']
        profile.gearbox_max_input_speed = self.gearbox_specs['max_input_speed']

        profile.pulley_radius = self.wallbot_specs['environment']['pulley_radius']
        profile.wallbot_weight = self.wallbot_specs['wallbot']['weight']

        # --- Derived quantities ---
        profile.motor_output_power = motor_output_power(profile.motor_voltage, profile.motor_current)
        profile.gearbox_output_speed = gearbox_output_speed(profile.nominal_motor_speed, profile.gear_ratio)
        profile.gearbox_max_speed = gearbox_max_speed(profile.gearbox_max_input_speed, profile.gear_ratio)
        profile.gearbox_output_torque = gearbox_output_torque(
            profile.motor_nominal_torque, profile.gear_ratio, profile.gearbox_efficiency
        )

        torque_max_current = torque_at_max_current(
            profile.motor_nominal_torque, profile.motor_current, profile.motor_max_irms_current
        )
        profile.motor_torque_max_current = torque_max_current
        profile.gearbox_at_max_current = torque_at_max_current_2(
            torque_max_current, profile.gear_ratio, profile.gearbox_efficiency
        )

        profile.reflected_inertia = reflected_inertia(profile.motor_rotor_inertia, profile.gear_ratio)
        profile.gearbox_output_power = gearbox_output_power(profile.gearbox_at_max_current, profile.gearbox_max_speed)

        profile.nominal_angular_velocity = angular_velocity(profile.gearbox_output_speed)
        profile.nominal_linear_velocity = linear_velocity(profile.nominal_angular_velocity, profile.pulley_radius)
        profile.max_angular_velocity = max_angular_velocity(profile.gearbox_max_speed)
        profile.max_linear_velocity = max_linear_velocity(profile.max_angular_velocity, profile.pulley_radius)
        profile.max_angular_acceleration = max_linear_acceleration(
            profile.gearbox_output_torque,
            profile.pulley_radius,
            profile.wallbot_weight,
            profile.reflected_inertia,
        )

        return profile

    def generate_winches(self):
        default_positions = [
            (1.75, 0.7, 2.75),
            (-1.75, 0.7, 2.75),
            (1.75, 0.7, 0.25),
            (-1.75, 0.7, 0.25),
        ]

        for i in range(self.num_winches):
            profile = self.generate_winch_profile()
            winch = Winch(
                winch_id=i,
                winch_profile=profile,
                wallbot_node=self.wallbot_node,
                position=default_positions[i],
            )
            self.winches.append(winch)

    def compute_rope_lengths(self, pose):
        rope_lengths = []
        for winch in self.winches:
            wx, wy, wz = winch.position

            if hasattr(pose, 'pose'):
                x = pose.pose.position.x
                y = pose.pose.position.y
                z = pose.pose.position.z
            else:
                x, y, z = pose

            dx = x - wx
            dy = y - wy
            dz = z - wz
            rope_lengths.append(math.sqrt(dx**2 + dy**2 + dz**2))
        return rope_lengths

    def compute_trajectories(self, goal, current_pose, n_steps=50):
        current_rope_lengths = self.compute_rope_lengths(current_pose)
        target_rope_lengths = self.compute_rope_lengths(goal)

        delta_rope_lengths = [t - c for c, t in zip(current_rope_lengths, target_rope_lengths)]
        largest_delta = max(abs(d) for d in delta_rope_lengths)
        move_time = largest_delta / max(self.winch_profile.nominal_linear_velocity, 1e-6)

        self.wallbot_node.get_logger().info(
            f"Computed trajectories: move_time={move_time:.3f}s, largest_delta={largest_delta:.3f}m"
        )

        # Create and send trajectory for each winch
        for i, winch in enumerate(self.winches):
            trajectory_points = []
            for step in range(n_steps):
                pos = current_rope_lengths[i] + delta_rope_lengths[i] * (step / (n_steps - 1))
                point = JointTrajectoryPoint()
                point.positions = [pos]
                t = (step / (n_steps - 1)) * move_time
                point.time_from_start.sec = int(t)
                point.time_from_start.nanosec = int((t % 1) * 1e9)
                trajectory_points.append(point)

            winch.trajectory = trajectory_points  # store in winch
            winch.publish_trajectory()  # if winch has its own topic

        # Publish combined trajectory message
        self.publish_joint_trajectory()

    def publish_joint_trajectory(self):
        traj = JointTrajectory()
        traj.header.stamp = self.wallbot_node.get_clock().now().to_msg()
        traj.joint_names = [f'winch_{i}_joint' for i in range(len(self.winches))]

        # Assuming each winch has the same number of trajectory points
        n_points = len(self.winches[0].trajectory)
        for step in range(n_points):
            point = JointTrajectoryPoint()
            point.positions = [w.trajectory[step].positions[0] for w in self.winches]
            point.time_from_start = self.winches[0].trajectory[step].time_from_start
            traj.points.append(point)

        self.joint_traj_pub.publish(traj)
        self.wallbot_node.get_logger().info("Published combined joint trajectory to /joint_trajectory_controller/joint_trajectory")

    def stop_winch(self, winch_id):
        winch = self.winches[winch_id]
        winch.trajectory = None
        winch.publish_trajectory()
        self.wallbot_node.get_logger().warn(f"Stopped winch {winch_id}")
