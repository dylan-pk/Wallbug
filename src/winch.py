from std_msgs.msg import Float64
from utils import SystemState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

class WinchProfile():
    motor_output_power = None
    gearbox_output_speed = None
    gearbox_max_speed = 4
    gearbox_output_torque = None
    motor_torque_max_current = None
    relfected_inertia = None
    gearbox_output_power = None
    max_linear_acceleration = None
    nominal_angular_velocity = None
    max_angular_velocity = None
    max_linear_velocity = None
    gearbox_at_max_current = None
    max_angular_acceleration = None

class Winch:
    def __init__(self, winch_id, winch_profile, wallbot_node, position):
        self.id = winch_id
        self.profile = winch_profile
        self.position = position  # [x, y]
        self.wallbot_node = wallbot_node
        self.state = SystemState.IDLE

        self.trajectory = None

        # Publisher for traj.
        self.publisher = self.wallbot_node.create_publisher(JointTrajectory, f'winch_{winch_id}/trajectory', 10)

    def publish_trajectory(self, trajectory, total_time=None):
        """
        Publish a full rope length trajectory as a JointTrajectory message.
        """
        traj_msg = JointTrajectory()
        traj_msg.joint_names = [f'winch_{self.id}_rope']

        n_points = len(trajectory)
        if total_time is None:
            total_time = n_points * 0.01  # default to 0.01s per step

        dt = total_time / n_points

        for i, length in enumerate(trajectory):
            point = JointTrajectoryPoint()
            point.positions = [float(length)]
            point.time_from_start.sec = int(i * dt)
            point.time_from_start.nanosec = int((i * dt % 1) * 1e9)
            traj_msg.points.append(point)

        self.publisher.publish(traj_msg)
        self.wallbot_node.get_logger().info(f"Winch {self.id} publishing trajectory with {n_points} points")
