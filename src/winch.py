from std_msgs.msg import Float64
from utils import SystemState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

class WinchProfile():
        #Motor specs
        motor_voltage = None
        motor_current = None
        motor_max_irms_current = None
        nominal_motor_speed = None
        motor_nominal_torque = None
        motor_rotor_inertia = None

        #Gearbox specs
        gear_ratio = None
        gearbox_efficiency = None
        gearbox_nominal_output_torque = None
        gearbox_max_input_speed = None

        #Wallbot / environment specs
        pulley_radius = None
        wallbot_weight = None

        #Profile outputs
        motor_output_power = None
        gearbox_output_speed = None
        gearbox_max_speed = None
        gearbox_output_torque = None
        motor_torque_max_current = None
        reflected_inertia = None
        gearbox_output_power = None
        nominal_angular_velocity = None
        nominal_linear_velocity = None
        max_angular_velocity = None
        max_linear_velocity = None
        gearbox_at_max_current = None
        max_angular_acceleration = None


class Winch:
    def __init__(self, winch_id, winch_profile, wallbot_node, position):
        self.id = winch_id
        self.profile = winch_profile
        self.position = position  # [x, y, z]
        self.wallbot_node = wallbot_node
        self.state = SystemState.IDLE

        self.trajectory = None
        self.move_time = None

        # Publisher for trajectory
        self.publisher = self.wallbot_node.create_publisher(
            JointTrajectory, f'winch_{winch_id}/trajectory', 10
        )

    def set_trajectory(self, trajectory, move_time):
        self.trajectory = trajectory
        self.move_time = move_time

    def publish_trajectory(self):
        if not self.trajectory:
            print(f"[WARN] Winch {self.id} has no trajectory to publish.")
            return

        msg = JointTrajectory()
        msg.joint_names = [f"winch_{self.id}_joint"]

        for i, point in enumerate(self.trajectory):
            if isinstance(point, JointTrajectoryPoint):
                msg.points.append(point)
            else:
                # Convert float to JointTrajectoryPoint if needed
                jp = JointTrajectoryPoint()
                jp.positions = [float(point)]
                jp.time_from_start.sec = int(i)
                jp.time_from_start.nanosec = 0
                msg.points.append(jp)

        self.publisher.publish(msg)
        print(f"[INFO] Published trajectory for Winch {self.id} with {len(msg.points)} points.")

