from std_msgs.msg import Float64
from utils import SystemState

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

        # Publisher for rope length
        self.publisher = self.wallbot_node.create_publisher(Float64, f'winch_{winch_id}/trajectory', 10)

    def publish_trajectory(self, length):
        msg = Float64()
        msg.data = float(length)
        self.publisher.publish(msg)
        self.wallbot_node.get_logger().info(f"Winch {self.id} publishing rope length: {length}")
