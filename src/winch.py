from std_msgs.msg import Float64
from utils import SystemState

class WinchProfile():
    d = 1
    f = 4

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
