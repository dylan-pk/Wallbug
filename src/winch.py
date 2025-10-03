from std_msgs.msg import Float64

class Winch:
    def __init__(self, winch_id, motor_specs, gearbox_specs, parent_node, position):
        self.winch_id = winch_id
        self.motor_specs = motor_specs
        self.gearbox_specs = gearbox_specs
        self.position = position  # [x, y]
        self.parent_node = parent_node

        # Publisher for rope length
        self.publisher = self.parent_node.create_publisher(Float64, f'winch_{winch_id}/rope_length', 10)

    def get_position(self):
        return self.position

    def set_position(self, position):
        self.position = position

    def publish_rope_length(self, length):
        from std_msgs.msg import Float64
        msg = Float64()
        msg.data = float(length)
        self.publisher.publish(msg)
        self.parent_node.get_logger().info(f"Winch {self.winch_id} publishing rope length: {length}")
