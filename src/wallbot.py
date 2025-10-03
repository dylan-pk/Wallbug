import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from motor_controller import MotorController
from winch import Winch

NUM_WINCHES = 4

class Wallbot(Node):
    def __init__(self, wallbot_specs, gearbox_specs, motor_specs, starting_pose):
        super().__init__('wallbot')
        self.get_logger().info('Initialising Wallbot with provided YAML configs')

        self.wallbot_specs = wallbot_specs
        self.gearbox_specs = gearbox_specs
        self.motor_specs = motor_specs
        self.pose = starting_pose

        # Create Winches
        self.winches = []
        default_positions = [(0,0,0), (5,0,0), (5,5,0), (0,5,0)]  # Example positions
        for i in range(NUM_WINCHES):
            winch = Winch(i, motor_specs, gearbox_specs, parent_node=self, position=default_positions[i])
            self.winches.append(winch)

        # Motor controller
        self.motor_controller = MotorController(self.wallbot_specs, self.winches)

        # Subscribe to goals
        self.goal_subscriber = self.create_subscription(
            PoseStamped,
            'wallbot/goal',
            self.goal_callback,
            10
        )

    def goal_callback(self, msg: PoseStamped):
        goal = [msg.pose.position.x, msg.pose.position.y, msg.pose.position.z]
        self.get_logger().info(f'Received goal from topic: {goal}')
        delta_lengths = self.motor_controller.compute_goal(self.pose, goal)
        self.pose = goal  # Update current pose
        self.get_logger().info(f'Rope deltas: {delta_lengths}')

