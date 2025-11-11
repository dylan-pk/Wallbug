from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from motor_controller import MotorController
from utils import SystemState
from wallbot.srv import SetGoal

NUM_WINCHES = 4

class Wallbot(Node):
    def __init__(self, wallbot_specs, gearbox_specs, motor_specs, starting_pose):
        super().__init__('wallbot')
        self.get_logger().info('Initialising Wallbot with provided YAML configs')

        self.wallbot_specs = wallbot_specs
        self.pose = starting_pose
        self.state = SystemState.IDLE

        # Motor controller
        self.motor_controller = MotorController(self.wallbot_specs, gearbox_specs, motor_specs, self)

        # Create service
        self.goal_service = self.create_service(SetGoal, 'wallbot/set_goal', self.handle_set_goal)
        self.get_logger().info('Goal service ready at wallbot/set_goal')

    def handle_set_goal(self, request: SetGoal.Request, response: SetGoal.Response):
        goal = request.goal

        
        
        if self.state != SystemState.IDLE:
            self.get_logger().warn('Wallbot is busy, goal rejected.')
            response.accepted = False
            return response

        self.get_logger().info(f"Received service goal: {[goal.pose.position.x, goal.pose.position.y, goal.pose.position.z]}")
        self.motor_controller.compute_trajectories(goal, self.pose)
        # self.motor_controller.send_trajectories()
        self.state = SystemState.RUNNING
        response.accepted = True
        return response

    def overwrite_and_send_goal(self, goal: PoseStamped):
        '''
        Ignores if robot is busy and sends new goal
        '''
        self.motor_controller.compute_trajectories(goal, self.pose)
        # self.motor_controller.send_trajectories()
        self.state = SystemState.RUNNING

    def all_stop(self):
        self.state = SystemState.STOPPED
        for winch in range(NUM_WINCHES):
            self.motor_controller.stop_winch(winch)
