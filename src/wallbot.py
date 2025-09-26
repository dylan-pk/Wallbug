from motor_controller import MotorController

class Wallbot():
    def __init__(self,
            wallbot_specs,
            winches,
            starting_pose
            ):
        
        self.motor_controller = MotorController(wallbot_specs, winches)

    def set_goal(self, goal):
        '''
        goal to move wallbot to
        '''
        self.motor_controller.compute_goal(goal)
        pass