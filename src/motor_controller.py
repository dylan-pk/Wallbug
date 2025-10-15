import math
from winch import Winch, WinchProfile
from geometry_msgs.msg import PoseStamped

class MotorController():
    def __init__(self, wallbot_specs, gearbox_specs, motor_specs, num_winches = 4):
        self.wallbot_specs = wallbot_specs
        self.gearbox_specs = gearbox_specs
        self.motor_specs = motor_specs
        self.winches = []
        self.num_winches = num_winches

        self.generate_winches()

    def generate_winch_profile():
        pass
    
    def generate_winches(self):
        default_positions = [(0,0,0), (5,0,0), (5,5,0), (0,5,0)]  # Example positions
        for i in range(self.num_winches):
            winch = Winch(i, self.motor_specs, self.gearbox_specs, wallbot_node=self, position=default_positions[i])
            self.winches.append(winch)

    
    def compute_trajecotires(self, goal, current_pose,):
        current_rope_lengths = self.compute_rope_lengths(current_pose)
        target_rope_lengths = self.compute_rope_lengths(goal)
        delta_rope_lengths = self.compute_rope_length_deltas(current_rope_lengths, target_rope_lengths)

        # Publish new rope lengths
        for winch, length in zip(self.winches, target_rope_lengths):
            winch.publish_rope_length(length)

        return delta_rope_lengths

    def compute_rope_lengths(self, pose):
        rope_lengths = []
        for winch in self.winches:
            wx, wy = winch.get_position()
            dx = pose[0] - wx
            dy = pose[1] - wy
            length = math.sqrt(dx**2 + dy**2)
            rope_lengths.append(length)
        return rope_lengths

    def compute_rope_length_deltas(self, current_rope_lengths, target_rope_lengths):
        return [target - current for current, target in zip(current_rope_lengths, target_rope_lengths)]
    
    def send_trajecotries(self):
        '''
        trajecotires are only sent if all winches are provided with trajecotires 
        '''
        for winch in self.winches:
            if winch.read_trajecotry() == None:
                print(f"[ERROR] Send trajecotires FAILED. Winch {winch.id} does not have a valid trajectorie!")
        
        for winch in self.winches:
            winch.publish_trjectory()

    def stop_winch(self, winch_id):
        winch = self.winches[winch_id]
        winch.trajectory(None)
        winch.publish_trjectory()
        
