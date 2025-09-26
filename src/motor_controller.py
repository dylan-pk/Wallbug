
class MotorController():

    def __init__(self,
                 wallbot_specs,
                 winches
                 ):
        self.wallbot_specs = wallbot_specs
        self.winches = winches

    def compute_goal(self, current_pose, goal_pose):
        #rope lengths        
        current_rope_lengths = self.compute_rope_lengths(current_pose)
        target_rope_lengths = self.compute_rope_lengths(goal_pose)

        delta_rope_lengths = self.compute_rope_length_deltas(current_rope_lengths, target_rope_lengths)

    

    def compute_rope_lengths(self, pose):
        '''
        computes the rope lenghts needed for given pose
        '''
        w1, w2, w3, w4 = 1
        return w1, w2, w3, w4
    
    def compute_rope_length_deltas(self, current_rope_lengths, target_rope_lengths):
        pass

