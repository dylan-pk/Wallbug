import math
from winch import Winch, WinchProfile
from geometry_msgs.msg import PoseStamped
import calcUtils
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

class MotorController():
    def __init__(self, wallbot_specs, gearbox_specs, motor_specs, num_winches = 4):
        self.wallbot_specs = wallbot_specs
        self.gearbox_specs = gearbox_specs
        self.motor_specs = motor_specs
        self.winches = []
        self.num_winches = num_winches
        self.winch_profile = self.generate_winch_profile(gearbox_specs, motor_specs)

        self.generate_winches()

    def generate_winch_profile(self):
        profile = WinchProfile
        profile.motor_output_power = None
        profile.gearbox_output_speed = None
        profile.gearbox_max_speed = 4
        profile.gearbox_output_torque = None
        profile.motor_torque_max_current = None
        profile.relfected_inertia = None
        profile.gearbox_output_power = None
        profile.nominal_linear_velocity = None
        profile.nominal_angular_velocity = None
        profile.max_angular_velocity = None
        profile.max_linear_velocity = None
        profile.gearbox_at_max_current = None
        profile.max_angular_acceleration = None
        return profile
    
    def generate_winches(self):
        default_positions = [(0,0,0), (5,0,0), (5,5,0), (0,5,0)]  # Example positions
        for i in range(self.num_winches):
            winch = Winch(i, 
                          self.motor_specs, 
                          self.gearbox_specs, 
                          wallbot_node=self, 
                          position=default_positions[i]
                          )
            self.winches.append(winch)

    
    def compute_trajectories(self, goal, current_pose, n_steps=50):
        current_rope_lengths = self.compute_rope_lengths(current_pose)
        target_rope_lengths = self.compute_rope_lengths(goal)

        # Compute deltas for each winch
        delta_rope_lengths = [t - c for c, t in zip(current_rope_lengths, target_rope_lengths)]
        largest_delta = max(abs(d) for d in delta_rope_lengths)

        # Compute total move time based on largest delta
        move_time = largest_delta / self.winch_profile.nominal_linear_velocity

        # Publish new rope lengths
        for i, winch in enumerate(self.winches):
            # Linear interpolation of rope lengths
            trajectory = [
                current_rope_lengths[i] + delta_rope_lengths[i] * (step / (n_steps - 1))
                for step in range(n_steps)
            ]
            # Publish the trajectory
            winch.publish_trajectory(trajectory, total_time=move_time)

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
        trajectories are only sent if all winches are provided with trajectories 
        '''
        for winch in self.winches:
            if winch.read_trajecotry() == None:
                print(f"[ERROR] Send trajectories FAILED. Winch {winch.id} does not have a valid trajectorie!")
        
        for winch in self.winches:
            winch.publish_trjectory()

    def stop_winch(self, winch_id):
        winch = self.winches[winch_id]
        winch.trajectory(None)
        winch.publish_trjectory()
        
