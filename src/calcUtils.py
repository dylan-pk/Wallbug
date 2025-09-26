import numpy as np


def motor_output_power(motor_voltage, motor_current):
    return motor_voltage * motor_current

def gearbox_output_speed(nominal_motor_speed, gear_ratio):
    '''
    nominal_motor_speed - RPM
    gear_ratio - X : 1
    '''
    return nominal_motor_speed / gear_ratio

def gearbox_max_speed(max_gearbox_input_speed, gear_ratio):
    '''
    max_gearbox_input_speed - RPM
    gear_ratio - X : 1
    '''
    return max_gearbox_input_speed / gear_ratio

def gearbox_output_torque(motor_nominal_torque, gear_ratio, gearbox_effficiency):
    return (motor_nominal_torque * gear_ratio) * gearbox_effficiency

def torque_at_max_current(motor_nominal_torque, motor_current, motor_max_irms_current):
    return (motor_nominal_torque / motor_current) * motor_max_irms_current

def torque_at_max_current_2(torque_at_max_current, gear_ratio, gearbox_effficiency):
    return torque_at_max_current * gear_ratio * gearbox_effficiency

def gearbox_output_power(torque_at_max_current, gearbox_max_speed):
    return torque_at_max_current * 2 * np.pi * (gearbox_max_speed/60)

def reflected_inertia(motor_rotor_inertia, gear_ratio):
    return motor_rotor_inertia * np.power(gear_ratio,2)

##Redundancy
def angular_velocity(gearbox_output_speed):
    return 2 * np.pi * (gearbox_output_speed/60)

def max_angular_velocity(gearbox_max_speed):
    return 2 * np.pi * (gearbox_max_speed/60)


def linear_velocity(angular_velocity, pulley_radius):
    return angular_velocity * pulley_radius

def max_linear_velocity(max_angular_velocity, pulley_radius):
    return max_angular_velocity * pulley_radius

def max_linear_acceleration(gearbox_nominal_output_torque, pulley_radius, wallbot_weight, reflected_inertia):
    return (gearbox_nominal_output_torque/pulley_radius) / (wallbot_weight+ (reflected_inertia/100))