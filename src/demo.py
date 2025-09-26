import os
from winch import Winch
from wallbot import Wallbot

NUM_WINCHES = 4

#File locations
CONFIG_DIRECTORY = "../config/"
MOTOR_FILE = "Schneider_BCH2HF0733CAS5C.yml"
GEARBOX_FILE= "Apex_No:PA_2_090-S2.yml"
PATH_PLAN_FILE = "path_plan.yml"
WALLBOT_FILE = "wallbot_config.yml"

WALLBOT_STARTING_POSE = [2,2]

def run_demo_once(wallbot, path_goals):

    for goal in range(path_goals):
        wallbot.set_goal(goal)

    return True


if __name__ == '__main__':
    print("Run Demo")

    OK = True

    motor_specs = os.path.join(CONFIG_DIRECTORY, MOTOR_FILE)
    gearbox_specs = os.path.join(CONFIG_DIRECTORY, MOTOR_FILE)

    demo_path = os.path.join(CONFIG_DIRECTORY, PATH_PLAN_FILE)
    wallbot_specs = os.path.join(CONFIG_DIRECTORY, WALLBOT_FILE)

    winches = []
    for winch_indx in range(NUM_WINCHES):
        winch = Winch(winch_indx, motor_specs, gearbox_specs)
        winches.append(winch)

    wallbot = Wallbot(wallbot_specs, winches, WALLBOT_STARTING_POSE)

    while OK:
        OK = run_demo_once(wallbot, wallbot_specs)