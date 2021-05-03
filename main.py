# import pypot.dynamixel
import time
import math

import kinematics
from utils import *
import time

# from kinematics import *
import math
import sys
from signal import signal, SIGINT
import traceback
import os

# import display


def main():
    # ports = pypot.dynamixel.get_available_ports()
    # dxl_io = pypot.dynamixel.DxlIO(ports[0], baudrate=1000000)
    #
    # robot = SimpleRobot(dxl_io)
    # robot.init()
    # time.sleep(0.1)
    # robot.enable_torque()
    # Defining the shutdown function here so it has visibility over the robot variable
    # def shutdown(signal_received, frame):
    #     # Handle any cleanup here
    #     print("SIGINT or CTRL-C detected. Setting motors to compliant and exiting")
    #     robot.disable_torque()
    #     print("Done ticking. Exiting.")
    #     sys.exit()
    #     # Brutal exit
    #     # os._exit(1)
    #
    # # Tell Python to run the shutdown() function when SIGINT is recieved
    # signal(SIGINT, shutdown)
    try:

        params = Parameters(
            freq=50,
            speed=1,
            initz=-0.06,
            travelDistancePerStep=0.08,
            lateralDistance=0.18,
            frontDistance=0.087,
            frontStart=0.032,
            method="minJerk",
        )

        print("Setting initial position")
        # for k, v in robot.legs.items():
        for k in range(6):
            print("Goal:", params.initLeg[k])
            angle = kinematics.computeIK(params.initLeg[k][0], params.initLeg[k][1] + 0.05, params.initLeg[k][2])
            print("Angles:", angle)
            # for i in range(3):
            #     v[i].goal_position = angle[i]

        # TODO create this function instead:
        # setPositionToRobot(0, 0, 0, robot, params)
        # robot.smooth_tick_read_and_write(3, verbose=False)
        print("Init position reached")
        time.sleep(2)
        print("Closing")
    except Exception as e:
        track = traceback.format_exc()
        print(track)
    # finally:
        # shutdown(None, None)


print("A new day dawns")
main()
print("Done !")
