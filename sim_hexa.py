#!/usr/bin/env python
import time
import argparse
import pybullet as p
from onshape_to_robot.simulation import Simulation
import numpy as np

import kinematics
from constants import *

class Parameters:
    def __init__(
        self
    ):
        # Initial leg positions in the coordinates of each leg.
        self.initLeg = [0.2, 0, -0.06]  # INIT_LEG_POSITIONS

        # Motor re-united by joint name for the simulation
        self.legs = {}
        self.legs[0] = ["j_c1_rf", "j_thigh_rf", "j_tibia_rf"]
        self.legs[1] = ["j_c1_rm", "j_thigh_rm", "j_tibia_rm"]
        self.legs[2] = ["j_c1_rr", "j_thigh_rr", "j_tibia_rr"]
        self.legs[3] = ["j_c1_lr", "j_thigh_lr", "j_tibia_lr"]
        self.legs[4] = ["j_c1_lm", "j_thigh_lm", "j_tibia_lm"]
        self.legs[5] = ["j_c1_lf", "j_thigh_lf", "j_tibia_lf"]


def to_pybullet_quaternion(roll, pitch, yaw):
    return p.getQuaternionFromEuler([roll, pitch, yaw])


# Updates the values of the dictionnary targets to set 3 angles to a given leg
def set_leg_angles(alphas, leg_id, targets, params):
    leg = params.legs[leg_id]
    i = 0
    for name in leg:
        targets[name] = alphas[i]
        i += 1

def absolute_position(leg, pt, robotPos):
    rot = kinematics.rotation_z(pt, LEG_ANGLES[leg])
    pos = kinematics.rotation_3D([rot[j] + LEG_CENTER_POS[leg][j] for j in range(3)], robotPos[1])
    return [pos[j] + robotPos[0][j] for j in range(3)]

def move_cross_attached(cross, leg, pt, robotPos):
    crossPos = absolute_position(leg, pt, robotPos)
    p.resetBasePositionAndOrientation(cross, crossPos, to_pybullet_quaternion(robotPos[1][0], robotPos[1][1], robotPos[1][2]))

# m_friction
parser = argparse.ArgumentParser()
parser.add_argument("--mode", "-m", type=str, default="direct", help="test")
args = parser.parse_args()
controls = {}
robotPath = "phantomx_description/urdf/phantomx.urdf"
sim = Simulation(robotPath, gui=True, panels=True, useUrdfInertia=False)
# sim.setFloorFrictions(lateral=0, spinning=0, rolling=0)
pos, rpy = sim.getRobotPose()
sim.setRobotPose([0, 0, 0.5], [0, 0, 0, 1])

params = Parameters()


if args.mode == "frozen-direct":
    crosses = []
    for i in range(4):
        crosses.append(p.loadURDF("target2/robot.urdf"))
    for name in sim.getJoints():
        # print(name)
        if ("c1" in name or "thigh" in name or "tibia" in name) and "rf" in name:
            controls[name] = p.addUserDebugParameter(name, -math.pi, math.pi, 0)
elif args.mode == "direct":
    crosses = []
    for i in range(6):
        crosses.append(p.loadURDF("target2/robot.urdf"))
    for name in sim.getJoints():
        # print(name)
        if "c1" in name or "thigh" in name or "tibia" in name:
            controls[name] = p.addUserDebugParameter(name, -math.pi, math.pi, 0)
elif args.mode == "inverse":
    cross = p.loadURDF("target2/robot.urdf")
    # Use your own DK function
    controls["target_x"] = p.addUserDebugParameter("target_x", -0.4, 0.4, 0)
    controls["target_y"] = p.addUserDebugParameter("target_y", -0.4, 0.4, 0)
    controls["target_z"] = p.addUserDebugParameter("target_z", -0.4, 0.4, 0)
elif args.mode == "robot-ik":
    last_points = None
elif args.mode == "walk":
    controls["speed_x"] = p.addUserDebugParameter("speed_x", -1, 1, 0)
    controls["speed_y"] = p.addUserDebugParameter("speed_y", -1, 1, 0)
    controls["speed_rot"] = p.addUserDebugParameter("speed_rot", -1, 1, 0)
    controls["center_height"] = p.addUserDebugParameter("center_height", 0, 0.175, 0.06)
    controls["step_height"] = p.addUserDebugParameter("step_height", 0, 0.4, 0.06)
    controls["step_max_amplitude"] = p.addUserDebugParameter("step_max_amplitude", 0, 0.1, 0.05)
elif args.mode == "control":
    last_points = None
    from pynput import keyboard
    keys = {
        'z': 0,
        's': 0,
        'q': 0,
        'd': 0,
        'a': 0,
        'e': 0,
        'r': 0,
        'm': 0,
        't': 0,
        'p': 0,
        keyboard.Key.space: 0,
    }


    def on_press(key):
        if type(key) == keyboard.Key:
            keys[key] = 1
        else:
            keys[key.char] = 1

    def on_release(key):
        if type(key) == keyboard.Key:
            keys[key] = 0
        else:
            keys[key.char] = 0

    listener = keyboard.Listener(
        on_press=on_press,
        on_release=on_release)
    listener.start()


def walk(speed_x, speed_y, speed_rot, height, max_ampl, z):
    speed_abs = abs(speed_x) + abs(speed_y) + abs(speed_rot)
    duree = (1 / speed_abs if speed_abs != 0 else 0)

    x = 0
    y = 0
    rot = 0
    if speed_abs > max_ampl:
        diff = speed_abs - max_ampl
        x = speed_x - diff * speed_x / speed_abs
        y = speed_y - diff * speed_y / speed_abs
        rot = speed_rot - diff * speed_rot / speed_abs

    t = time.time()

    Linear = kinematics.LinearStep(x, y, z, height, duree)
    Rotation = kinematics.RotationStep([-params.initLeg[0], -params.initLeg[1], 0], rot * 2 * math.pi, duree)

    for leg_id in range(6):
        pos = Linear.step(t, leg_id % 2)
        pos2 = kinematics.rotation_z(Rotation.step(t, leg_id % 2), LEG_ANGLES[leg_id])
        alphas = kinematics.computeIKOriented(
            pos[0] + pos2[0],
            pos[1] + pos2[1],
            pos[2] + pos2[2],
            leg_id,
            params,
        )
        set_leg_angles(alphas, leg_id, targets, params)
        state = sim.setJoints(targets)

def robotik():
    last_points = None
    for leg_id in range(6):
        alphas = kinematics.computeIKOriented(
                0.01 * math.sin(2 * math.pi * 0.5 * time.time() * 2),
                0.02 * math.cos(2 * math.pi * 0.5 * time.time() * 2),
                0.03 * math.sin(2 * math.pi * 0.2 * time.time() * 2),
                leg_id,
                params,
            )
        set_leg_angles(alphas, leg_id, targets, params)

    leg = params.legs[0]
    point = kinematics.computeDK(targets[leg[0]], targets[leg[1]], targets[leg[2]])

    if last_points is None:
        last_points = time.time(), absolute_position(0, point, sim.getRobotPose())
    elif time.time() - last_points[0] > 0.1:
        tip = absolute_position(0, point, sim.getRobotPose())
        p.addUserDebugLine(last_points[1], tip, [1, 0, 0], 2., 10.)
        last_points = time.time(), tip

    # p.addUserDebugLine([0, 0, 0], getLegTip(), [1, 0, 0], 2., 10.)

    #sim.setRobotPose([0, 0, 0.5], [0, 0, 0, 1])
    state = sim.setJoints(targets)

while True:
    targets = {}
    for name in sim.getJoints():
        if "c1" in name or "thigh" in name or "tibia" in name:
            targets[name] = 0
    if args.mode == "frozen-direct":
        for name in controls.keys():
            targets[name] = p.readUserDebugParameter(controls[name])
        # Use your own DK function, the result should be: https://www.youtube.com/watch?v=w3psAbh3AoM
        points = kinematics.computeDKDetailed(targets["j_c1_rf"], targets["j_thigh_rf"], targets["j_tibia_rf"])
        pos = sim.getRobotPose()
        i = 0
        for pt in points:
            move_cross_attached(crosses[i], 0, pt, pos)
            i += 1

        sim.setRobotPose([0, 0, 0.5], [0, 0, 0, 1])
        state = sim.setJoints(targets)

    elif args.mode == "direct":
        for name in controls.keys():
            targets[name] = p.readUserDebugParameter(controls[name])

        points = []
        for leg in params.legs.values():
            points.append(kinematics.computeDK(targets[leg[0]], targets[leg[1]], targets[leg[2]]))

        pos = sim.getRobotPose()
        i = 0
        for pt in points:
            move_cross_attached(crosses[i], i, pt, pos)
            i += 1

        #sim.setRobotPose([0, 0, 0.5], [0, 0, 0, 1])
        state = sim.setJoints(targets)

    elif args.mode == "inverse":
        adds = kinematics.computeDK(LEG_CENTER_POS[0][0], LEG_CENTER_POS[0][1], LEG_CENTER_POS[0][2])
        x = p.readUserDebugParameter(controls["target_x"]) + adds[0]
        y = p.readUserDebugParameter(controls["target_y"]) + adds[1]
        z = p.readUserDebugParameter(controls["target_z"]) + adds[2]
        # Use your own IK function
        alphas = kinematics.computeIK(x, y, z)

        targets["j_c1_rf"] = alphas[0]
        targets["j_thigh_rf"] = alphas[1]
        targets["j_tibia_rf"] = alphas[2]

        move_cross_attached(cross, 0, [x, y, z], sim.getRobotPose())

        sim.setRobotPose([0, 0, 0.5], [0, 0, 0, 1])
        state = sim.setJoints(targets)

    elif args.mode == "robot-ik":
        # Use your own IK function
        for leg_id in range(6):
            alphas = kinematics.computeIKOriented(
                0.01 * math.sin(2 * math.pi * 0.5 * time.time()),
                0.02 * math.cos(2 * math.pi * 0.5 * time.time()),
                0.03 * math.sin(2 * math.pi * 0.2 * time.time()),
                leg_id,
                params,
            )
            set_leg_angles(alphas, leg_id, targets, params)

        leg = params.legs[0]
        point = kinematics.computeDK(targets[leg[0]], targets[leg[1]], targets[leg[2]])

        if last_points is None:
            last_points = time.time(), absolute_position(0, point, sim.getRobotPose())
        elif time.time() - last_points[0] > 0.1:
            tip = absolute_position(0, point, sim.getRobotPose())
            p.addUserDebugLine(last_points[1], tip, [1, 0, 0], 2., 10.)
            last_points = time.time(), tip

        # p.addUserDebugLine([0, 0, 0], getLegTip(), [1, 0, 0], 2., 10.)

        sim.setRobotPose([0, 0, 0.5], [0, 0, 0, 1])
        state = sim.setJoints(targets)

    elif args.mode == "walk":
        speed_x = p.readUserDebugParameter(controls["speed_x"])
        speed_y = p.readUserDebugParameter(controls["speed_y"])
        speed_rot = p.readUserDebugParameter(controls["speed_rot"])
        height = p.readUserDebugParameter(controls["center_height"]) - 0.06
        max_ampl = p.readUserDebugParameter(controls["step_max_amplitude"])
        z = p.readUserDebugParameter(controls["step_height"])

        walk(speed_x, speed_y, speed_rot, height, max_ampl, z)

        # sim.setRobotPose([0, 0, 0.5], [0, 0, 0, 1])

    elif args.mode == "control":
        # print(keys)
        speed_x = (keys['z'] - keys['s']) * (1 / 2 if keys['q'] - keys['d'] != 0 else 1) * (1 + keys['m'] * 2)
        speed_y = (keys['q'] - keys['d']) * (1 / 2 if speed_x != 0 else 1) * (1 + keys['m'] * 2)
        speed_rot = (keys['a'] - keys['e']) / 2
        height = 0.04 + keys[keyboard.Key.space] / 8
        max_ampl = 0.06 + 0.04 * keys['m']
        z = 0.06 + 0.5 * keys['p']

        walk(speed_x, speed_y, speed_rot, height, max_ampl, z)

        angles = sim.getRobotPose()
        if keys['r']:
            sim.setRobotPose(angles[0], to_pybullet_quaternion(0, 0, angles[1][2]))

        if keys['t']:
            robotik()

        p.resetDebugVisualizerCamera(
            1.2, angles[1][2] * 180 / np.pi - 90, -20, angles[0])
    sim.tick()

