import numpy as np
import interpolation
from constants import *


def rotation_z(pos, angle):
    M = np.dot(np.array([[np.cos(angle), -np.sin(angle), 0], [np.sin(angle), np.cos(angle), 0], [0, 0, 1]]), np.array(pos))
    return M


def rotation_y(pos, angle):
    M = np.dot(np.array([[np.cos(angle), 0, np.sin(angle)], [0, 1, 0],[-np.sin(angle), 0, np.cos(angle)]]), np.array(pos))
    return M


def rotation_x(pos, angle):
    M = np.dot(np.array([[1, 0, 0], [0, np.cos(angle), -np.sin(angle)], [0, np.sin(angle), np.cos( angle)]]), np.array(pos))
    return M


def rotation_3D(pos, angles):
    return rotation_z(rotation_y(rotation_x(pos, angles[0]), angles[1]), angles[2])


def computeDKDetailed(a, b, c, use_rads=USE_RADS_INPUT, use_mm=USE_MM_OUTPUT, verbose=False):
    points = []
    theta = [a * THETA1_MOTOR_SIGN, (b - theta2Correction) * THETA2_MOTOR_SIGN, (-c - theta3Correction) * THETA3_MOTOR_SIGN]

    if not use_rads:
        for i in range(3):
            theta[i] *= np.pi / 180

    O = np.array([[0], [0], [0]])
    points.append([O[i][0] for i in range(3)])

    A = np.array([[constL1 * np.cos(theta[0])], [constL1 * np.sin(theta[0])], [0]]) + O
    points.append([A[i][0] * (100 if use_mm else 1) for i in range(3)])

    B = rotation_z([[constL2 * np.cos(-theta[1])], [0], [constL2 * np.sin(-theta[1])]], theta[0]) + A
    points.append([B[i][0] * (100 if use_mm else 1) for i in range(3)])

    M = rotation_z(rotation_y([[constL3 * np.cos(theta[2])], [0], [constL3 * np.sin(theta[2])]], theta[1]), theta[0]) + B
    points.append([M[i][0] * (100 if use_mm else 1) for i in range(3)])

    if verbose:
        print("computeDk(", a, ",", b, ",", c, ") -->", points[-1])
    return points


def computeDK(a, b, c, use_rads=USE_RADS_INPUT, use_mm=USE_MM_OUTPUT, verbose=False):
    return computeDKDetailed(a, b, c, use_rads=use_rads, use_mm=use_mm, verbose=verbose)[-1]


def computeIK(x, y, z, use_rads=USE_RADS_OUTPUT, use_mm=USE_MM_INPUT, verbose=False):
    theta = [0, 0, 0]
    if use_mm:
        x /= 100
        y /= 100
        z /= 100

    def alKashi(a, b, c):
        angle = (a**2 + b**2 - c**2) / (2 * a * b)
        if angle > 1:
            angle = 1
        elif angle < -1:
            angle = -1
        return math.acos(angle)

    theta[0] = (math.atan2(y, x)) * THETA1_MOTOR_SIGN
    AH = math.sqrt(x**2 + y**2) - constL1
    AM = math.sqrt(AH**2 + z**2)

    if AM == 0:
        theta[1] = 0 # Peu importe
    else:
        theta[1] = (-alKashi(AM, constL2, constL3) - math.atan2(z, AH) + theta2Correction) * THETA2_MOTOR_SIGN

    theta[2] = (-alKashi(constL2, constL3, AM) + np.pi + theta3Correction) * THETA3_MOTOR_SIGN

    if not use_rads:
        for i in range(3):
            theta[i] *= 180 / np.pi
    if verbose:
        print("computeIk(", x * (100 if use_mm else 1), ", ", y * (100 if use_mm else 1), ",", z * (100 if use_mm else 1), ") -->", theta)
        
    return theta

def computeIKOriented(x, y, z, leg_id, params, use_rads=USE_RADS_OUTPUT, use_mm=USE_MM_INPUT, verbose=False):
    pos = rotation_z([x, y, z], -LEG_ANGLES[leg_id])
    for i in range(3):
        pos[i] += params.initLeg[i]
    res = computeIK(pos[0], pos[1], pos[2], use_rads=use_rads, use_mm=use_mm, verbose=verbose)
    return res

class LinearStep:
    def __init__(self, x, y, z, height, duree):
        self.duree = duree
        self.height = height
        self.spline = interpolation.LinearSpline3D()
        self.spline.add_entry(0, 0, 0, -height)
        self.spline.add_entry(duree / 4, -x, -y, -height)
        self.spline.add_entry(2 * duree / 4, x, y, z - height)
        self.spline.add_entry(3 * duree / 4, x, y, -height)
        self.spline.add_entry(duree, 0, 0, -height)


    def step(self, t, phase):
        if self.duree == 0:
            return 0, 0, -self.height
        return self.spline.interpolate((t + self.duree / 2 * phase) % self.duree)

class RotationStep:
    def __init__(self, center, angle, duree):
        self.duree = duree
        self.spline = interpolation.CircularSpline3D([0, 0, 0])
        self.spline.add_entry(0, center, [0, 0, 1], -angle, duree / 4)
        # self.spline.add_entry(duree / 4, center, [0, 0, 1], -2 * angle, duree / 2)
        center2 = [self.spline.get_end(0)[0], center[1], center[2]]
        self.spline.add_entry(duree / 4, center2, [(1 if angle > 0 else -1), 0, 0], -np.pi, duree / 2)
        self.spline.add_entry(3 * duree / 4, center, [0, 0, 1], -angle, duree / 4)

    def step(self, t, phase):
        if self.duree == 0:
            return 0, 0, 0
        return self.spline.interpolate((t + self.duree / 2 * phase) % self.duree)

if __name__ == "__main__":
    computeDK(0, -90, -90, verbose=True, use_rads=False, use_mm=True)
    computeDK(0, 0, 0, verbose=True, use_rads=False, use_mm=True)
    computeDK(90, 0, 0, verbose=True, use_rads=False, use_mm=True)
    computeDK(180, -30.501, -67.819, verbose=True, use_rads=False, use_mm=True)
    computeDK(0, -30.645, 38.501, verbose=True, use_rads=False, use_mm=True)
    computeIK(200, 30, -60, verbose=True, use_rads=False, use_mm=True)
    computeIK(150, -50, -90, verbose=True, use_rads=False, use_mm=True)


