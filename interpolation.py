import numpy as np
import matplotlib.pyplot as plt


class LinearSpline:
    def __init__(self):
        self.ts = []
        self.xs = []

    def add_entry(self, t, x):
        self.ts.append(t)
        self.xs.append(x)

    def interpolate(self, t):
        if t <= self.ts[0]:
            return self.xs[0]
        elif t >= self.ts[-1]:
            return self.xs[-1]
        else:
            i2 = 0
            while t > self.ts[i2]:
                i2 += 1
            i1 = i2 - 1
            k = (t - self.ts[i1])/(self.ts[i2]-self.ts[i1])
            return self.xs[i1]*(1-k) + self.xs[i2]*k


class LinearSpline3D:
    def __init__(self):
        self.xs = LinearSpline()
        self.ys = LinearSpline()
        self.zs = LinearSpline()

    def add_entry(self, t, x, y ,z):
        self.xs.add_entry(t, x)
        self.ys.add_entry(t, y)
        self.zs.add_entry(t, z)

    def get_start(self):
        return self.xs.ts[0]

    def get_duree(self):
        return self.xs.ts[-1] - self.xs.ts[0]

    def interpolate(self, t):
        x = self.xs.interpolate(t)
        y = self.ys.interpolate(t)
        z = self.zs.interpolate(t)
        return x, y, z


class CircularSpline3D:
    def __init__(self, start):
        self.ts = []
        self.duree = []
        self.rayon = []
        self.normale = []
        self.center = []
        self.angle = []
        self.end = [start]

    def add_entry(self, t, center, normale, angle, duree):
        self.ts.append(t)
        self.duree.append(duree)
        rayon = np.array([self.end[-1][i] - center[i] for i in range(3)])
        self.rayon.append(rayon)
        self.center.append(center)
        self.normale.append(normale)
        self.angle.append(angle)
        self.end.append(rayon * np.cos(angle) + np.cross(normale, rayon) * np.sin(angle) + center)

    def get_middle(self, i):
        return (self.end[i - 1] + self.end[i]) / 2

    def get_end(self, i):
        return self.end[i + 1]

    def interpolate(self, t):
        if t <= self.ts[0]:
            print(t)
            return self.rayon[0] + self.center[0]
        elif t >= self.ts[-1] + self.duree[-1]:
            return self.end[-1]
        else:
            i = 0
            while t > self.ts[i] + self.duree[i]:
                i += 1
            angle = self.angle[i] * (t - self.ts[i]) / self.duree[i]
            M = self.rayon[i] * np.cos(angle) + np.cross(self.normale[i], self.rayon[i]) * np.sin(angle) + self.center[i]
            return M

if __name__ == "__main__":
    # from mpl_toolkits import mplot3d
    # ax = plt.axes(projection='3d')
    # ts = np.arange(0, 2*np.pi, 0.05)
    # xs = []
    # ys = []
    # zs = []
    # nx = []
    # ny = []
    # nz = []
    # for t in ts:
    #     R = np.array([3, 0, 0])
    #     N = np.array([0, 0, 1])
    #     O = np.array([2, 0, 0])
    #     nx.append(N[0] * t + O[0])
    #     ny.append(N[1] * t + O[1])
    #     nz.append(N[2] * t + O[2])
    #     M = R * math.cos(t) + np.cross(N, R) * math.sin(t) + O
    #     xs.append(M[0])
    #     ys.append(M[1])
    #     zs.append(M[2])




    from mpl_toolkits import mplot3d
    ax = plt.axes(projection='3d')
    ts = np.arange(0., 9, 0.05)
    xs = []
    ys = []
    zs = []
    N1 = []
    N2 = []

    # point = [3, 1, 0]
    # spline = CircularSpline3D(point)
    # center = [0, 0, 0]
    # normale1 = [0, 0, 1]
    # spline.add_entry(0, center, normale1, 3, np.pi / 2)
    # normale2 = spline.get_middle(0)
    # center2 = spline.get_middle(0)
    # spline.add_entry(3, center2, normale2, 3, np.pi)

    point = [3, 0, 0]
    spline = CircularSpline3D(point)
    angle = np.pi / 3
    center = [0, 0, 0]
    normale1 = [0, 0, 1]
    spline.add_entry(0, center, normale1, angle, 3)
    normale2 = [1, 0, 0]
    center2 = [spline.get_end(0)[0], center[1], center[2]]
    print(center2)
    # spline.add_entry(3, center, normale1, - np.pi, 3)
    spline.add_entry(3, center2, normale2, np.pi, 3)
    spline.add_entry(6, center, normale1, angle, 3)


    ax.scatter3D(center[0], center[1], center[2], c='r')
    ax.scatter3D(center2[0], center2[1], center2[2], c='g')
    ax.scatter3D(point[0], point[1], point[2], c='b')
    for t in ts:
        x, y, z = spline.interpolate(t)
        xs.append(x)
        ys.append(y)
        zs.append(z)
        N1.append(np.array(normale1) * t + np.array(center))
        N2.append(np.array(normale2) * t + np.array(center))


    ax.plot3D(xs, ys, zs, c = 'b')
    ax.plot3D([N1[i][0] for i in range(len(N1))], [N1[i][1] for i in range(len(N1))], [N1[i][2] for i in range(len(N1))], c='r')
    ax.plot3D([N2[i][0] for i in range(len(N2))], [N2[i][1] for i in range(len(N2))], [N2[i][2] for i in range(len(N2))], c='g')
    plt.show()
