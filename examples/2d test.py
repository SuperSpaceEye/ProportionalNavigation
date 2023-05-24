import numpy as np
import matplotlib.pyplot as plt
import math
import matplotlib
matplotlib.use("GTK3Agg")

def rot_to_unit(psi):
    return np.array([
        np.cos(psi),
        np.sin(psi)
    ], dtype=float)


class HeadingVelocity2d():
    def __init__(self,psi,x,y,V):
        self._psi = psi # Heading angle relative to global ref frame (x = north, y = east) in DEGREES
        self.V = V
        self.pos = np.array([x, y], dtype=float)
        self.vel = V * rot_to_unit(psi)

    @property
    def psi(self):
        return self._psi
    @psi.setter
    def psi(self, value):
        self._psi = value
        self.vel = rot_to_unit(self._psi) * self.V

def ZEM_2d(pursuer, target, N=3):
    if N <= 0: raise RuntimeError("Invalid N parameter")

    dR = target.pos - pursuer.pos
    dV = target.vel - pursuer.vel

    R_squared = dR.dot(dR)
    R = np.sqrt(R_squared)

    V = np.sqrt(dV.dot(dV))
    t_go = R / V

    # lamd = (dR[0] * dV[1] - dR[1] * dV[0]) / R_squared
    #
    # ZEM = dR + dV * t_go
    # ZEM_plos = -ZEM[0] * np.sin(lamd) + ZEM[1] * np.cos(lamd)
    #
    # nL = (N * ZEM_plos) / (t_go*t_go)

    V_c = R / t_go
    psi_c = np.arctan2(dR[1], dR[0])
    LOS_rate = (dR[0] * dV[1] - dR[1] * dV[0]) / R_squared
    psi_cmd = psi_c - np.arctan2(N * LOS_rate, V_c)


    return {
        "R":R,
        "t_go":t_go,
        "nL":psi_cmd,
        # "ZEM":ZEM,
        # "ZEM_plos":ZEM_plos,
    }


sqrt = math.sqrt
if __name__ == "__main__":
    g = 10

    pursuer = HeadingVelocity2d(np.deg2rad(45), 0, 0, g * 3)
    target = HeadingVelocity2d(np.deg2rad(180), -100, -50, -g)
    dt = 1. / 20
    N = 4

    max_sim_time = 20

    terminate = False
    t = 0

    log = {'pursuer': [], 'target': []}
    while True:
        ret = ZEM_2d(pursuer, target, N)
        nL = ret['nL']
        R = ret['R']

        t = t + dt
        if R <= 1 or t > max_sim_time:
            break

        psipd = nL / pursuer.V

        pursuer.pos += pursuer.vel * dt
        target.pos  += target.vel * dt
        pursuer.pos[1] -= g * dt

        # ====================

        # pursuer.psi += dt * psipd
        pursuer.psi = nL

        # =======================

        target.psi = np.cos(t * 5)

        log['pursuer'].append(tuple(pursuer.pos))
        log['target'].append(tuple(target.pos))

    distance = [sqrt((x1 - x2)**2 + (y1 - y2)**2)for (x1, y1), (x2, y2) in zip(log["pursuer"], log["target"])]
    print(min(distance))

    px, py = [], []
    for x, y in log["pursuer"]: px.append(x); py.append(y)

    tx, ty = [], []
    for x, y in log["target"]: tx.append(x); ty.append(y)

    plt.scatter(px, py, c=range(len(distance)))
    plt.scatter(tx, ty, c=range(len(distance)))
    plt.colorbar()
    plt.show()
