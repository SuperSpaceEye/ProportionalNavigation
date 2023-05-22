# Perfect instant control

import sys
sys.path.append("..")
sys.path.append("../ProportionalNavigation")
sys.path.append("../ProportionalNavigation/fn_3d")
import ProportionalNavigation.fn_3d as pn
import matplotlib.pyplot as plt
import numpy as np
import math

import matplotlib
matplotlib.use("GTK3Agg")

sqrt = math.sqrt
if __name__ == "__main__":
    g = 10

    pursuer = pn.HeadingVelocity3d(np.deg2rad(45), 0, np.array([0, 0, 0]), g * 3)
    target = pn.HeadingVelocity3d(np.deg2rad(0), 0, np.array([100, 100, 100]), g*0)
    dt = 1. / 20
    N = 5

    max_sim_time = 200

    t = 0

    log = {'pursuer': [], 'target': [], "pursuer_vel":[], "pursuer_acc":[]}
    while True:
        ret = pn.ZEM_3d(pursuer, target, N=N)
        nL = ret['nL']
        R = ret['R']

        t = t + dt
        if R <= 1 or t > max_sim_time:
            break

        pursuer.pos += pursuer.vel * dt
        pursuer.pos[1] -= g * dt

        target.pos += target.vel * dt

        pursuer.vel += nL * dt

        target.yaw = np.cos(t * 5)
        target.pitch = np.cos(t / 2.)

        log["pursuer"].append(list(pursuer.pos))
        log['target'].append(list(target.pos))
        log["pursuer_vel"].append(np.sqrt(pursuer.vel.dot(pursuer.vel)))
        log["pursuer_acc"].append(np.sqrt((nL*dt).dot(nL*dt)))

    distance = [sqrt((x1 - x2) ** 2 + (y1 - y2) ** 2 + (z1 - z2) ** 2) for (x1, y1, z1), (x2, y2, z2) in
                zip(log["pursuer"], log["target"])]

    print(min(distance))

    fig = plt.figure(figsize=(12, 12))
    ax = fig.add_subplot(projection="3d")

    px, py, pz = [], [], []
    for x, y, z in log["pursuer"]: px.append(x); py.append(y); pz.append(z)

    tx, ty, tz = [], [], []
    for x, y, z in log["target"]: tx.append(x); ty.append(y); tz.append(z)

    ax.scatter(px, pz, py, c=range(len(distance)), cmap=matplotlib.colormaps["Greens"])
    ax.scatter(tx, tz, ty, c=range(len(distance)), cmap=matplotlib.colormaps["Reds"])

    plt.show()

    plt.scatter(range(len(distance)), log["pursuer_vel"], c=log["pursuer_acc"])
    plt.colorbar()
    plt.show()
