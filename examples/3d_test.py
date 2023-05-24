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

def rot_to_unit(psi, theta):
    return np.array([
        np.cos(theta) * np.cos(psi),
        np.cos(theta) * np.sin(psi),
        -np.sin(theta)
    ], dtype=float)

sqrt = math.sqrt
if __name__ == "__main__":
    g = 10

    pursuer = pn.HeadingVelocity3d(np.deg2rad(45), 0, np.array([0, 0, 0]), g * 3)
    target = pn.HeadingVelocity3d(np.deg2rad(0), 0, np.array([-100, -100, -100]), g*1)
    dt = 1. / 20
    N = 3

    max_sim_time = 20

    t = 0

    log = {'pursuer': [], 'target': [], "pursuer_vel":[], "pursuer_acc":[]}
    while True:
        ret = pn.ZEM_3d(pursuer, target, N=N)
        nL = ret['nL']
        R = ret['R']

        psi_cmd, theta_cmd = pn.ai_true_3d(pursuer, target, N)

        t = t + dt
        if R <= 5 or t > max_sim_time:
            break

        pursuer.pos += pursuer.vel * dt
        pursuer.pos[1] -= g * dt

        target.pos += target.vel * dt
        # =======================
        v_psi = pursuer.V * np.array([np.cos(psi_cmd), np.sin(psi_cmd), 0])
        v_theta = pursuer.V * np.array([np.sin(theta_cmd) * np.cos(psi_cmd), np.sin(theta_cmd) * np.sin(psi_cmd), np.cos(theta_cmd)])

        a_cmd = (v_psi + v_theta - pursuer.vel)

        pursuer.vel += a_cmd * dt

        #================================

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
