import ProportionalNavigation.fn_2d as pn
import matplotlib.pyplot as plt
import numpy as np
import math

import matplotlib
matplotlib.use("GTK3Agg")

sqrt = math.sqrt
if __name__ == "__main__":
    g = 10

    pursuer = pn.HeadingVelocity2d(np.deg2rad(45), 0, 0, g * 3)
    target = pn.HeadingVelocity2d(np.deg2rad(180), 100, 50, g)
    dt = 1. / 20
    N = 4

    max_sim_time = 200

    terminate = False
    t = 0

    log = {'pursuer': [], 'target': []}
    while True:
        ret = pn.pure_2d(pursuer, target, N)
        # ret = pn.ZEM_2d(pursuer, target, N)
        nL = ret['nL']
        R = ret['R']

        t = t + dt
        if R <= 1 or t > max_sim_time:
            break

        psipd = nL / pursuer.V

        pursuer.pos += pursuer.vel * dt
        target.pos  += target.vel * dt
        pursuer.pos[1] -= g * dt

        pursuer.psi = pursuer.psi + dt * psipd

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
