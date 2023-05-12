import ProportionalNavigation.fn_2d as pn
import matplotlib.pyplot as plt
import numpy as np
import math

import matplotlib
matplotlib.use("GTK3Agg")

sqrt = math.sqrt
if __name__ == "__main__":
    g = 10

    pursuer = pn.HeadingVelocity2d(45, 0, 0, g * 3)
    target = pn.HeadingVelocity2d(180, 100, 50, g)
    dt = 1. / 20
    N = 3

    max_sim_time = 200

    terminate = False
    t = 0

    log = {'pursuer': [], 'target': []}
    while not terminate:
        ret = pn.pure_2d(pursuer, target, N)
        # ret = PN.ZEM_2d(pursuer, target, N)
        nL = ret['nL']
        R = ret['R']

        t = t + dt
        if R <= 1 or t > max_sim_time:
            terminate = True

        psipd = nL / pursuer.V

        pursuer.x += pursuer.xd * dt
        pursuer.y += pursuer.yd * dt - g * dt
        target.x += target.xd * dt
        target.y += target.yd * dt

        pursuer.psi = pursuer.psi + np.rad2deg(dt * psipd)

        target.psi = np.cos(t * 5)

        log['pursuer'].append((pursuer.x, pursuer.y))
        log['target'].append((target.x, target.y))

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
