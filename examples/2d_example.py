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
    options = pn.PNOptions(return_R=True, return_Vc=True)
    dt = 1. / 20
    N = 3

    max_sim_time = 200

    terminate = False
    t = 0

    log = {'pursuer': {'x': [], 'y': []}, 'target': {'x': [], 'y': []}}
    while not terminate:
        ret = pn.pure_2d(pursuer, target, N=N, options=options)
        # ret = PN.ZEM_2d(pursuer, target, N=N, options=options)
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

        # print(np.rad2deg(psipd * dt))

        target.psi = np.cos(t * 5)

        log['pursuer']['x'].append(pursuer.x)
        log['pursuer']['y'].append(pursuer.y)
        log['target']['x'].append(target.x)
        log['target']['y'].append(target.y)
    print(target.xd, N, t)

    distance = [sqrt((x1 - x2)**2 + (y1 - y2)**2) for (x1, y1), (x2, y2) in
                zip(zip(log["pursuer"]["x"], log["pursuer"]["y"]), zip(log["target"]["x"], log["target"]["y"]))]

    # plt.plot(distance)
    print(min(distance))

    # plt.plot(log['pursuer']['x'], log['pursuer']['y'])
    plt.scatter(log['pursuer']['x'], log['pursuer']['y'], c=range(len(distance)))
    plt.scatter(log['target']['x'], log['target']['y'], c=range(len(distance)))
    plt.colorbar()
    plt.show()
