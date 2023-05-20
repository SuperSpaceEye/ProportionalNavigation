import numpy as np

from ProportionalNavigation.exceptions import InvalidProportionalGainError

def rotate(ar, a):
    x = ar[0]
    # z = ar[2]
    # ar[0] = np.cos(a) * x - np.sin(a) * z
    # ar[2] = np.cos(a) * z + np.sin(a) * x
    ar[0] = -x
    # ar[2] = -z
    return ar

def ZEM_3d(pursuer, target, N=3):
    if N <= 0: raise InvalidProportionalGainError(N)
    dR = target.pos - pursuer.pos
    dV = target.vel - pursuer.vel

    # rotated = False
    # a = 180
    # if dR[0] < 0:
    #     rotated = True
        # dR = rotate(dR, np.deg2rad(a))
        # dV = rotate(dV, np.deg2rad(a))

    R = np.sqrt(dR.dot(dR))
    V = np.sqrt(dV.dot(dV))

    if V == 0: V = 1e16  # if target is stationary and initial speed is 0

    t_go = R / V  # time to go

    ZEM_i = dR + dV * t_go  # ZEM at t
    LOS_u = dR / R  # LOS direction unit vector
    ZEM_n = ZEM_i - ZEM_i.dot(LOS_u) * LOS_u  # ZEM normal to LOS

    nL = N * ZEM_n/(t_go * t_go)

    # if rotated: nL = rotate(nL, -np.deg2rad(a))

    return {
        "nL": nL,
        "R": R,
        "V": V,
        "t_go": t_go,
        "ZEM_i": ZEM_i,
        "ZEM_n": ZEM_n,
        # "rotated":rotated
    }
