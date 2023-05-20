import numpy as np

from ProportionalNavigation.exceptions import InvalidProportionalGainError


def ZEM_3d(pursuer, target, N=3):
    if N <= 0: raise InvalidProportionalGainError(N)
    dR = target.pos - pursuer.pos
    dV = target.vel - pursuer.vel

    mask = dR >= 0
    mask = np.array(list(map(lambda x: -1 if x else 1, mask)))
    dR = abs(dR)
    # dV = abs(dV)

    # dR[0], dR[2] = abs(dR[0]), abs(dR[2])
    dV = dV * mask
    # dV[0], dV[2] = abs(dV[0]), abs(dV[2])

    R = np.sqrt(dR.dot(dR))
    V = np.sqrt(dV.dot(dV))

    if V == 0: V = 1e16  # if target is stationary and initial speed is 0

    t_go = R / V  # time to go

    ZEM_i = dR + dV * t_go  # ZEM at t
    LOS_u = dR / R  # LOS direction unit vector
    ZEM_n = ZEM_i - ZEM_i.dot(LOS_u) * LOS_u  # ZEM normal to LOS

    nL = N * ZEM_n/(t_go * t_go)

    return {
        "nL": nL,
        "R": R,
        "V": V,
        "t_go": t_go,
        "ZEM_i": ZEM_i,
        "ZEM_n": ZEM_n
    }
