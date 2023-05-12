import numpy as np

from .Vehicle3D import Vehicle3d
from ProportionalNavigation.exceptions import InvalidProportionalGainError, OutOfBoundsRangeError

def ZEM_3d(pursuer, target, N=3):
    if N <= 0: raise InvalidProportionalGainError(N)
    dR = target.pos - pursuer.pos
    dV = target.vel - pursuer.vel

    R = np.sqrt(dR.dot(dR))
    V = np.sqrt(dV.dot(dV))

    t_go = R / V # time to go

    ZEM_i = dR + dV * t_go  # ZEM at t
    LOS_u = dR / R  # LOS direction unit vector
    ZEM_n = ZEM_i - ZEM_i.dot(LOS_u) * LOS_u  # ZEM normal to LOS

    nL = N * ZEM_n/(t_go * t_go)

    return {
        "nL": nL,
        "R": R,
        "V":V,
        "t_go":t_go,
        "ZEM_i":ZEM_i,
    }
