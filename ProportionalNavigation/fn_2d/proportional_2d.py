import numpy as np

from ProportionalNavigation.exceptions import InvalidProportionalGainError


def pure_2d(pursuer, target, N=3):
    if N <= 0: raise InvalidProportionalGainError(N)

    dR = abs(target.pos - pursuer.pos)
    dV = abs(target.vel - pursuer.vel)

    R_squared = dR.dot(dR)
    R = np.sqrt(R_squared)

    # Vc = -(dR[0] * dV[0] + dR[1] * dV[1]) / R
    Vc = -dR.dot(dV.T) / R
    lamd = (dR[0] * dV[1] - dR[1] * dV[0]) / R_squared

    nL = N * lamd * Vc

    return {
        "R": R,
        "Vc": Vc,
        "lamdad": lamd,
        "nL": nL
    }


def ZEM_2d(pursuer, target, N=3):
    if N <= 0: raise InvalidProportionalGainError(N)

    dR = (target.pos - pursuer.pos)
    dV = (target.vel - pursuer.vel)

    R_squared = dR.dot(dR)
    R = np.sqrt(R_squared)

    V = np.sqrt(dV.dot(dV))
    t_go = R / V

    lamd = (dR[0] * dV[1] - dR[1] * dV[0]) / R_squared

    ZEM = dR + dV * t_go
    ZEM_plos = -ZEM[0] * np.sin(lamd) + ZEM[1] * np.cos(lamd)

    nL = (N * ZEM_plos) / (t_go*t_go)

    return {
        "R":R,
        "t_go":t_go,
        "nL":nL,
        "ZEM":ZEM,
        "ZEM_plos":ZEM_plos,
    }