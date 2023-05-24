import numpy as np

from ProportionalNavigation.exceptions import InvalidProportionalGainError

def ZEM_3d(pursuer, target, N=3):
    if N <= 0: raise InvalidProportionalGainError(N)
    dR = target.pos - pursuer.pos
    dV = target.vel - pursuer.vel

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
        "ZEM_n": ZEM_n,
    }

def true_3d(pursuer, target, N=3):
    # dR = target.pos - pursuer.pos
    # dV = target.vel - pursuer.vel
    #
    # LOS = np.array((
    #     (dR[0]*dV[2] - dR[2]*dV[0])/(dR.dot(dR)),
    #     (dR[0]*dV[1] - dR[1]*dV[0])/(dR.dot(dR)),
    # ))
    #
    # Vc = -(dR.dot(dV))/np.sqrt(dR.dot(dR))
    #
    # return N * Vc * LOS

    if N <= 0: raise RuntimeError("Invalid N parameter")

    dR = target.pos - pursuer.pos
    dV = target.vel - pursuer.vel

    R_squared = dR.dot(dR)
    R = np.sqrt(R_squared)

    V = np.sqrt(dV.dot(dV))
    t_go = R / V

    V_c = R / t_go
    psi_c = np.arctan2(dR[1], dR[0])
    theta_c = np.arctan2(np.sqrt(dR[0] ** 2 + dR[1] ** 2), dR[2])

    LOS_rate_psi = (dR[0] * dV[1] - dR[1] * dV[0]) / R_squared
    LOS_rate_theta = (R * dV[2] - dR[2] * (dR[0] * dV[0] + dR[1] * dV[1]) / np.sqrt(
        dR[0] ** 2 + dR[1] ** 2) ** 2) / R_squared

    psi_cmd = psi_c - np.arctan2(N * LOS_rate_psi, V_c)
    theta_cmd = theta_c - np.arctan2(N * LOS_rate_theta, V_c)

    return psi_cmd, theta_cmd