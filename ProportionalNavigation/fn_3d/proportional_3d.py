import numpy as np

from ProportionalNavigation.exceptions import InvalidProportionalGainError

def ZEM_3d(pursuer, target, N=3):
    if N <= 0: raise InvalidProportionalGainError(N)
    dR = target.pos - pursuer.pos
    dV = target.vel - pursuer.vel

    R = np.sqrt(dR.dot(dR))
    V = np.sqrt(dV.dot(dV))

    if V == 0: V = 1e-16  # if target is stationary and initial speed is 0

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



#ai made this code, i have no idea how this works but it does
def ai_true_3d(pursuer, target, N=3):
    if N <= 0: raise RuntimeError("Invalid N parameter")

    dR = target.pos - pursuer.pos
    dV = target.vel - pursuer.vel

    R_squared = dR.dot(dR)
    R = np.sqrt(R_squared)

    V = np.sqrt(dV.dot(dV))
    if V == 0: V = 1e-16  # if target is stationary and initial speed is 0
    t_go = R / V

    psi_c = np.arctan2(dR[1], dR[0])
    theta_c = np.arctan2(np.sqrt(dR[0] ** 2 + dR[1] ** 2), dR[2])

    LOS_rate_psi = (dR[0] * dV[1] - dR[1] * dV[0]) / R_squared
    LOS_rate_theta = (R * dV[2] - dR[2] * (dR[0] * dV[0] + dR[1] * dV[1]) / np.sqrt(
        dR[0] ** 2 + dR[1] ** 2) ** 2) / R_squared

    psi_cmd = psi_c - np.arctan2(N * LOS_rate_psi, V)
    theta_cmd = theta_c - np.arctan2(N * LOS_rate_theta, V)

    return {"psi":psi_cmd,
            "theta":theta_cmd,
            "R":R,
            "t_go":t_go
            }