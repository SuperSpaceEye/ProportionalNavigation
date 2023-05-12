import numpy as np

from ProportionalNavigation.exceptions import InvalidProportionalGainError, OutOfBoundsRangeError

def pure_2d(pursuer, target, N=3):
    if N <= 0: raise InvalidProportionalGainError(N)

    R1 = (target.x - pursuer.x) ** 2 + (target.y - pursuer.y) ** 2
    R = np.sqrt(R1)

    if R <= 0: raise OutOfBoundsRangeError(R)

    Rdot = ((target.x - pursuer.x) * (target.xd - pursuer.xd) + (
            target.y - pursuer.y) * (target.yd - pursuer.yd)) / R
    Vc = -Rdot

    lamd = ((target.x - pursuer.x) * (target.yd - pursuer.yd) - (
            target.y - pursuer.y) * (target.xd - pursuer.xd)) / R1

    nL = N * lamd * Vc

    return {
        "R":R,
        "Vc":Vc,
        "lamdad":lamd,
        "nL":nL
    }


def ZEM_2d(pursuer, target, N=3):
    if N <= 0: raise InvalidProportionalGainError(N)

    R1 = (target.x - pursuer.x) ** 2 + (target.y - pursuer.y) ** 2
    R = np.sqrt(R1)

    if R <= 0: raise OutOfBoundsRangeError(R)

    V = np.sqrt((target.xd - pursuer.xd) ** 2 + (target.yd - pursuer.yd) ** 2)

    t_go = R / V

    ZEM_x = target.x - pursuer.x + (target.xd - pursuer.xd) * t_go
    ZEM_y = target.y - pursuer.y + (target.yd - pursuer.yd) * t_go

    lamd = ((target.x - pursuer.x) * (target.yd - pursuer.yd) - (
            target.y - pursuer.y) * (target.xd - pursuer.xd)) / R1

    ZEM_plos = -ZEM_x * np.sin(lamd) + ZEM_y * np.cos(lamd)

    nL = (N * ZEM_plos) / (t_go*t_go)

    return {
        "R":R,
        "t_go":t_go,
        "nL":nL,
        "ZEM_plos":ZEM_plos,
    }