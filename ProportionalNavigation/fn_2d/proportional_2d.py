import numpy as np

from .Vehicle2D import Vehicle2d
from ProportionalNavigation.exceptions import InvalidProportionalGainError, OutOfBoundsRangeError


class PNOptions:
    def __init__(self, return_R=False, return_Vc=False, return_lambdad=False):
        self.return_R = return_R
        self.return_Vc = return_Vc
        self.return_lambdad = return_lambdad


def __test_inputs(pursuer, target, N=3, options=None):
    if not isinstance(target, Vehicle2d):
        raise TypeError(f"Pursuer is instance of {type(pursuer)} and not Vehicle2d")
    if not isinstance(pursuer, Vehicle2d):
        raise TypeError(f"Target is instance of {type(pursuer)} and not Vehicle2d")

    if not isinstance(N, (float, int)):
        raise TypeError(f"N is type {type(N)} and not float or int")
    if N <= 0:
        raise InvalidProportionalGainError(N)

    if not isinstance(options, PNOptions) and options is not None:
        raise TypeError(f"options is type {type(options)} and not PNOptions")


def pure_2d(pursuer, target, N=3, options=None):
    __test_inputs(pursuer, target, N, options)

    R1 = (target.x - pursuer.x) ** 2 + (target.y - pursuer.y) ** 2
    R = np.sqrt(R1)
    if R <= 0:
        raise OutOfBoundsRangeError(R)
    Rdot = ((target.x - pursuer.x) * (target.xd - pursuer.xd) + (
            target.y - pursuer.y) * (target.yd - pursuer.yd)) / R
    Vc = -Rdot

    lamd = ((target.x - pursuer.x) * (target.yd - pursuer.yd) - (
            target.y - pursuer.y) * (target.xd - pursuer.xd)) / R1

    nL = N * lamd * Vc

    if options is None:
        return nL

    ret = {}
    if options.return_R:
        ret["R"] = R
    if options.return_Vc:
        ret["Vc"] = Vc
    if options.return_lambdad:
        ret["lambdad"] = lamd
    ret["nL"] = nL
    return ret


def ZEM_2d(pursuer, target, N=3, options=None):
    __test_inputs(pursuer, target, N, options)

    R1 = (target.x - pursuer.x) ** 2 + (target.y - pursuer.y) ** 2
    R = np.sqrt(R1)
    V = np.sqrt((target.xd - pursuer.xd) ** 2 + (target.yd - pursuer.yd) ** 2)

    t_go = R / V

    ZEM_x = target.x - pursuer.x + (target.xd - pursuer.xd) * t_go
    ZEM_y = target.y - pursuer.y + (target.yd - pursuer.yd) * t_go

    lamd = ((target.x - pursuer.x) * (target.yd - pursuer.yd) - (
            target.y - pursuer.y) * (target.xd - pursuer.xd)) / R1

    ZEM_plos = -ZEM_x * np.sin(lamd) + ZEM_y * np.cos(lamd)

    nL = (N * ZEM_plos) / (t_go*t_go)

    ret = {}
    if options.return_R:
        ret["R"] = R
    ret["nL"] = nL
    return ret