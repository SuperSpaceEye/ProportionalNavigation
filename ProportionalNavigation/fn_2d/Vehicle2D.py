import numpy as np


class Vehicle2d(object):
    pass


def rot_to_unit(psi):
    return np.array([
        np.cos(psi),
        np.sin(psi)
    ], dtype=float)


class HeadingVelocity2d(Vehicle2d):
    def __init__(self,psi,x,y,V):
        self._psi = psi # Heading angle relative to global ref frame (x = north, y = east) in DEGREES
        self.V = V
        self.pos = np.array([x, y], dtype=float)
        self.vel = V * rot_to_unit(psi)

    @property
    def psi(self):
        return self._psi
    @psi.setter
    def psi(self, value):
        self._psi = value
        self.vel = rot_to_unit(self._psi) * self.V


class GlobalVelocity2d(Vehicle2d):
    def __init__(self,x,y,xd,yd):
        self.x = x
        self.y = y
        self.xd = xd
        self.yd = yd

        self.pos = np.array([x, y], dtype=float)
        self.vel = np.array([xd, yd], dtype=float)
        self._psi = None
        self._psi = self.psi
        self.V = sum(self.vel)

    @property
    def psi(self):
        if self._psi == None:
            self._psi = np.arccos(self.vel[0]/np.sqrt(self.vel[0]*self.vel[0] + self.vel[1]*self.vel[1]))
        return self._psi
    @psi.setter
    def psi(self, value):
        self._psi = value
        self.vel = rot_to_unit(self._psi) * self.V