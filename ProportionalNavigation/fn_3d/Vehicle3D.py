import numpy as np


class Vehicle3d:
    pass


def rot_to_unit(psi, theta):
    return np.array([
            np.cos(psi) * np.cos(theta),
            np.sin(psi) * np.cos(theta),
            np.sin(theta)
        ], dtype=float)


class HeadingVelocity3d(Vehicle3d):
    # yaw, pitch
    def __init__(self, psi, theta, pos, V):
        self._psi = psi
        self._theta = theta
        self.pos = np.array(pos, dtype=float)
        self.vel = rot_to_unit(psi, theta) * V
        self._V = V

    @property
    def psi(self):
        return self._psi
    @psi.setter
    def psi(self, value):
        self._psi = value
        self.vel = rot_to_unit(value, self.theta) * self.V

    @property
    def theta(self):
        return self._theta
    @theta.setter
    def theta(self, value):
        self._theta = value
        self.vel = rot_to_unit(self.psi, value) * self.V

    @property
    def V(self):
        return self._V
    @V.setter
    def V(self, value):
        self._V = value
        self.vel = rot_to_unit(self.psi, self.theta) * value


class GlobalVelocity3d(Vehicle3d):
    # yaw, pitch
    def __init__(self, psi, theta, pos, V):
        self._psi = psi
        self._theta = theta
        self.pos = np.array(pos, dtype=float)
        self.vel = np.array(V, dtype=float)
        self._psi, self._theta = self.get_angles()
        self._V = V / rot_to_unit(self._psi, self._theta)

    def get_angles(self):
        theta = np.arcsin(self.vel[2])
        psi = np.arcsin(self.vel[1]/np.cos(theta))
        return psi, theta

    @property
    def psi(self):
        return self._psi

    @psi.setter
    def psi(self, value):
        self._psi = value
        self.vel = rot_to_unit(value, self.theta) * self.V

    @property
    def theta(self):
        return self._theta

    @theta.setter
    def theta(self, value):
        self._theta = value
        self.vel = rot_to_unit(self.psi, value) * self.V

    @property
    def V(self):
        return self._V

    @V.setter
    def V(self, value):
        self._V = value
        self.vel = rot_to_unit(self.psi, self.theta) * value