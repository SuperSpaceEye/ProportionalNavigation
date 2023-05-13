import numpy as np


def rot_to_unit(yaw, pitch):
    return np.array([
            np.cos(yaw) * np.cos(pitch),
            np.sin(yaw) * np.cos(pitch),
            np.sin(pitch)
        ], dtype=float)

class HeadingVelocity3d:
    # yaw, pitch
    def __init__(self, yaw, pitch, pos, V):
        self._yaw = yaw
        self._pitch = pitch
        self.pos = np.array(pos, dtype=float)
        self.vel = rot_to_unit(yaw, pitch) * V
        self._V = V

    @property
    def yaw(self):
        return self._yaw
    @yaw.setter
    def yaw(self, value):
        self._yaw = value
        self.vel = rot_to_unit(value, self.pitch) * self.V

    @property
    def pitch(self):
        return self._pitch
    @pitch.setter
    def pitch(self, value):
        self._pitch = value
        self.vel = rot_to_unit(self.yaw, value) * self.V

    @property
    def V(self):
        return self._V
    @V.setter
    def V(self, value):
        self._V = value
        self.vel = rot_to_unit(self.yaw, self.pitch) * value


class GlobalVelocity3d:
    # yaw, pitch
    def __init__(self, pos, vel):
        self.pos = np.array(pos, dtype=float)
        self.vel = np.array(vel, dtype=float)
        self._yaw, self._pitch = self.get_angles()
        self._V = np.sqrt(self.vel.dot(self.vel))

    def get_angles(self):
        v = self.vel

        yaw = np.arctan2(v[1], v[0])
        pitch = np.arctan2(v[2], np.sqrt(v[0]*v[0] + v[1]*v[1]))
        return yaw, pitch

    @property
    def yaw(self):
        return self._yaw

    @yaw.setter
    def yaw(self, value):
        self._yaw = value
        self.vel = rot_to_unit(value, self.pitch) * self.V

    @property
    def pitch(self):
        return self._pitch

    @pitch.setter
    def pitch(self, value):
        self._pitch = value
        self.vel = rot_to_unit(self.yaw, value) * self.V

    @property
    def V(self):
        return self._V

    @V.setter
    def V(self, value):
        self._V = value
        self.vel = rot_to_unit(self.yaw, self.pitch) * value