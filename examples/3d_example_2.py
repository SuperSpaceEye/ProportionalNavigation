# Control through gimbal and pid with angular inertia

import sys
sys.path.append("..")
sys.path.append("../ProportionalNavigation")
sys.path.append("../ProportionalNavigation/fn_3d")
import ProportionalNavigation.fn_3d as pn
import matplotlib.pyplot as plt
from simple_pid import PID  # https://pypi.org/project/simple-pid/
import numpy as np
import math

def _clamp(value, limits):
    lower, upper = limits
    if value is None:
        return None
    elif (upper is not None) and (value > upper):
        return upper
    elif (lower is not None) and (value < lower):
        return lower
    return value
def pid_starting(self, starting):
    self._integral = _clamp(starting, self.output_limits)
PID.set_starting = pid_starting
def get_angles(v):
    yaw = np.arctan2(v[1], v[0])
    pitch = np.arctan2(v[2], np.sqrt(v[0] * v[0] + v[1] * v[1]))
    return yaw, pitch

import matplotlib
matplotlib.use("GTK3Agg")

sqrt = math.sqrt
t = 0
def time_fn(): return t  # for pids


g = 10

pursuer_v = g * 3
target_v  = g

pursuer = pn.HeadingVelocity3d(np.deg2rad(90), 0, np.array([0, 0, 0]), pursuer_v)
target = pn.HeadingVelocity3d(np.deg2rad(0), 0, np.array([100, 100, 0]), target_v)
dt = 1. / 20
N = 5

Kp, Kl, Kd = 1, 1, 1 # parameters of pid
modif = 100. # allows pid finer control but makes it slower to respond
limits = (np.deg2rad(-22.5) * modif, np.deg2rad(22.5) * modif) # simple_pid outputs only integers for some reason

max_sim_time = 40  # maximum sim time

R_min = 5  # minimum distance between target and pursuer before terminating

def pi_clip(angle):
    if angle > 0:
        if angle > math.pi * modif:
            return angle - 2 * math.pi * modif
    else:
        if angle < -math.pi * modif:
            return angle + 2 * math.pi * modif
    return angle

pitch_pid = PID(Kp, Kl, Kd, 0, sample_time=dt, output_limits=limits, error_map=pi_clip, time_fn=time_fn)
yaw_pid   = PID(Kp, Kl, Kd, 0, sample_time=dt, output_limits=limits, error_map=pi_clip, time_fn=time_fn)


change_ticks = 0

ret = None
nL = None
R = None
new_yaw = None
new_pitch = None

# angular rotation of pursuer
pitch_ar = 0
yaw_ar = 0

log = {'pursuer': [], 'target': [], "pursuer_vel":[], "pursuer_acc":[]}
while True:
    ret = pn.ZEM_3d(pursuer, target, N=N)
    nL = ret['nL']
    R = ret['R']

    log["pursuer"].append(list(pursuer.pos))
    log['target'].append(list(target.pos))
    log["pursuer_vel"].append(pursuer.V)

    t = t + dt
    if R <= R_min or t > max_sim_time:
        break

    # simulating pursuer and target movement's
    pursuer.pos += pursuer.vel * dt
    pursuer.pos[1] -= g * dt

    target.pos += target.vel * dt
    target.yaw = np.cos(t)
    target.pitch = np.cos(t / 5)

    # setting new targets and calculating gimbal
    new_yaw, new_pitch = get_angles(nL*dt)
    pitch_pid.set_starting(new_pitch); yaw_pid.set_starting(new_yaw)
    pitch_gimbal, yaw_gimbal = pitch_pid(pursuer.pitch)/modif, yaw_pid(pursuer.yaw)/modif

    # simulating angular inertia
    pursuer.pitch += pitch_ar
    pursuer.yaw   += yaw_ar

    # simulating gimbal change of angular inertia
    pitch_ar += np.sin(pitch_gimbal) * pursuer_v * dt
    yaw_ar   += np.sin(yaw_gimbal)   * pursuer_v * dt

    pursuer.pitch = pi_clip(pursuer.pitch)
    pursuer.yaw   = pi_clip(pursuer.yaw)

    change_ticks += 1

    pursuer.V = abs(np.cos(pitch_gimbal) * np.cos(yaw_gimbal)) * pursuer_v

distance = [sqrt((x1 - x2)**2 + (y1 - y2)**2 + (z1-z2)**2) for (x1, y1, z1), (x2, y2, z2) in
            zip(log["pursuer"], log["target"])]

print(min(distance))

fig = plt.figure(figsize=(12, 12))
ax = fig.add_subplot(projection="3d")

px, py, pz = [], [], []
for x, y, z in log["pursuer"]: px.append(x); py.append(y); pz.append(z)

tx, ty, tz = [], [], []
for x, y, z in log["target"]: tx.append(x); ty.append(y); tz.append(z)

ax.scatter(px, pz, py, c=range(len(distance)), cmap=matplotlib.colormaps["Greens"])
ax.scatter(tx, tz, ty, c=range(len(distance)), cmap=matplotlib.colormaps["Reds"])

plt.show()

# plt.scatter(range(len(distance)), log["pursuer_vel"])
# plt.colorbar()
# plt.show()
