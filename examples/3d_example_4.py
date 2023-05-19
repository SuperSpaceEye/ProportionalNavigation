# Control through gimbal and pid with angular inertia, lagged observations

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
drag = 0.99

pursuer_acc = g * 3

target_v = g

# rotation and acceleration direction
pursuer_rot = pn.HeadingVelocity3d(np.deg2rad(90), 0, np.array([0, 0, 0]), 0)
# velocity and position
pursuer_physic = pn.HeadingVelocity3d(0, 0, np.array([20, 20, 20]), 0)

# lagged_pursuer_rot  = pn.HeadingVelocity3d(np.deg2rad(0), 0, np.array([0, 0, 0]), 0)
# lagged_pursuer_phys = pn.HeadingVelocity3d(np.deg2rad(0), 0, np.array([0, 0, 0]), 0)
lagged_pursuer_phys = pursuer_physic

actual_target = pn.HeadingVelocity3d(np.deg2rad(0), 0, np.array([100, 100, 0]), target_v)
lagged_target = pn.HeadingVelocity3d(np.deg2rad(0), 0, np.array([100, 100, 0]), target_v)

dt = 1. / 20
N = 5

update_actual_every = 1

Kp, Kl, Kd = 1, 1, 1 # parameters of pid
modif = 20. # allows pid finer control but makes it slower to respond
limits = (np.deg2rad(-22.5) * modif, np.deg2rad(22.5) * modif) # simple_pid outputs only integers for some reason

max_sim_time = 20  # maximum sim time

R_min = 3  # minimum distance between target and pursuer before terminating

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

counter = 0

log = {'pursuer': [], 'target': [], "lagged_target":[], "lagged_pursuer":[], "pursuer_vel":[], "t_go":[], "ZEM_i":[], "ZEM_n":[]}
while True:
    # ========== Physics simulation ==========
    # simulating pursuer and target movement's
    pursuer_physic.pos += pursuer_physic.vel * dt
    pursuer_physic.pos[1] -= g * dt

    if pursuer_physic.pos[1] < 0:
        pursuer_physic.pos[1] = 0

    actual_target.pos += actual_target.vel * dt
    actual_target.yaw = np.cos(t)
    actual_target.pitch = np.cos(t / 4)
    # actual_target.V = 0

    # simulating angular inertia
    pursuer_rot.pitch += pitch_ar
    pursuer_rot.yaw += yaw_ar

    # ========== Homing loop ==========
    if counter % update_actual_every == 0:
        lagged_target.pos = np.array(actual_target.pos)
        lagged_target.vel = np.array(actual_target.vel)

        # lagged_pursuer_phys.pos = np.array(pursuer_physic.pos)
        # lagged_pursuer_phys.vel = np.array(pursuer_physic.vel)
    else:
        lagged_target.pos += lagged_target.vel * dt
    counter+=1

    ret = pn.ZEM_3d(lagged_pursuer_phys, lagged_target, N)
    nL = ret['nL']
    R = ret['R']

    log["pursuer"].append(list(pursuer_physic.pos))
    log['target'].append(list(actual_target.pos))
    log["lagged_target"].append(list(lagged_target.pos))
    log["pursuer_vel"].append(np.sqrt(pursuer_physic.vel.dot(pursuer_physic.vel)))

    log["t_go"].append(ret["t_go"])
    log["ZEM_i"].append(ret["ZEM_i"])
    log["ZEM_n"].append(ret["ZEM_n"])
    # log["lagged_pursuer"].append(list(lagged_pursuer_phys.pos))

    t = t + dt
    if R <= R_min or t > max_sim_time:
        break

    # setting new targets and calculating gimbal
    new_yaw, new_pitch = get_angles(nL)
    pitch_pid.set_starting(new_pitch); yaw_pid.set_starting(new_yaw)
    pitch_gimbal, yaw_gimbal = pitch_pid(pursuer_rot.pitch) / modif, yaw_pid(pursuer_rot.yaw) / modif

    pitch_ar_change = np.sin(pitch_gimbal) * pursuer_acc * dt
    yaw_ar_change   = np.sin(yaw_gimbal)   * pursuer_acc * dt

    # lagged_pursuer_rot.pitch += pitch_ar + pitch_ar_change
    # lagged_pursuer_rot.yaw   += yaw_ar   + yaw_ar_change

    pursuer_rot.pitch = pi_clip(pursuer_rot.pitch)
    pursuer_rot.yaw   = pi_clip(pursuer_rot.yaw)

    # lagged_pursuer_rot.pitch = pi_clip(lagged_pursuer_rot.pitch)
    # lagged_pursuer_rot.yaw   = pi_clip(lagged_pursuer_rot.yaw)
    #
    # lagged_pursuer_phys.vel = (lagged_pursuer_phys.vel + lagged_pursuer_rot.vel * dt) * (1 - drag*dt)
    # ========== Post homing loop simulation ==========

    # simulating gimbal change of angular inertia
    pitch_ar += pitch_ar_change
    yaw_ar   += yaw_ar_change

    pursuer_rot.V = pursuer_acc * abs(np.cos(pitch_gimbal) * np.cos(yaw_gimbal))

    pursuer_physic.vel = (pursuer_physic.vel + pursuer_rot.vel * dt) * (1 - drag*dt)

    change_ticks += 1

distance = [sqrt((x1 - x2)**2 + (y1 - y2)**2) + (z1-z2)**2 for (x1, y1, z1), (x2, y2, z2) in
            zip(log["pursuer"], log["target"])]

print(min(distance))

# actual = np.array(log["pursuer"])
# lagged = np.array(log["lagged_pursuer"])
#
# print("pursuer_diff", np.round(abs(sum(actual - lagged)) / len(actual)))
#
# actual = np.array(log["target"])
# lagged = np.array(log["lagged_target"])
#
# print("target_diff", np.round(abs(sum(actual - lagged)) / len(actual)))

fig = plt.figure(figsize=(12, 12))
ax = fig.add_subplot(projection="3d")

px, py, pz = [], [], []
for x, y, z in log["pursuer"]: px.append(x); py.append(y); pz.append(z)

tx, ty, tz = [], [], []
for x, y, z in log["target"]: tx.append(x); ty.append(y); tz.append(z)

# lx, ly, lz = [], [], []
# for x, y, z in log["lagged_pursuer"]: lx.append(x); ly.append(y); lz.append(z)

ax.scatter(px, pz, py, c=range(len(distance)), cmap=matplotlib.colormaps["Greens"])
ax.scatter(tx, tz, ty, c=range(len(distance)), cmap=matplotlib.colormaps["Reds"])
# ax.scatter(lx, lz, ly, c=range(len(distance)), cmap=matplotlib.colormaps["Blues"])

plt.show()

time = np.array(range(len(distance)))*dt

fig, ax = plt.subplots(figsize=(12, 12), nrows=2, ncols=3)

ax[0][0].plot(time, log["pursuer_vel"])
ax[0][0].set_ylabel("pursuer vel")
ax[0][1].plot(time, log["t_go"])
ax[0][1].set_ylabel("time to collision")
ax[0][2].plot(time, distance)
ax[0][2].set_ylabel("distance")

ax[1][0].plot(time, [np.sqrt(it.dot(it)) for it in log["ZEM_n"]])
ax[1][0].set_ylabel("zero effort miss")

ax[1][1].plot(time, [np.log2(np.sqrt(it.dot(it))) for it in log["ZEM_n"]])
ax[1][1].set_ylabel("log2 zero effort miss")

plt.show()

