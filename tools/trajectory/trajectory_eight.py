import matplotlib.pyplot as plt
import numpy as np
import roboticstoolbox as rtb
import spatialmath as sm

from tools.figure import generate_3d_figure8
from tools.model import PLOT_LIMITS, ArmRobot

# Parameters of the eight
width = 0.125
center_offset = [0.15, 0, 0.2]
num_points = 100

# Parameters of the trajectory
dt = 0.02
tacc = 0.06
qdmax = 1
qd0 = np.zeros(3)
qdf = np.zeros(3)
t_segment = np.linspace(0, 2, 100)

# Generating the figure of eight
via_points = generate_3d_figure8(num_points, width, center_offset)

# Generating trajectory
traj = rtb.mstraj(
    viapoints=via_points,
    dt=dt,
    tacc=tacc,
    qdmax=qdmax,
    q0=via_points[0],
    qd0=qd0,
    qdf=qdf
)

traj.plot(block=True)

fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
ax.clear()
ax.plot(traj.q[:, 0], traj.q[:, 1], traj.q[:, 2])
plt.show()

# Getting the position of the joints
robot = ArmRobot()

all_trajectories = []
joint_angles = []

goal_pos = sm.SE3(traj.q[0, :])
q_pos = robot.ikine_LM(
    Tep=goal_pos,
    q0=robot.qz,
    tol=1e-6,
    mask=[1, 1, 1, 0, 0, 0]
).q

traj_q_pos = rtb.jtraj(robot.qz, q_pos, t_segment)
all_trajectories.append(traj_q_pos.q)

for i in range(1, traj.q.shape[0]):
    goal_pos = sm.SE3(traj.q[i, :])

    q_solutions = robot.ikine_LM(
        Tep=goal_pos,
        q0=q_pos,
        tol=1e-6,
        mask=[1, 1, 1, 0, 0, 0]
    )

    q_pos = q_solutions.q

    joint_angles.append(q_solutions.q)

joint_angles = np.array(joint_angles)
all_trajectories.append(joint_angles)

traj_q0 = rtb.jtraj(q_pos, robot.qz, t_segment)
all_trajectories.append(traj_q0.q)

combined_trajectory = np.vstack(all_trajectories)

np.save("eight_trajectory.npy", combined_trajectory)

robot.plot(combined_trajectory, backend='pyplot', dt=dt, block=True, limits=PLOT_LIMITS)
