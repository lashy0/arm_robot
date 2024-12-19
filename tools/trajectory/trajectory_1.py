"""
This script computes and visualizes a robot's trajectory through
a series of target positions in the workspace.
It calculates joint configurations using inverse kinematics,
generates smooth trajectories, and saves the result.
"""
import numpy as np
import roboticstoolbox as rtb
import spatialmath as sm

from tools.model import PLOT_LIMITS, ArmRobot

robot = ArmRobot()

# Intermediate points in the workspace
target_positions = [
    sm.SE3([0.15, 0, 0.2]),
    sm.SE3([0.2, 0.1, 0.25]),
    sm.SE3([0.1, -0.1, 0.3]),
    sm.SE3([0.2, -0.2, 0.25]),
    sm.SE3([0.3, 0.0, 0.2]),
    sm.SE3([0.15, 0, 0.2]),
    sm.SE3([0.1, 0.1, 0.2]),
    sm.SE3([0.1, -0.15, 0.3]),
    sm.SE3([0.15, 0.15, 0.2])
]

# Initial configuration (i.e., robot's starting position)
q0 = robot.qz

# List of joint configurations
joint_configurations = [q0]

# Compute joint angles for each target position
for pos in target_positions:
    q_sol = robot.ikine_LM(
        Tep=pos,
        q0=joint_configurations[-1],
        tol=1e-6,
        mask=[1, 1, 1, 0, 0, 0]
    ).q
    joint_configurations.append(q_sol)

# Return to the initial position
joint_configurations.append(robot.qz)

# Create trajectories between points
t_per_segment = np.linspace(0, 2.5, 100)  # Time for each segment
all_trajectories = []

for i in range(len(joint_configurations) - 1):
    #  robot.jtraj передать уже в SE3
    # Generate joint trajectories
    traj = rtb.jtraj(joint_configurations[i], joint_configurations[i + 1], t_per_segment)
    all_trajectories.append(traj.q)

    # traj.plot(block=True)

# Combine all trajectories
combined_trajectory = np.vstack(all_trajectories)

np.save("trajectory.npy", combined_trajectory)

robot.plot(combined_trajectory, backend="pyplot", block=True, dt=0.025, limits=PLOT_LIMITS)
