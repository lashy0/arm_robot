import numpy as np
import roboticstoolbox as rtb
import spatialmath as sm

from tools.model import ArmRobot

robot =  ArmRobot()

Tep_pos = sm.SE3([0.15, -0.15, 0.15])

solution = robot.ikine_LM(
    Tep=Tep_pos,
    q0=robot.qz,
    tol=1e-6,
    mask=[1, 1, 1, 0, 0, 0]
)

if solution.success:
    t = np.linspace(0, 3, 100)

    traj = rtb.jtraj(robot.qz, solution.q, t)

    traj.plot(block=True)

    robot.plot(traj.q, backend='pyplot', block=True, dt=0.03)
