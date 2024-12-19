from pathlib import Path

import numpy as np
import roboticstoolbox as rtb

_path_file = Path(__file__).resolve().parent

PATH_URDF = _path_file / "urdf/robot.urdf"

PLOT_LIMITS = [-0.5, 0.5, -0.5, 0.5, 0, 0.5]

class ArmRobot(rtb.Robot):
    def __init__(self):
        if not PATH_URDF.exists():
            raise FileNotFoundError(f"URDF file not found: {PATH_URDF}")

        links, name, urdf_string, urdf_filepath = self.URDF_read(PATH_URDF)

        super().__init__(
            links,
            name=name,
            urdf_string=urdf_string,
            urdf_filepath=urdf_filepath,
        )

        self.qz = np.zeros(4)

        self.addconfiguration("qz", self.qz)


if __name__ == '__main__':
    import spatialmath as sm

    robot = ArmRobot()

    print(robot)

    limits = [-0.5, 0.5, -0.5, 0.5, 0, 0.5]

    target_pos = sm.SE3([0.15, -0.15, 0.15])

    solution = robot.ikine_LM(
        Tep=target_pos,
        q0=robot.qz,
        tol=1e-6,
        mask=[1, 1, 1, 0, 0, 0]
    )

    robot.dynamics()

    if solution.success:
        q_deg = np.degrees(solution.q)
        print(f"Joint angles: {q_deg}")

        robot.plot(solution.q, backend='pyplot', block=True, limits=limits)
    else:
        ...
