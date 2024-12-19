from typing import List, Union

import numpy as np


def generate_3d_figure8(
    num_points: int,
    width: float,
    center_offset: Union[np.ndarray, List[float]]
) -> np.ndarray:
    """Generate points describing a figure-eight shape in 3D space.

    The figure-eight lies in a series of planes parallel to the yz-plane,
    with a fixed offset along the x-axis.

    Args
    ----
    num_points: int
        The number of points to generate for the figure-eight shape.
    
    width: float
        The width of the figure-eight (in meters).
    
    center_offset: np.ndarray, List[float]
        The offset of the center of the figure-eight in 3D space, specified as [x, y, z].
    
    Returns
    -------
    points: np.ndarray
        A NumPy array of shape (num_points, 3), where each row represents a point in
        the 3D figure-eight with coordinates [x, y, z].
    """
    if isinstance(center_offset, list):
        center_offset = np.array(center_offset)
    
    if len(center_offset) != 3:
        raise ValueError("'center_offset' must have a length of 3 (x, y, z)")
    
    t = np.linspace(0, 2 * np.pi, num_points)

    x = np.full(num_points, center_offset[0])
    y = width * np.sin(2 * t) + center_offset[1]
    z = width * np.sin(t) + center_offset[2]

    return np.vstack((x, y, z)).T
