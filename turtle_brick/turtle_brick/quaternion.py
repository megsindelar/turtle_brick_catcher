from geometry_msgs.msg import Quaternion
from math import sin, cos, sqrt


# This code was used from Matthew Elwin
def angle_axis_to_quaternion(theta, axis):
    """
    Convert from angle-axis of rotation to a quaternion.

    Args:
      theta: angle in radians
      axis: axis of rotation

    Returns
    -------
      Quaternion from the given rotation

    """
    magnitude = sqrt(axis[0]**2 + axis[1]**2 + axis[2]**2)
    normalized = [v/magnitude for v in axis]
    sinTheta2 = sin(theta/2.0)
    return Quaternion(x=normalized[0]*sinTheta2,
                      y=normalized[1]*sinTheta2,
                      z=normalized[2]*sinTheta2,
                      w=cos(theta/2.0))
