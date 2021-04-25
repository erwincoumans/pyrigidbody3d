# pylint: disable=invalid-name
# Lint as: python3
"""Pose definition of a 3D coordinate frame.

Notation:

  pose_a = pose.Pose

"""

import numpy as onp
import numpy as np
import math
from pyrigidbody3d import quaternion


class Pose(object):
  """Pose is a coordinate frame specified as position and orientation quaternion."""

  def __init__(self, position=onp.zeros(3), orientation=quaternion.identity()):
    self.position = position
    self.orientation = quaternion.identity()

  def transform(self, point):
    """Transform a point by a pose."""
    return quaternion.rotate(self.orientation, point) + self.position

  def inverse_transform(self, point):
    """Inverse transform a point by a pose."""
    point_out = point - self.position
    inverse_orientation = quaternion.inverse(self.orientation)
    return quaternion.rotate(inverse_orientation, point_out)

  def matrix(self):
    q = self.orientation
    q = np.array([q.w,q.x,q.y,q.z], dtype=np.float64, copy=True)
    n = np.dot(q, q)
    q *= math.sqrt(2.0 / n)
    q = np.outer(q, q)
    return np.array([
        [1.0-q[2, 2]-q[3, 3],     q[1, 2]-q[3, 0],     q[1, 3]+q[2, 0], self.position[0]],
        [    q[1, 2]+q[3, 0], 1.0-q[1, 1]-q[3, 3],     q[2, 3]-q[1, 0], self.position[1]],
        [    q[1, 3]-q[2, 0],     q[2, 3]+q[1, 0], 1.0-q[1, 1]-q[2, 2], self.position[2]],
        [0.0                , 0.0                ,0.0                 , 1.0]])
