# pylint: disable=invalid-name
# Lint as: python3
"""Pose definition of a 3D coordinate frame.

Notation:

  pose_a = pose.Pose

"""

import numpy as onp

import quaternion


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
