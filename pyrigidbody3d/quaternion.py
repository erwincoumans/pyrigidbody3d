# pylint: disable=invalid-name
# Lint as: python3
"""Quaternion Arithmetic.

Notation:

  q = quaternion.Quaternion(x, y, z, w)

  q = quaternion.identity()
  b = quaternion.inverse(a)
  q = quaternion.normalize(q)
  c = quaternion.multiply(a, b)
  v_out = quaternion.rotate(q, v_in)
  q = quaternion.from_axis_angle(axis, angle)

"""

import numpy as np
import numpy as onp
import math

class Quaternion(object):
  """Quaternion defines a 3D rotation and some operators."""

  def __init__(self, x, y, z, w):
    self.xyzw = onp.array([x, y, z, w])

  @property
  def x(self):
    return self.xyzw[0]

  @property
  def y(self):
    return self.xyzw[1]

  @property
  def z(self):
    return self.xyzw[2]

  @property
  def w(self):
    return self.xyzw[3]

  @property
  def vec(self):
    return self.xyzw[0:3]

  def normalize(self):
    """Convert to unit quaternion."""
    norm = np.linalg.norm(self.xyzw)
    self.xyzw = np.array(
        [self.x / norm, self.y / norm, self.z / norm, self.w / norm])


def normalized(q):
  """Convert to unit quaternion."""
  norm = np.linalg.norm(q.xyzw)
  return Quaternion(q.x / norm, q.y / norm, q.z / norm, q.w / norm)


def multiply(a, b):
  """Quaternion multiplication."""
  c = a.w * b.vec + b.w * a.vec + np.cross(a.vec, b.vec)
  w = a.w * b.w - np.dot(a.vec, b.vec)
  return Quaternion(x=c[0], y=c[1], z=c[2], w=w)


def from_axis_angle(axis, angle):
  """Create a Quaternion from an axis and angle description."""
  return Quaternion(*(axis * np.sin(angle / 2.0)), w=np.cos(angle / 2.0))


def rotate(q, v):
  """Rotation of a 3-vector by a unit quaternion."""
  q_v = Quaternion(*v, w=0.0)
  return (multiply(multiply(q, q_v), conjugate(q))).vec


identity = lambda: Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
conjugate = lambda q: Quaternion(x=-q.x, y=-q.y, z=-q.z, w=q.w)
inverse = lambda q: normalized(conjugate(q))
