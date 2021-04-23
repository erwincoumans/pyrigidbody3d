from __future__ import absolute_import
from __future__ import division
from __future__ import print_function

import numpy as np
from pyrigidbody3d import quaternion
import unittest


class QuaternionTest(unittest.TestCase):

  def test_identity(self):
    q = quaternion.identity()
    self.assertLess(
        np.linalg.norm(q.xyzw - np.array([0.0, 0.0, 0.0, 1.0])), 1e-6)

  def test_rotate(self):
    q = quaternion.from_axis_angle(np.array([0.0, 0.0, 1.0]), np.pi / 2)
    v = np.array([1.0, 0.0, 0.0])
    v_rot = quaternion.rotate(q, v)
    self.assertLess(np.linalg.norm(v_rot - np.array([0.0, 1.0, 0.0])), 1e-6)

  def test_cross(self):
    res = np.cross(np.array([1.0, 0.0, 0.0]), np.array([0.0, 1.0, 0.0]))
    self.assertLess(np.linalg.norm(res - np.array([0.0, 0.0, 1.0])), 1e-6)

  def test_normalize(self):
    q = quaternion.Quaternion(0.0, 0.0, 0.0, 2.0)
    q.normalize()
    self.assertLess(
        np.linalg.norm(q.xyzw - np.array([0.0, 0.0, 0.0, 1.0])), 1e-6)


if __name__ == '__main__':
  unittest.main()
