from __future__ import absolute_import
from __future__ import division
from __future__ import print_function

import numpy as np
import numpy as onp
from pyrigidbody3d import pose
import unittest


class PoseTest(unittest.TestCase):

  def test_init(self):
    p = pose.Pose()
    self.assertLess(np.linalg.norm(p.position), 1e-6)

  def test_transform(self):
    """Transform a point by a pose."""
    pt = np.array([1.0, 2.0, 3.0])
    tr = pose.Pose()
    tr.position = onp.array([4.0, 5.0, 6.0])
    pt2 = tr.transform(pt)
    self.assertLess(np.linalg.norm(pt2 - np.array([5.0, 7.0, 9.0])), 1e-6)


if __name__ == '__main__':
  unittest.main()
