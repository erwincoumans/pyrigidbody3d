from __future__ import absolute_import
from __future__ import division
from __future__ import print_function
import math
import numpy as np
from pyrigidbody3d import rigidbody
import unittest


class RigidbodyTest(unittest.TestCase):

  def test_init_test(self):
    b = rigidbody.RigidBody()
    self.assertLess(
        np.linalg.norm(b.world_pose.position - np.array([0.0, 0.0, 0.0])), 1e-6)

  def test_linear_velocity(self):
    b = rigidbody.RigidBody()
    num_steps = 100
    b.linear_velocity = np.array([1, 0, 0])
    b.angular_velocity = np.array([0, 2.0 * math.pi / float(num_steps), 0])
    dt = 1. / float(num_steps)
    for _ in range(num_steps):
      b.integrate(dt)
    self.assertLess(
        np.linalg.norm(b.world_pose.position - np.array([1.0, 0.0, 0.0])), 1e-6)


if __name__ == '__main__':
  unittest.main()
