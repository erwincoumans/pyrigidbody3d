from __future__ import absolute_import
from __future__ import division
from __future__ import print_function

import numpy as np
from pyrigidbody3d import geometry
from pyrigidbody3d import pose
import unittest


class GeometryTest(unittest.TestCase):

  def test_sphere_sphere_collision(self):
    s_a = geometry.Sphere(1)
    pose_a = pose.Pose(position=np.array([0.0, 0.0, 1.0]))
    s_b = geometry.Sphere(1)
    pose_b = pose.Pose(position=np.array([0.0, 3.0, 1.0]))
    contacts = geometry.compute_contacts(s_a, pose_a, s_b, pose_b)
    for i in range(len(contacts)):
      c = contacts[i]
      print(i, "distance=", c.distance)
      self.assertEqual(c.distance, 1)
      self.assertLess(
          np.linalg.norm(c.world_normal_on_b - np.array([0.0, -1.0, 0.0])),
          1e-6)
      self.assertLess(
          np.linalg.norm(c.local_point_a - np.array([0.0, 1.0, 0.0])), 1e-6)
      self.assertLess(
          np.linalg.norm(c.local_point_b - np.array([0.0, -1.0, 0.0])), 1e-6)

  def test_plane_sphere_collision(self):
    p_a = geometry.Plane()
    pose_a = pose.Pose(position=np.array([0.0, 0.0, 0.0]))
    s_b = geometry.Sphere(0.5)
    pose_b = pose.Pose(position=np.array([0.0, 1.2, 30]))
    contacts = geometry.compute_contacts(p_a, pose_a, s_b, pose_b)
    for i in range(len(contacts)):
      c = contacts[i]
      self.assertEqual(c.distance, 29.5)
      self.assertLess(
          np.linalg.norm(c.world_normal_on_b - np.array([0.0, 0.0, -1.0])),
          1e-6)
      world_point_a = pose_a.transform(c.local_point_a)
      world_point_b = pose_b.transform(c.local_point_b)
      self.assertLess(
          np.linalg.norm(world_point_a - np.array([0., 1.2, 0.])), 1e-6)
      self.assertLess(
          np.linalg.norm(world_point_b - np.array([0., 1.2, 29.5])), 1e-6)

  def test_update_distance(self):
    s_a = geometry.Sphere(1)
    pose_a = pose.Pose(position=np.array([0.0, 0.0, 1.0]))
    s_b = geometry.Sphere(1)
    pose_b = pose.Pose(position=np.array([0.0, 3.0, 1.0]))
    contact = geometry.compute_contacts(s_a, pose_a, s_b, pose_b)
    self.assertEqual(len(contact), 1)

    pose_a = pose.Pose(position=np.array([0.0, 0.234, 1.0]))
    pose_b = pose.Pose(position=np.array([0.0, 3.486, 1.0]))
    contact[0].update_distance(pose_a, pose_b)
    updated = contact[0].distance

    contact = geometry.compute_contacts(s_a, pose_a, s_b, pose_b)
    self.assertEqual(len(contact), 1)
    self.assertLess(np.fabs(updated - contact[0].distance), 1e-6)


if __name__ == "__main__":
  unittest.main()
