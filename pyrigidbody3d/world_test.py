from __future__ import absolute_import
from __future__ import division
from __future__ import print_function

import copy
import numpy as np

from pyrigidbody3d import geometry
from pyrigidbody3d import rigidbody
from pyrigidbody3d import world
import unittest

SIMULATION_TIME_STEP = 1. / 60.
NUM_SOLVER_ITERATIONS = 20


class WorldTest(unittest.TestCase):

  def test_world_step(self):
    # For now we only test if the code is running. No unit test yet.
    my_world = world.World(NUM_SOLVER_ITERATIONS)

    plane = geometry.Plane()
    plane_id = rigidbody.RigidBody(inv_mass=0.0, collision_shape=plane)
    my_world.bodies.append(plane_id)

    sphere = geometry.Sphere(0.5)
    sphere_id = rigidbody.RigidBody(inv_mass=1.0, collision_shape=sphere)
    sphere_id.world_pose.position = np.array([0., 0., 0.6])
    my_world.bodies.append(sphere_id)

    dt = SIMULATION_TIME_STEP
    for _ in range(10):
      my_world.step(dt)

  def test_friction(self):
    my_world = world.World(NUM_SOLVER_ITERATIONS)

    plane = geometry.Plane()
    body_0 = rigidbody.RigidBody(inv_mass=0.0, collision_shape=plane)
    body_0.friction_coeffcient = 1.0
    my_world.bodies.append(body_0)
    original_plane_pose = copy.deepcopy(body_0.world_pose)

    radius = 0.5
    sphere = geometry.Sphere(radius)
    body_1 = rigidbody.RigidBody(inv_mass=1.0, collision_shape=sphere)
    body_1.world_pose.position = np.array([0., 0., .5])
    body_1.linear_velocity = np.array([0., 1, 0])
    body_1.friction_coeffcient = 1.0
    my_world.bodies.append(body_1)

    dt = SIMULATION_TIME_STEP

    for _ in range(2):
      my_world.step(dt)
    self.assertLess(abs(body_1.linear_velocity[1]), 1.0)
    self.assertLess(0.2, abs(body_1.linear_velocity[1]))

    for _ in range(20):
      my_world.step(dt)

    # Static objects should not move.
    self.assertAlmostEqual(original_plane_pose.position.all(),
                           body_0.world_pose.position.all())
    self.assertAlmostEqual(original_plane_pose.orientation.xyzw.all(),
                           body_0.world_pose.orientation.xyzw.all())

    # Velocity at contact point with ground is close to zero.
    contact_vel = body_1.linear_velocity + np.cross(body_1.angular_velocity,
                                                    np.array([0, 0, -radius]))
    self.assertLess(np.linalg.norm(contact_vel), 1e-2)


if __name__ == "__main__":
  unittest.main()
