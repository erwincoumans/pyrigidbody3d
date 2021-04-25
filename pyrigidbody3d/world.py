# pylint: disable=invalid-name
# Lint as: python3
"""Placeholder for Dynamics World for Rigid Body simulation.

Notation:

  w = world.World()
  b = rigidbody.RigidBody()
  w.bodies.append(b)
  w.step(dt)

"""
import numpy as np
from pyrigidbody3d import geometry
from pyrigidbody3d import rigidbody
DEFAULT_EARTH_GRAVITY = np.array([0.0, 0.0, -9.8])



class World(object):
  """World is a container a step method to perform forward dynamics."""

  def __init__(self, num_solver_iterations):
    self.bodies = []
    self.spherical_joints = []
    self.num_solver_iterations = num_solver_iterations
    self.gravity = DEFAULT_EARTH_GRAVITY

  def step(self, dt):
    """step performs forward dynamics including collision detection and response."""

    for b in self.bodies:
      b.apply_gravity(self.gravity)
      b.apply_force_impulse(dt)
      b.clear_forces()

    # perform collision detection
    contact_points = []
    for i0 in range(len(self.bodies)):
      for i1 in range(len(self.bodies)):
        if i0 < i1:
          body_0 = self.bodies[i0]
          body_1 = self.bodies[i1]
          if body_0 != body_1:
            pts = geometry.compute_contacts(body_0.collision_shape,
                                            body_0.world_pose,
                                            body_1.collision_shape,
                                            body_1.world_pose)
            for i in range(len(pts)):
              contact = pts[i]
              if contact.distance < 0:
                contact.body_a = body_0
                contact.body_b = body_1
                contact_points.append(contact)
    for i in range(self.num_solver_iterations):
      for joint in self.spherical_joints:
        joint.solve_joint(dt)
      for contact in contact_points:
        rigidbody.resolve_collision(contact, dt)

    # TODO(erwincoumans) implement LCP example.
    # if LCP, build A matrix, b vector
    # impulses = solveLCP(A,b)
    # for b in self.bodies:
    #   b.applyImpulse(impulses[index])

    for b in self.bodies:
      b.integrate(dt)

  def stepTGS(self, dt):
    """stepTGS performs forward dynamics.

    Forward dynamics includes applying gravity, collision detection
    and response.

    Args:
        dt: step size.
    """

    for b in self.bodies:
      b.apply_gravity()
      b.apply_force_impulse(dt)
      b.clear_forces()

    sub_dt = dt / self.num_solver_iterations
    # perform collision detection
    contact_points = []
    for i0 in range(len(self.bodies)):
      for i1 in range(len(self.bodies)):
        if i0 < i1:
          body_0 = self.bodies[i0]
          body_1 = self.bodies[i1]
          if body_0 != body_1:
            pts = geometry.compute_contacts(body_0.collision_shape,
                                            body_0.world_pose,
                                            body_1.collision_shape,
                                            body_1.world_pose)
            for i in range(len(pts)):
              contact = pts[i]
              # TGS needs a slightly more relaxed bound than PGS.
              if contact.distance < 1e-5:
                contact.body_a = body_0
                contact.body_b = body_1
                contact_points.append(contact)
    # Solve constraints.
    for i in range(self.num_solver_iterations):
      for contact in contact_points:
        contact.update_distance(contact.body_a.world_pose,
                                contact.body_b.world_pose)
      for contact in contact_points:
        rigidbody.resolve_collision(contact, sub_dt)
      for b in self.bodies:
        b.integrate(sub_dt)
