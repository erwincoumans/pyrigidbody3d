# pylint: disable=invalid-name
# Lint as: python3
"""6 Degree of Freedom Rigid Body.

Notation:

  b = rigidbody.RigidBody()

  b.linear_velocity = np.array([1,0,0])
  b.integrate(dt)

"""

import numpy as np
import numpy as onp
from pyrigidbody3d import pose
from pyrigidbody3d import quaternion

# Baumgarte stabilization, fraction of constraint error reduced each step
BAUMGARTE_ERROR_REDUCTION_PARAMETER = 0.1


class RigidBody(object):
  """RigidBody with world pose, mass and inertia and collision geometry.

  world_pose of the center of mass with respect to the world frame.
  the rigid body is aligned with the principal axis of its inertial frame
  """

  def __init__(self, inv_mass=0.0, collision_shape=None):
    self.world_pose = pose.Pose()
    self.linear_velocity = np.zeros(3)
    self.angular_velocity = np.zeros(3)
    self.local_inertia = np.zeros(3)
    self.inv_mass = inv_mass
    # self.mass = np.where(self.inv_mass > 0.0, 1.0 / self.inv_mass, 0.0)
    if self.inv_mass > 0.0:
      self.mass = 1.0 / self.inv_mass
      # TODO(b/145025267) Update inv_inertia_world when rigidbody pose changes.
      self.inv_inertia_world = np.identity(3)
    else:
      self.mass = 0.0
      self.inv_inertia_world = np.zeros((3, 3))

    self.total_force = np.zeros(3)
    self.total_torque = np.zeros(3)
    self.collision_shape = collision_shape
    self.restitution = 0.0
    self.friction_coeffcient = 0.5

  def apply_gravity(self, gravity_acceleration):
    """Apply gravity force given the acceleration.

    Args:
        gravity_acceleration: list of 3 gravity acceleration components.
    """
    gravity_force = self.mass * gravity_acceleration
    self.apply_central_force(gravity_force)

  def apply_central_force(self, force):
    """Apply a force at the center of mass."""
    self.total_force += force

  def apply_force_impulse(self, dt):
    """Apply an impulse at the center of mass."""
    self.linear_velocity += self.total_force * self.inv_mass * dt
    self.angular_velocity += self.inv_inertia_world.dot(self.total_torque) * dt

  def apply_impulse(self, impulse, rel_pos):
    """Apply an impulse at a position relative to the center of mass."""
    self.linear_velocity += self.inv_mass * impulse
    torqueImpulse = np.cross(rel_pos, impulse)
    self.angular_velocity += self.inv_inertia_world.dot(torqueImpulse)

  def clear_forces(self):
    """Clear the applied forces and torques to zero."""
    self.total_force = np.zeros(3)
    self.total_torque = np.zeros(3)

  def get_velocity(self, rel_pos):
    """Return the velocity at a position relative to the center of mass.

    Args:
      rel_pos: position vector relative to center of mass..

    Returns:
      Velocity at this point in world coordinate frame.
    """
    return self.linear_velocity + np.cross(self.angular_velocity, rel_pos)

  def integrate(self, dt):
    """integrate the velocity.

    Args:
      dt: delta time (in seconds)

    Returns:
      None
    """
    self.world_pose.position += self.linear_velocity * dt
    # using quaternion derivative.
    # TODO(erwincoumans): use exponential map
    # https://www.cs.cmu.edu/~spiff/moedit99/expmap.pdf
    angular_velocity_orientation = quaternion.Quaternion(
        self.angular_velocity[0] * dt * 0.5,
        self.angular_velocity[1] * dt * 0.5,
        self.angular_velocity[2] * dt * 0.5, 0.0)
    self.world_pose.orientation.xyzw += quaternion.multiply(
        angular_velocity_orientation, self.world_pose.orientation).xyzw
    self.world_pose.orientation.normalize()


def resolve_collision(cp, dt):
  """resolve_collision is a numerical method that can be used to solve a contact linear complementarity (LCP) problem.

    Note that the LCP(A,b) is not explicitly constructed.
    Baumgarte stabilization is used to reduce positional drift. See description
    in Michael Cline's thesis:
    https://www.cs.ubc.ca/grads/resources/thesis/Nov02/Michael_Cline.pdf

  Args:
   cp: contact point
   dt: delta time (in seconds)
  """

  erp = BAUMGARTE_ERROR_REDUCTION_PARAMETER
  worldPointA = cp.body_a.world_pose.transform(cp.local_point_a)
  worldPointB = cp.body_b.world_pose.transform(cp.local_point_b)
  rel_posA = worldPointA - cp.body_a.world_pose.position
  rel_posB = worldPointB - cp.body_b.world_pose.position
  baumgarte_rel_vel = 0
  # This check is added for TGS, for which the distance needs to be checked
  # each time this function is called.
  if cp.distance < 0:
    baumgarte_rel_vel = erp * cp.distance / dt
  rel_vel = cp.body_a.get_velocity(rel_posA) - cp.body_b.get_velocity(rel_posB)
  normal_rel_vel = cp.world_normal_on_b.dot(rel_vel)
  if normal_rel_vel < 0.0:
    temp1 = cp.body_a.inv_inertia_world.dot(
        np.cross(rel_posA, cp.world_normal_on_b))
    temp2 = cp.body_b.inv_inertia_world.dot(
        np.cross(rel_posB, cp.world_normal_on_b))
    ang = cp.world_normal_on_b.dot(
        np.cross(temp1, rel_posA) + np.cross(temp2, rel_posB))
    impulse = (-(1.0 + cp.body_b.restitution) * normal_rel_vel -
               baumgarte_rel_vel) / (
                   cp.body_a.inv_mass + cp.body_b.inv_mass + ang)
    if impulse > 0:
      impulse_vector = impulse * cp.world_normal_on_b
      cp.body_a.apply_impulse(impulse_vector, rel_posA)
      cp.body_b.apply_impulse(-impulse_vector, rel_posB)
      lateral_rel_vel = rel_vel - normal_rel_vel * cp.world_normal_on_b
      friction_impulse_trial = np.linalg.norm(lateral_rel_vel) / (
          cp.body_a.inv_mass + cp.body_b.inv_mass + ang)
      friction_coeffcient = min(cp.body_a.friction_coeffcient,
                                cp.body_b.friction_coeffcient)
      if friction_impulse_trial < friction_coeffcient * impulse:
        friction = friction_impulse_trial
      else:
        friction = friction_coeffcient * impulse
      if np.linalg.norm(lateral_rel_vel) > 1e-5:
        friction_dir = lateral_rel_vel / np.linalg.norm(lateral_rel_vel)
      else:
        friction = 0
        friction_dir = np.array([1, 0, 0])
      cp.body_a.apply_impulse(-friction * friction_dir, rel_posA)
      cp.body_b.apply_impulse(friction * friction_dir, rel_posB)




