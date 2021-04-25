import numpy as np


class SphericalJoint(object):
  """SphericalJoint  stores a two local points connecting body A and body B.
  """

  def __init__(self, local_point_a, local_point_b):
    self.local_point_a = local_point_a
    self.local_point_b = local_point_b
    self.erp = 0.1

  def projected_distance(self, point_a_world, point_b_world, normal_world):
    """compute the distance between two points projected onto a normal.

    Args:
      transform_a: transform of body A.
      transform_b: transform of body B.
      normal_world: normal to project distance
    """
    diff = point_a_world - point_b_world
    distance = np.dot(diff, normal_world)
    return distance


  def solve_joint(self, dt):
    joint = self
    directions = np.array([[1,0,0],[0,1,0],[0,0,1]])
    for normal in directions:
      worldPointA = joint.body_a.world_pose.transform(joint.local_point_a)
      worldPointB = joint.body_b.world_pose.transform(joint.local_point_b)
      rel_posA = worldPointA - joint.body_a.world_pose.position
      rel_posB = worldPointB - joint.body_b.world_pose.position
      distance = joint.projected_distance(worldPointA, worldPointB, normal)
      baumgarte_rel_vel = self.erp * distance / dt
      
      rel_vel = joint.body_a.get_velocity(rel_posA) - joint.body_b.get_velocity(rel_posB)
      normal_rel_vel = normal.dot(rel_vel)
      temp1 = joint.body_a.inv_inertia_world.dot(
          np.cross(rel_posA, normal))
      temp2 = joint.body_b.inv_inertia_world.dot(
          np.cross(rel_posB, normal))
      ang = normal.dot(
          np.cross(temp1, rel_posA) + np.cross(temp2, rel_posB))
      impulse = (-(1.0 + joint.body_b.restitution) * normal_rel_vel -
                   baumgarte_rel_vel) / (
                       joint.body_a.inv_mass + joint.body_b.inv_mass + ang)
      impulse_vector = impulse * normal
      joint.body_a.apply_impulse(impulse_vector, rel_posA)
      joint.body_b.apply_impulse(-impulse_vector, rel_posB)
