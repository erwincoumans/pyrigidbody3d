import numpy as np

class SphericalJoint(object):
  """SphericalJoint  stores a two local points connecting body A and body B.
  """

  def __init__(self, local_point_a, local_point_b):
    self.local_point_a = local_point_a
    self.local_point_b = local_point_b

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

