# pylint: disable=invalid-name
# Lint as: python3
"""Geometry definitions and Closest Point Computations.

Notation:

  s_a = geometry.Sphere(1.0)
  pose_a = pose.Pose
  s_b = geometry.Sphere(2.0)

  contacts = geometry.computeContacts(s_a, pose_a, s_b, pose_b)

"""

import numpy as np

SPHERE_TYPE = 0
PLANE_TYPE = 1
# CONTACT_EPSILON avoids division by zero when normalizing zero-length vectors.
CONTACT_EPSILON = 1.0e-6


class Sphere(object):
  """Sphere shape is an implicit sphere defined by its radius."""

  def __init__(self, radius=1.0):
    self.type = SPHERE_TYPE
    self.radius = radius


class Plane(object):
  """Plane shape is a half space defined by a plane normal and a plane constant."""

  def __init__(self):
    self.type = PLANE_TYPE
    self.normal = np.array([0.0, 0.0, 1.0])
    self.constant = 0.0


class ContactPoint(object):
  """ContactPoint stores a closest point between body A and body B.

  Positive distance means objects are separate, negative distance is overlap.
  Contact normal is defined in world (Cartesian) space pointing from B to A.
  Two witness points are specified in local body coordinates.
  """

  def __init__(self, distance, world_normal_on_b, local_point_a, local_point_b):
    self.distance = distance
    self.world_normal_on_b = world_normal_on_b
    self.local_point_a = local_point_a
    self.local_point_b = local_point_b

  def update_distance(self, transform_a, transform_b):
    """update_distance updates distance between two objects.

    Instead of doing a full collision detection, approximate distance between
    two objects by reusing contact points. Project the world position
    difference between the two points to the previously computed normal.

    Args:
      transform_a: transform of body A.
      transform_b: transform of body B.
    """
    point_a_world = transform_a.transform(self.local_point_a)
    point_b_world = transform_b.transform(self.local_point_b)
    diff = point_a_world - point_b_world
    self.distance = np.dot(diff, self.world_normal_on_b)


def contact_sphere_sphere(sphere_a, pose_a, sphere_b, pose_b):
  """Compute closest point between twp spheres.

  Args:
    sphere_a: first sphere
    pose_a: world position of first sphere
    sphere_b: second sphere
    pose_b: world position and orientation of second sphere

  Returns:
    List containing one ContactPoint.
  """

  diff = pose_a.position - pose_b.position
  length = np.linalg.norm(diff)
  distance = length - (sphere_a.radius + sphere_b.radius)
  normal_on_b = np.array([1.0, 0.0, 0.0])
  if length > CONTACT_EPSILON:
    normal_on_b = 1.0 / length * diff
  point_a_world = pose_a.position - sphere_a.radius * normal_on_b
  point_b_world = point_a_world - distance * normal_on_b
  local_point_a = pose_a.inverse_transform(point_a_world)
  local_point_b = pose_b.inverse_transform(point_b_world)
  pt = ContactPoint(distance, normal_on_b, local_point_a, local_point_b)
  return [pt]


def contact_plane_sphere(plane_a, pose_a, sphere_b, pose_b):
  """Compute closest point between a plane and a sphere.

  Args:
    plane_a: plane
    pose_a: world position of the plane
    sphere_b: sphere
    pose_b: world position and orientation of the sphere

  Returns:
    List containing one ContactPoint.
  """
  t = -(pose_b.position.dot(-plane_a.normal) + plane_a.constant)
  point_a_world = pose_b.position + t * -plane_a.normal
  distance = t - sphere_b.radius
  point_b_world = pose_b.position - sphere_b.radius * plane_a.normal
  local_point_a = pose_a.inverse_transform(point_a_world)
  local_point_b = pose_b.inverse_transform(point_b_world)
  pt = ContactPoint(distance, -plane_a.normal, local_point_a, local_point_b)
  return [pt]


# TODO(erwincoumans) Implement contact_sphere_plane calling contact_plane_sphere
#   and swapping the contact normal.
# def contact_sphere_plane(sphereA, poseA, planeB, poseB):
#   pt = contact_plane_sphere(planeB, poseB, sphereA, poseA)[0]
#   pt.world_normal_on_b *= -1
#   return [pt]

contactFunctions = [[contact_sphere_sphere, None], [contact_plane_sphere, None]]
#contactFunctions = [[None,None], [contact_plane_sphere,None]]

def compute_contacts(shape_a, pose_a, shape_b, pose_b):
  """Compute closest points between two shapes.

  Args:
    shape_a: first shape
    pose_a: world position of the first shape
    shape_b: second shape
    pose_b: world position and orientation of the second shape

  Returns:
    a list of contact points.
  """
  func = contactFunctions[shape_a.type][shape_b.type]
  if (func is not None):
    return func(shape_a, pose_a, shape_b, pose_b)
  return []

