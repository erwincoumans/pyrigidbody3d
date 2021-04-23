# Lint as: python3
"""Example to simulated a dynamics world using PyBullet for visualization.

Usage:
  Build and launch a Bullet physics server, purely for visualization (not
  physics simulation!)

  pip install pybullet --user

  Now run the world_example:

  python world_example.py

"""

from __future__ import absolute_import
from __future__ import division
from __future__ import print_function

from absl import app
from absl import flags
import numpy as np
from pyrigidbody3d import geometry
from pyrigidbody3d import rigidbody
from pyrigidbody3d import world
import pybullet as p
import pybullet_data as pd

SIMULATION_TIME_STEP = 1. / (60.)
NUM_SOLVER_ITERATIONS = 20

FLAGS = flags.FLAGS

USE_TGS = False

USE_TWO_SPHERE = True


def main(argv):
  if len(argv) > 1:
    raise app.UsageError('Too many command-line arguments.')

  p.connect(p.GUI)
  p.resetSimulation()
  p.setAdditionalSearchPath(pd.getDataPath())
  plane_id = p.loadURDF('plane.urdf')
  sphere_id = p.loadURDF('sphere2.urdf', [0., 0., .5])
  if USE_TWO_SPHERE:
    sphere2_id = p.loadURDF('sphere2.urdf', [0., 0., 2.])
  w = world.World(NUM_SOLVER_ITERATIONS)

  plane = geometry.Plane()
  body_0 = rigidbody.RigidBody(inv_mass=0.0, collision_shape=plane)
  w.bodies.append(body_0)

  sphere = geometry.Sphere(0.5)
  body_1 = rigidbody.RigidBody(inv_mass=1.0, collision_shape=sphere)
  body_1.world_pose.position = np.array([0., 0., .5])
  w.bodies.append(body_1)

  if USE_TWO_SPHERE:
    body_2 = rigidbody.RigidBody(inv_mass=2, collision_shape=sphere)
    body_2.world_pose.position = np.array([0., 0.2, 1.5])
    w.bodies.append(body_2)

  dt = SIMULATION_TIME_STEP
  for s in range(1000):
    print('step ', s)
    if USE_TGS:
      w.stepTGS(dt)
    else:
      w.step(dt)

    # Set plane position and orientation for visualization.
    orientation = body_0.world_pose.orientation
    p.resetBasePositionAndOrientation(
        plane_id, body_0.world_pose.position,
        [orientation.x, orientation.y, orientation.z, orientation.w])

    # Set sphere position and orientation for visualization.
    orientation = body_1.world_pose.orientation
    p.resetBasePositionAndOrientation(
        sphere_id, body_1.world_pose.position,
        [orientation.x, orientation.y, orientation.z, orientation.w])

    if USE_TWO_SPHERE:
      orientation = body_2.world_pose.orientation
      p.resetBasePositionAndOrientation(
          sphere2_id, body_2.world_pose.position,
          [orientation.x, orientation.y, orientation.z, orientation.w])


if __name__ == '__main__':
  app.run(main)
