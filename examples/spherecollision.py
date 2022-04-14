
import sys
import copy
import numpy as np
from pyrigidbody3d import geometry
from pyrigidbody3d import rigidbody
from pyrigidbody3d import world

# real-time updates are a bit choppy
import meshcat
import meshcat.geometry as g
import meshcat.transformations as tf
import math
import time

import numpy as np
 

SIMULATION_TIME_STEP = 1. / 60.#240.
NUM_SOLVER_ITERATIONS = 20
RADIUS=0.5

physics_world = world.World(NUM_SOLVER_ITERATIONS)
physics_world.gravity = np.array([0.0, 0.0, -9.8])
use_viz = False

if use_viz:
  vis = meshcat.Visualizer().open()

#physics sphere
sphere = geometry.Sphere(RADIUS)
sphere_id = rigidbody.RigidBody(inv_mass=0.0, collision_shape=sphere)
sphere_id.world_pose.position = np.array([0., 0., 1.])
physics_world.bodies.append(sphere_id)

#rendering sphere
if use_viz:
  sphere = g.Sphere([RADIUS])
  vis['sphere_fixed'].set_object(sphere,g.MeshPhongMaterial(color=0x55ffff, wireframe=True))
  
  mat4 = sphere_id.world_pose.matrix()
  vis["sphere_fixed"].set_transform(mat4)


#physics sphere
sphere = geometry.Sphere(RADIUS)
sphere_id = rigidbody.RigidBody(inv_mass=1.0, collision_shape=sphere)
sphere_id.world_pose.position = np.array([0., 0., 2])
physics_world.bodies.append(sphere_id)

if use_viz:
  #rendering sphere
  sphere = g.Sphere([RADIUS])
  vis['sphere'].set_object(sphere,g.MeshPhongMaterial(color=0x5555ff, wireframe=True))



dt = SIMULATION_TIME_STEP
#todo: convert the sphere orientation quaternion to mat3x3
mat4 = tf.rotation_matrix(0, [0, 0, 1])
mat4[:3, 3] = sphere_id.world_pose.position
if use_viz:
  vis['sphere'].set_transform(mat4)

#real-time updates are a bit choppy, so record an animation instead 
#for _ in range(200):
#  physics_world.step(dt)
#  mat4[:3, 3] = sphere_id.world_pose.position
#  vis['sphere'].set_transform(mat4)
#  time.sleep(0.5*SIMULATION_TIME_STEP)
if use_viz:
  from meshcat.animation import Animation
  import meshcat.transformations as tf

sphere_id.world_pose.position = np.array([0., 0., 2])

if use_viz:
  anim = Animation()

for frame_index in range(1):
  physics_world.step(dt)
  mat4 = sphere_id.world_pose.matrix()
  if use_viz:
    with anim.at_frame(vis, frame_index) as frame:
      frame["sphere"].set_transform(mat4)
      
# `set_animation` actually sends the animation to the
# viewer. By default, the viewer will play the animation
# right away. To avoid that, you can also pass `play=False`. 
if use_viz:
  vis.set_animation(anim)#, play=False)
  time.sleep(10)