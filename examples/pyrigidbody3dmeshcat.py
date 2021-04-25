
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

SIMULATION_TIME_STEP = 1. / 60.#240.
NUM_SOLVER_ITERATIONS = 20
RADIUS=0.5

physics_world = world.World(NUM_SOLVER_ITERATIONS)
vis = meshcat.Visualizer().open()

#physics plane
plane = geometry.Plane()
plane_id = rigidbody.RigidBody(inv_mass=0.0, collision_shape=plane)
physics_world.bodies.append(plane_id)

#rendering plane
ground = g.Box([10,10,0.01])
vis['ground'].set_object(ground,g.MeshLambertMaterial( color=0xffffff, wireframe=False))

#physics sphere
sphere = geometry.Sphere(RADIUS)
sphere_id = rigidbody.RigidBody(inv_mass=1.0, collision_shape=sphere)
sphere_id.world_pose.position = np.array([0., 0., 2.6])
physics_world.bodies.append(sphere_id)

#rendering sphere
sphere = g.Sphere([RADIUS])
vis['sphere'].set_object(sphere,g.MeshPhongMaterial(color=0x5555ff, wireframe=True))


dt = SIMULATION_TIME_STEP
#todo: convert the sphere orientation quaternion to mat3x3
mat4 = tf.rotation_matrix(0, [0, 0, 1])
mat4[:3, 3] = sphere_id.world_pose.position
vis['sphere'].set_transform(mat4)

#real-time updates are a bit choppy, so record an animation instead 
#for _ in range(200):
#  physics_world.step(dt)
#  mat4[:3, 3] = sphere_id.world_pose.position
#  vis['sphere'].set_transform(mat4)
#  time.sleep(0.5*SIMULATION_TIME_STEP)

from meshcat.animation import Animation
import meshcat.transformations as tf

sphere_id.world_pose.position = np.array([0., 0., 2.6])

anim = Animation()

for frame_index in range(200):
  physics_world.step(dt)
  mat4[:3, 3] = sphere_id.world_pose.position
  with anim.at_frame(vis, frame_index) as frame:
    frame["sphere"].set_transform(mat4)
    
# `set_animation` actually sends the animation to the
# viewer. By default, the viewer will play the animation
# right away. To avoid that, you can also pass `play=False`. 
vis.set_animation(anim)#, play=False)
