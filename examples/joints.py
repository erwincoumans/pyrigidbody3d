
import sys
import copy
import numpy as np
from pyrigidbody3d import geometry
from pyrigidbody3d import rigidbody
from pyrigidbody3d import world
from pyrigidbody3d import joint

# real-time updates are a bit choppy
import meshcat
import meshcat.geometry as g
import meshcat.transformations as tf
import math
import time

SIMULATION_TIME_STEP = 1. / 60.#240.
NUM_SOLVER_ITERATIONS = 10 
RADIUS=0.5

physics_world = world.World(NUM_SOLVER_ITERATIONS)
vis = meshcat.Visualizer().open()

#physics plane
plane = geometry.Plane()
plane_id = rigidbody.RigidBody(inv_mass=0.0, collision_shape=plane)
physics_world.bodies.append(plane_id)

#rendering plane
ground = g.Box([40,40,0.01])
vis['ground'].set_object(ground,g.MeshLambertMaterial( color=0xffffff, wireframe=False))

dt = SIMULATION_TIME_STEP
spheres=[]
#physics sphere
prev_sphere = None

num_spheres=6
for i in range (num_spheres):
  sphere = geometry.Sphere(RADIUS)
  mass = 0
  if i > 0:
    mass = 1
  sphere_id = rigidbody.RigidBody(inv_mass=mass, collision_shape=sphere)

  if i>0:
    j = joint.SphericalJoint([0,0.5,0],[0,-0.5,0])
    j.body_a = prev_sphere
    j.body_b = sphere_id
    physics_world.spherical_joints.append(j)
  
  prev_sphere = sphere_id
  sphere_id.world_pose.position = np.array([0., i, 4.6])
  physics_world.bodies.append(sphere_id)

  #rendering sphere
  sphere = g.Sphere([RADIUS])
  name = 'sphere'+str(i)
  vis[name].set_object(sphere,g.MeshPhongMaterial(color=0x5555ff, wireframe=True))

  #todo: convert the sphere orientation quaternion to mat3x3
  #mat4 = tf.rotation_matrix(0, [0, 0, 1])
  #mat4[:3, 3] = sphere_id.world_pose.position
  mat4 = sphere_id.world_pose.matrix()
  vis[name].set_transform(mat4)
  spheres.append(sphere_id)

from meshcat.animation import Animation
import meshcat.transformations as tf

anim = Animation()

#for frame_index in range(500):
while 1:
  physics_world.step(dt)
  for i in range (num_spheres):
    sphere_id = spheres[i]
    name = 'sphere'+str(i)
    mat4 = sphere_id.world_pose.matrix()
    #with anim.at_frame(vis, frame_index) as frame:
    #  frame[name].set_transform(mat4)
    vis[name].set_transform(mat4)
# `set_animation` actually sends the animation to the
# viewer. By default, the viewer will play the animation
# right away. To avoid that, you can also pass `play=False`. 
#vis.set_animation(anim)#, play=False)
