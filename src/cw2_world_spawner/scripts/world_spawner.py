#!/usr/bin/python3

"""
This code contains the world spawner. This file accomplishes two main goals:

  1. Create and manage the objects in the gazebo world using the World() class
  2. Spawn and monitor the completion of coursework tasks using the Task() class

The coursework contains three tasks, and each of them are defined here. There
are three classes derived from the Task() base class, Task1(), Task2(), and
Task3().

Each of these TaskX() classes defines what objects will spawn for a given task,
as well as sending the service request for taskX_start() - this is the request
that you will receive and respond to (by solving the task).

Currently, tasks may spawn with random box positions and colours. If you wish to
change this, you can edit the task parameters:

  1. Task parameters defined as globals (IN UPPER CASE) at the top of this file.
     Edit these to adjust the tasks, this can help you during testing.

  2. Each TaskX() class has a method 'spawn_task_objects' which sets up the task,
     you can edit this function as well to change tasks during testing.

IMPORTANT! You will NOT submit your version of this file. When we mark your work,
we will use a clean version of this file.

During our tests, we may adjust the tasks to look at edge cases. The following
parameters may change:
  - T1_ANY_ORIENTATION
  - T1_GROUND_PLANE_NOISE
  - T1_USE_MULTIPLE_SIZES
  - T2_ANY_ORIENTATION
  - T2_GROUND_PLANE_NOISE
  - T2_USE_MULTIPLE_SIZES
  - T3_N_OBSTACLES
  - T3_ANY_ORIENTATION
  - T3_GROUND_PLANE_NOISE
  - T3_USE_MULTIPLE_SIZES
  - POSSIBLE_SHAPES*

*there may be new, unseen shapes. However, these new shapes will always have a
certain features which are detailed in the coursework pdf document.

No other parameters will change.

Best of luck!
"""

import rospy
from geometry_msgs.msg import Point, PointStamped, PoseStamped
from cw2_world_spawner_lib.coursework_world_spawner import *
from cw2_world_spawner.srv import TaskSetup, TaskSetupResponse
from cw2_world_spawner.srv import Task1Service, Task2Service, Task3Service
import numpy as np

# # hint: an easy way to 'disable' the randomness in the task spawning is:
# myseed = 0
# np.random.seed(myseed) # choose any int as your seed

# ----- key coursework task parameters ----- #

# ----- key coursework task parameters ----- #

# task 1 parameters                 
T1_SHAPE_X_LIMS = [0.40, 0.55]           # xrange a shape can spawn
T1_SHAPE_Y_LIMS = [-0.40, 0.40]          # yrange a shape can spawn
T1_ANY_ORIENTATION = True               # do we allow any rotation of a shape
T1_GROUND_PLANE_NOISE = 0e-3             # do we add noise on the z height of the green tiles
T1_USE_MULTIPLE_SIZES = False            # do we spawn objects with varying sizes

# task 2 parameters
T2_SHAPE_X_LIMS = [0.40, 0.55]           # xrange a shape can spawn
T2_SHAPE_Y_LIMS = [-0.40, 0.40]          # yrange a shape can spawn
T2_N_REF_SHAPES = 2                      # number of baskets to spawn
T2_OBJECT_REF_POINTS = [(-0.43, -0.4), 
                        (-0.43,  0.4)]
T2_ANY_ORIENTATION = True               # do we allow any rotation of a shape
T2_GROUND_PLANE_NOISE = 50e-3             # do we add noise on the z height of the green tiles
T2_USE_MULTIPLE_SIZES = False            # do we spawn objects with varying sizes

# task 3 parameters
T3_MAX_SHAPES = 7                        # maximum number of spawned shapes
T3_SHAPE_X_LIMS = [-0.6, 0.7]            # xrange a shape can spawn
T3_SHAPE_Y_LIMS = [-0.55, 0.55]          # yrange a shape can spawn
T3_N_OBSTACLES = 3
T3_ANY_ORIENTATION = True               # do we allow any rotation of a shape
T3_GROUND_PLANE_NOISE = 0e-3             # do we add noise on the z height of the green tiles
T3_USE_MULTIPLE_SIZES = True            # do we spawn objects with varying sizes

# possible goal basket locations (x, y)
BASKET_LOCATIONS = [(-0.41, -0.36), 
                    (-0.41,  0.36)]

# define the variety of spawned objects, these values are defined in .sdf files
POSSIBLE_SHAPES = ["nought", "cross"]
POSSIBLE_SIZES = ["40", "30", "20"]
POSSIBLE_COLOURS = {'purple': [0.8, 0.1, 0.8],
                    'red':    [0.8, 0.1, 0.1], 
                    'blue':   [0.1, 0.1, 0.8]}
POSSIBLE_OBSTACLES = [
  ["obstacle_1", 80e-3],
  ["obstacle_2", 50e-3],
  ["obstacle_3", 60e-3],
  ["obstacle_4", 100e-3],
]

# ----- Class definitions for base world and task ----- #

class World(object):

  # lengths in metres for world objects, resultant from sdf model files, don't change
  tile_side_length = 100e-3
  tile_thickness = 20e-3
  basket_side_length = 350e-3
  basket_height = 50e-3
  robot_safety_radius = 250e-3

  def __init__(self):
    """
    World managing class for the coursework
    """
    world_spawner.despawn_all(exceptions="object-golf-tile")
    self.spawn_tiles()

  def spawn_tiles(self, tile_height=0):
    """
    Spawn all of the green tiles around the panda robot
    """

    # spawn in all of the tiles, xyz already input in model.sdf so use (0, 0, 0)
    model = Model(model_name = "all_tiles",
                  instance_name = 'object-all-golf-tiles',
                  model_type = 'sdf',
                  position = [0, 0, tile_height]
            )
    world_spawner.spawn(model)

    return

  def reset(self):
    world_spawner.despawn_all()
    return

class Task(object):

  def __init__(self, mode='coursework', validation_scenario=0):
    """
    Task base class with methods to spawn objects and reset
    """

    self.models = []
    self.ground_level = world.tile_thickness

    if mode == 'coursework':
      self.spawn_task_objects()
      self.begin_task()
    else:
      self.spawn_test_objects(validation_scenario)
      self.begin_test(validation_scenario)

    return 

  def spawn_model(self, name, point=None, spawn_height=None,
                   xlims = [0.3,0.5], ylims = [-0.12,0.12],
                   rotationlims = [0, 0]):
    """
    Spawn a model
    """

    model_name = name
    instance_name = name + f"_object_{len(self.models) + 1}"

    if spawn_height is None: 
      zlims = [self.ground_level + 5e-3, self.ground_level + 5e-3]
    else:
      zlims = [spawn_height, spawn_height]

    if point is None:
      position = random_position_in_area(xlims, ylims, zlims)
    else:
      position = [point[0], point[1], zlims[0]]

    # create and spawn the model
    model = Model(model_name = model_name,
                  instance_name = instance_name,
                  model_type = 'sdf',
                  position = position,
                  orientation = random_orientation(rotationlims, [0, 0], [0, 0])
            )
    world_spawner.spawn(model)

    # save the spawned model
    self.models.append(model)

    return model

  def prepare_for_task_request(self, service_name, timeout=60):
    """
    Search for the required task solving service to be advertised
    """
    rospy.logdebug(f"Attempting to connect to {service_name} Service...")
    try:
      rospy.wait_for_service(service_name, timeout=timeout)
    except rospy.ROSException as e:
      rospy.logwarn(f"Error in {service_name} request:", e)
      rospy.logdebug(f"{service_name} Request failed - not advertised")
      return False
    return True

  def reset_task(self, respawn_tiles=False, tile_height=0):
    """
    Remove any objects from the world, so baskets and cubes
    """
    world_spawner.despawn_all(keyword='object', exceptions="tile")
    if respawn_tiles:
      world_spawner.despawn_all(keyword="tile")
      world.spawn_tiles(tile_height=tile_height)
      self.ground_level = world.tile_thickness + tile_height
    self.models = []
    return

  def get_position_from_point(self,pt):
    return np.asarray([pt.x, pt.y, pt.z])

  def get_position_from_point_stamped(self,ptst):
    pt = ptst.point
    return np.asarray([pt.x, pt.y, pt.z])

  def get_position_from_pose(self,pose):
    pos_np = self.get_position_from_point(pose.position)
    return pos_np

  def get_euclidean_distance(self,a,b):
    return np.sqrt(np.sum(np.power(a - b,2)))

  def spawn_task_objects():
    raise NotImplementedError
  
  def begin_task(self):
    raise NotImplementedError
  
  def spawn_test_objects(self, validation_scenario):
    raise NotImplementedError
  
  def begin_test(self, validation_scenario):
    raise NotImplementedError
  
# ----- Class definitions for the coursework tasks ----- #

class Task1(Task):

  # name of service which is requested to solve the task
  service_to_request = "/task1_start"

  def __init__(self, mode='coursework', validation_scenario=0):
    """
    Task class, initialise either in 'coursework' mode (normal, no validation
    scenario needed), or initialise in 'validation' mode and pass in a chosen
    validation scenario number.
    """
    rospy.loginfo('================Starting Task1==============')
    Task.__init__(self, mode, validation_scenario)

  def spawn_task_objects(self):
    """
    Spawns the objects for the task, feel free to edit
    """

    if T1_GROUND_PLANE_NOISE < 1e-3:
      self.reset_task()
    else:
      tile_height = np.random.random() * (T1_GROUND_PLANE_NOISE)
      self.reset_task(respawn_tiles=True, tile_height=tile_height)

    # spawn a random object
    colour_options = list(POSSIBLE_COLOURS.keys())
    rand_colour = np.random.choice(colour_options)
    rand_object = "nought"
    if T1_USE_MULTIPLE_SIZES:
      rand_size = np.random.choice(POSSIBLE_SIZES)
    else: rand_size = POSSIBLE_SIZES[0]
    random_object = rand_object + "_" + rand_colour + "_" + rand_size + "mm"
    rot_lims = [0, 2*np.math.pi] if T1_ANY_ORIENTATION else [0, 0]
    self.spawn_model(name=random_object, rotationlims=rot_lims, xlims=T1_SHAPE_X_LIMS,
                     ylims=T1_SHAPE_Y_LIMS)

    # save the name of the object
    # Temporarily set to nought, for testing
    self.object_type = "nought"
    
    # spawn a goal basket
    random_goal = BASKET_LOCATIONS[np.random.random_integers(0, len(BASKET_LOCATIONS)) - 1]
    self.spawn_model(name="basket", point=random_goal)

    return

  def send_task1_request(self, object_point, goal_point):
    """
    Sends out a service request that this task get solved
    """

    rospy.logdebug("Task1 Service connected. Sending request...")
    task1srv = rospy.ServiceProxy(self.service_to_request, Task1Service)
    obj_pt = PointStamped()
    obj_pt.point = object_point
    obj_pt.header.frame_id = "panda_link0"
    obj_pt.header.stamp = rospy.Time.now()
    goal_pt = PointStamped()
    goal_pt.point = goal_point
    goal_pt.header.frame_id = "panda_link0"
    goal_pt.header.stamp = rospy.Time.now()
    _ = task1srv(obj_pt, goal_pt, self.object_type)

    return True

  def begin_task(self):
    """
    Start the task, check a service is available to solve the task
    """

    success = self.prepare_for_task_request(self.service_to_request)
    rospy.sleep(rospy.Duration(1))

    object_point = self.models[0].get_model_state().pose.position
    goal_point = self.models[1].get_model_state().pose.position
    if success: resp = self.send_task1_request(object_point, goal_point)
    else: rospy.logerr("Task Request failed - not advertised")

    return

class Task2(Task):

  # name of service which is requested to solve the task
  service_to_request = "/task2_start"

  def __init__(self, mode='coursework', validation_scenario=0):
    """
    Task class, initialise either in 'coursework' mode (normal, no validation
    scenario needed), or initialise in 'validation' mode and pass in a chosen
    validation scenario number.
    """
    rospy.loginfo('================Starting Task2==============')
    Task.__init__(self,mode, validation_scenario)

  def spawn_task_objects(self):
    """
    Spawns the objects for the task, feel free to edit
    """

    if T2_GROUND_PLANE_NOISE < 1e-3:
      self.reset_task()
    else:
      tile_height = np.random.random() * (T2_GROUND_PLANE_NOISE)
      self.reset_task(respawn_tiles=True, tile_height=tile_height)

    colour_options = list(POSSIBLE_COLOURS.keys())
    shape_options = POSSIBLE_SHAPES
    n_ref_shapes = T2_N_REF_SHAPES
    rot_lims = [0, 2*np.math.pi] if T2_ANY_ORIENTATION else [0, 0]

    # randomise shapes
    rand_shapes = np.random.permutation(shape_options)

    # check we have enough possible shapes to use as references
    if n_ref_shapes > len(shape_options):
      rospy.logwarn("T2_N_REF_SHAPES is greater than number of possible shapes")
      n_ref_shapes = len(shape_options) # local change only
    if n_ref_shapes > len(T2_OBJECT_REF_POINTS):
      rospy.logwarn("T2_N_REF_SHAPES is greater than number of reference positions")
      n_ref_shapes = len(T2_OBJECT_REF_POINTS) # local change only

    # clip our random shapes by the number of references
    rand_shapes = rand_shapes[:n_ref_shapes]

    # loop and spawn our references shapes
    for i in range(n_ref_shapes):

      rand_colour = np.random.choice(colour_options)
      rand_object = rand_shapes[i]
      if T2_USE_MULTIPLE_SIZES:
        rand_size = np.random.choice(POSSIBLE_SIZES)
      else: rand_size = POSSIBLE_SIZES[0]
      random_ref = rand_object + "_" + rand_colour + "_" + rand_size + "mm"
      self.spawn_model(name=random_ref, point=T2_OBJECT_REF_POINTS[i], rotationlims=rot_lims)

    # spawn the mystery object
    rand_colour = np.random.choice(colour_options)
    rand_object = np.random.choice(rand_shapes)
    if T2_USE_MULTIPLE_SIZES:
      rand_size = np.random.choice(POSSIBLE_SIZES)
    else: rand_size = POSSIBLE_SIZES[0]
    random_object = rand_object + "_" + rand_colour + "_" + rand_size + "mm"
    self.spawn_model(name=random_object, rotationlims=rot_lims, xlims=T2_SHAPE_X_LIMS,
                     ylims=T2_SHAPE_Y_LIMS)

    return

  def send_task2_request(self, ref_points, mystery_point):
    """
    Sends out a service request that this task get solved
    """

    rospy.logdebug("Task2 Service connected. Sending request...")
    task2srv = rospy.ServiceProxy(self.service_to_request, Task2Service)
    resp = task2srv(ref_points, mystery_point)

    return resp

  def begin_task(self):
    """
    Start the task, check a service is available to solve the task and then call
    that service
    """

    success = self.prepare_for_task_request(self.service_to_request)

    # get reference points
    ref_points = []
    for i in range(len(self.models) - 1):
      point_st = PointStamped()
      point_st.point = self.models[i].get_model_state().pose.position
      point_st.header.frame_id = "panda_link0"
      point_st.header.stamp = rospy.Time.now()
      ref_points.append(point_st)
    
    mystery_point = PointStamped()
    mystery_point.point = self.models[-1].get_model_state().pose.position
    mystery_point.header.frame_id = "panda_link0"
    mystery_point.header.stamp = rospy.Time.now()

    if success: resp = self.send_task2_request(ref_points, mystery_point)
    else: rospy.logerr("Task Request failed - not advertised")

    return

class Task3(Task):

  # name of service which is requested to solve the task
  service_to_request = "/task3_start"

  def __init__(self, mode='coursework', validation_scenario=0):
    """
    Task class, initialise either in 'coursework' mode (normal, no validation
    scenario needed), or initialise in 'validation' mode and pass in a chosen
    validation scenario number.
    """
    rospy.loginfo('================Starting Task3==============')
    Task.__init__(self, mode, validation_scenario)

  def find_empty_point(self, mysize, xlims, ylims, point_size_pairs):
    """
    Returns a suitable empty point given an area=[(x0, y0), (x1, y1)] and some
    points each being [(x, y), size]
    """

    grid_index = 0

    # remove references
    xlims = xlims[:]
    ylims = ylims[:]

    # clip the area down by the object size
    radius = mysize * (1.0 / np.math.sqrt(2))
    xlims[0] += radius
    xlims[1] -= radius
    ylims[0] += radius
    ylims[1] -= radius

    # map out all possible random points in the area
    x = np.arange(xlims[0], xlims[1], 2e-3)
    y = np.arange(ylims[0], ylims[1], 2e-3)
    X_grid, Y_grid = np.meshgrid(x, y)
    points_grid = np.dstack((X_grid, Y_grid))
    points_line = np.reshape(points_grid, (-1, 2))
    randomised_points = np.random.permutation(points_line)

    while grid_index < len(randomised_points):
      
      good_point = True

      rand_x = randomised_points[grid_index][0]
      rand_y = randomised_points[grid_index][1]
      grid_index += 1

      # loop over our existing points to see if our sample is allowed
      for ((x, y), prev_size) in point_size_pairs:

        radius = (prev_size + mysize * 1.1) / np.math.sqrt(2)

        this_xlims = [x - radius, x + radius]
        this_ylims = [y - radius, y + radius]

        # is our random point within this area?]
        if not (rand_x > this_xlims[0] and rand_x < this_xlims[1] and
                rand_y > this_ylims[0] and rand_y < this_ylims[1]):
          good_point = True

        else:
          good_point = False
          break

      # if we found a valid point
      if good_point: return (rand_x, rand_y)

    # no valide point found within our search
    return None

  def spawn_task_objects(self):
    """
    Spawns the objects for the task, feel free to edit
    """

    if T3_GROUND_PLANE_NOISE < 1e-3:
      self.reset_task()
    else:
      tile_height = np.random.random() * (T3_GROUND_PLANE_NOISE)
      self.reset_task(respawn_tiles=True, tile_height=tile_height)

    colour_options = list(POSSIBLE_COLOURS.keys())
    rot_lims = [0, 2*np.math.pi] if T3_ANY_ORIENTATION else [0, 0]
    n_objects = np.random.random_integers(T3_MAX_SHAPES // 2, T3_MAX_SHAPES)

    # spawn a goal basket
    random_goal = BASKET_LOCATIONS[np.random.random_integers(0, len(BASKET_LOCATIONS)) - 1]
    self.spawn_model(name="basket", point=random_goal)

    # record where the panda and basket are so we don't spawn objects on them
    panda = [(0, 0), world.robot_safety_radius * np.math.sqrt(2)]
    basket = [
      (self.models[0].get_model_state().pose.position.x, 
      self.models[0].get_model_state().pose.position.y),
      0.5 * world.basket_side_length * np.math.sqrt(2)
    ]
    self.spawned_points = [panda, basket]

    # spawn a number of obstacles
    for i in range(T3_N_OBSTACLES):

      random_obstacle = POSSIBLE_OBSTACLES[np.random.random_integers(0, len(POSSIBLE_OBSTACLES)) - 1]
      obstacle_size = 200e-3
      obstacle_name = random_obstacle[0]
      obstacle_spawn_height = random_obstacle[1] + self.ground_level + 10e-3
      spawn_point = self.find_empty_point(obstacle_size, T3_SHAPE_X_LIMS, 
                                          T3_SHAPE_Y_LIMS, self.spawned_points)
      if spawn_point is not None:
        self.spawn_model(name=obstacle_name, point=spawn_point, rotationlims=rot_lims,
                         spawn_height=obstacle_spawn_height)
        self.spawned_points.append([spawn_point, obstacle_size])
      else: break

    # spawn a number of random objects in empty gaps
    for i in range(n_objects):

      # spawn a random object
      rand_colour = np.random.choice(colour_options)
      rand_object = np.random.choice(POSSIBLE_SHAPES)
      if T3_USE_MULTIPLE_SIZES:
        rand_size = np.random.choice(POSSIBLE_SIZES)
      else: rand_size = POSSIBLE_SIZES[0]
      random_object = rand_object + "_" + rand_colour + "_" + rand_size + "mm"
      random_size = int(rand_size) * 5 * 1e-3
      spawn_point = self.find_empty_point(random_size, T3_SHAPE_X_LIMS, 
                                          T3_SHAPE_Y_LIMS, self.spawned_points)
      if spawn_point is not None:
        self.spawn_model(name=random_object, point=spawn_point, rotationlims=rot_lims)
        self.spawned_points.append([spawn_point, random_size])
      else: break

    return

  def send_task3_request(self):
    """
    Sends out a service request that this task get solved
    """

    # Send request
    rospy.logdebug("Task3 Service connected. Sending request...")
    task3srv = rospy.ServiceProxy(self.service_to_request, Task3Service)
    resp = task3srv()

    return resp
  
  def begin_task(self):
    """
    Start the task, check a service is available to solve the task and then call
    that service
    """

    # Prepare and send request
    success = self.prepare_for_task_request(self.service_to_request)
    if success: resp = self.send_task3_request()
    else: rospy.logerr("Task Request failed - not advertised")

    return

# ----- Running the coursework node ----- #

def handle_task_request(req):
  """
  Helper function to handle requesting tasks
  """

  # Callback for selecting which task to start
  if req.task_index == 1:
    Task1(mode="coursework")

  elif req.task_index == 2:
    Task2(mode="coursework")

  elif req.task_index == 3:
    Task3(mode="coursework")

  else: rospy.logwarn("Unrecognized task requested")

  return TaskSetupResponse()

# globals for interacting with the world
world_spawner = WorldSpawner()
world = World()

if __name__ == "__main__":

  # create the world and run the coursework /task service
  rospy.init_node('coursework3_wrapper')

  # create the /task service callback
  rospy.Service('/task', TaskSetup, handle_task_request)
  rospy.loginfo("Ready to initiate task.")
  rospy.loginfo("Use rosservice call /task <INDEX> to start a task")

  # find the path to the models folder and print a helpful warning
  import rospkg, os
  rospack = rospkg.RosPack()
  rospath = rospack.get_path("cw2_world_spawner")
  pkg_models_path = rospath + "/models"
  export_cmd = f"export GAZEBO_MODEL_PATH={pkg_models_path}"
  rospy.logwarn(f"If Gazebo freezes when you call a task you need to run in your terminal the following: {export_cmd}")


  rospy.spin()
