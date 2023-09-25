#!/usr/bin/env python3
# Copyright (c) 2022, NVIDIA CORPORATION.  All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto.  Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.
#

from omni.isaac.kit import SimulationApp

simulation_app = SimulationApp({"headless": False, "renderer": "RayTracedLighting"})

import sys
import os
import time
import numpy as np
import carb
import omni
import omni.graph.core as og
from omni.isaac.core import World
from omni.kit.viewport.utility import get_active_viewport
from omni.isaac.core.utils.prims import set_targets
from omni.isaac.core.utils import stage, extensions, nucleus
from omni.isaac.wheeled_robots.robots import WheeledRobot
from omni.isaac.motion_generation import WheelBasePoseController
from omni.isaac.wheeled_robots.controllers.differential_controller import DifferentialController
from omni.isaac.core.objects import DynamicCuboid
from omni.isaac.core.utils.string import find_unique_string_name
from omni.isaac.core.materials import PreviewSurface
from omni.isaac.core.utils.prims import get_prim_at_path, is_prim_path_valid
from omni.isaac.core.articulations import ArticulationSubset
from omni.isaac.core.utils.stage import get_current_stage
from omni.isaac.core.utils.stage import get_stage_units
from omni.isaac.core.utils.render_product import create_hydra_texture
import omni.replicator.core as rep
from pxr import UsdPhysics, Usd, Sdf, Gf, UsdGeom


from src.tasks.pick_place import PickPlace
from src.tasks.navigation import Navigate
from src.tasks.follow_target import FollowTarget
from src.controllers.rmpflow import RMPFlowController
from src.controllers.pick_place import PickPlaceController
from src.config import get_config, Config
from src.sensors.tf import setup_tf_graph
from src.sensors.cameras import setup_cameras_graph, setup_cameras
from src.sensors.lidar import setup_lidar_graph
from src.sensors.imu import setup_imu_graph

#!#######!#######!######
import rospy
import actionlib
import threading
import queue

from src.action_servers.servers import MoveToActionServer, PickupObjectActionServer, PutObjectActionServer, OpenSeedActionServer

from my_action_package.msg import SquareNumberAction, SquareNumberFeedback, SquareNumberResult
from communication_msgs.msg import TaskArray
from communication_msgs.msg import PickupObjectAction, PickupObjectResult
from communication_msgs.msg import PutObjectAction, PutObjectResult
from communication_msgs.msg import MoveToAction, MoveToResult, MoveToFeedback
from communication_msgs.msg import OpenSeeDSetterAction, OpenSeeDSetterGoal
#!#######!#######!######

cfg = get_config()

# enable ROS bridge extension
extensions.enable_extension(cfg.ros_cfg[cfg.ros].ros_bridge_extension)

if cfg.ros_cfg[cfg.ros].ros_v == 1:
    import rosgraph
    
    if not rosgraph.is_master_online():
        carb.log_error("Please run roscore before executing this script")
        simulation_app.close()
        exit()

def add_goals(world, scene, goals):
    for i, (x, y) in enumerate(goals):
         color = np.array([1., 0, 0])
         goal = world.scene.add(
            DynamicCuboid(
                prim_path=f"/World/goal_cube_{i}", # The prim path of the cube in the USD stage
                name=f"goal_cube_{i}", # The unique name used to retrieve the object from the scene later on
                position=np.array([x, y, 0.1]), # Using the current stage units which is in meters by default.
                scale=np.array([0.07, 0.07, 0.07]), # most arguments accept mainly numpy arrays.
                color=color, # RGB channels, going from 0-1
            ))
         scene[f'goal_{i}'] = goal

    complete_goal_prim_path = find_unique_string_name(
                    initial_name="/World/Looks/complete_goal_material", is_unique_fn=lambda x: not is_prim_path_valid(x)
                )
    scene['complete_goal_material'] = PreviewSurface(prim_path=complete_goal_prim_path, color=np.array([0., 1., 0]))

    complete_goal_prim_path = find_unique_string_name(
                    initial_name="/World/Looks/next_goal_material", is_unique_fn=lambda x: not is_prim_path_valid(x)
                )
    scene['next_goal_material'] = PreviewSurface(prim_path=complete_goal_prim_path, color=np.array([1., 1., 0]))

    world.reset()

###################################### Init Scene #############################################
my_world = World(stage_units_in_meters=1.0)


# Locate Isaac Sim assets folder to load environment and robot stages
assets_root_path = nucleus.get_assets_root_path() #!
if assets_root_path is None:
    carb.log_error("Could not find Isaac Sim assets folder")
    simulation_app.close()
    sys.exit()

# Loading the simple_room environment
if cfg.use_background:
    if cfg.use_omni_background:
        stage.add_reference_to_stage(assets_root_path + cfg.background_usd_path, cfg.background_stage_path) #!
    else:
        stage.add_reference_to_stage(cfg.background_usd_path, cfg.background_stage_path)
else:
    stage.add_reference_to_stage(assets_root_path + "/Isaac/Environments/Grid/default_environment.usd")

# traj=[[-2,-2]]
# traj=[[3,3], [3, -3], [-3, -3], [-3, 3], [0, 0]]

# traj_completed = False
navigation_finish = False

# Relative coordinates
#TODO Do we need this relative coordinates?
# cube_size = np.array([0.0515, 0.0515, 0.0515]) / get_stage_units()
# relative_cube_init_position = np.array([0.3, 0.3, 0.3]) / get_stage_units()
# relative_cube_init_orientation = np.array([1, 0, 0, 0])
# relative_target_position = np.array([-0.3, -0.3, 0]) / get_stage_units()
# relative_target_position[2] = cube_size[2] / 2.0

pick_task = PickPlace(name="denso_pick_place",
                      cfg=cfg,
                      )

my_world.add_task(pick_task)
my_world.reset()
my_denso = my_world.scene.get_object("my_ur5")

my_world.reset()
husky=pick_task.robots["husky"]

scene={}

add_goals(my_world, scene, cfg.trajectory)
my_world.reset()



####################################### Init Husky controller ################################
husky_controller = WheelBasePoseController(name="cool_controller",
                                           open_loop_wheel_controller=
                                           DifferentialController(name="simple_control",
                                           wheel_radius=cfg.wheel_radius,
                                           wheel_base=cfg.wheel_base),
                                           is_holonomic=False,)
articulation_controller = my_denso.get_articulation_controller()


##? set up cameras
our_stage = get_current_stage()

simulation_app.update()
setup_cameras(cfg, simulation_app, our_stage) #, ["zed", "realsense_rear", "realsense_front"]

setup_tf_graph(cfg, simulation_app, our_stage)
setup_lidar_graph(cfg, simulation_app, our_stage)
setup_imu_graph(cfg, simulation_app, our_stage)

# Need to initialize physics getting any articulation..etc
my_world.initialize_physics()

#!#######!#######!######
#!#######!#######!######
#!#######!#######!######



task_queue = queue.Queue()
action_servers = {}

def server_thread():
    rospy.spin()  # This will block and process ROS callbacks

class HuskyActionServer:
    def __init__(self, 
                 cfg: Config,
                 server_name: str,
                 task_queue: queue,
                ):
        self.action = cfg.action_servers[server_name].action
        self.result = cfg.action_servers[server_name].result
        self.feedback = cfg.action_servers[server_name].feedback
        self.server_name = server_name

        self.server = actionlib.SimpleActionServer(server_name, self.action, self.execute, False)
        self.server.start()
        self.task_queue = task_queue
        self.id = id

        if self.feedback:
            self.feedback_queue = queue.Queue()
        if self.result:
            self.result_queue = queue.Queue()

    def execute(self, goal):
        task = (self.server_name, goal) # server id, task
        print(f"Task added: (server_id, task) -- {task.__str__()}")
        self.task_queue.put(task)

        while self.result_queue.empty():
            rospy.sleep(0.2)
            if self.server.is_preempt_requested():
                rospy.loginfo(f'[{self.server_name}] The goal has been preempted')
                self.server.set_preempted()
                return
            feedback = self.feedback_queue.get()
            self.server.publish_feedback(feedback)
            self.feedback_queue.task_done()

        result = self.result_queue.get()
        rospy.loginfo(f'[{self.server_name}] Sending result:', result.__str__())
        self.server.set_succeeded(result)
        self.result_queue.task_done()


# for i in range(3):  # Let's say we have 3 servers
#     t = threading.Thread(target=server_thread, args=(i,))
#     t.start()

rospy.init_node(f'HuskyActionServer', anonymous=True)

action_servers['move_to'] = MoveToActionServer(server_name="move_to_location",
                                               task_queue=task_queue,
                                               )
action_servers['pickup_object'] = PickupObjectActionServer(server_name="pick_up_object",
                                                           task_queue=task_queue,
                                                           )
action_servers['put_object'] = PutObjectActionServer(server_name="put_object",
                                                     task_queue=task_queue,
                                                     )

action_servers['open_seed_setter'] = OpenSeedActionServer(server_name="text_query_generation",
                                                            task_queue=task_queue,
                                                            )


t = threading.Thread(target=server_thread)
t.start()

while simulation_app.is_running():
    print(f"Checking queue...")
    if not task_queue.empty():
        my_world.step(render=True)
        if my_world.is_playing():
            server_name, task = task_queue.get()
            print(f"Main thread processing: task: {task.__str__()} server_id: {server_name}")

            if server_name == "move_to_location":
                pass
            elif server_name == "pick_up_object":
                pass
            elif server_name == "put_object":
                pass
            elif server_name == "text_query_generation":
                pass
            else:
                print(f"Unknown server name: {server_name}")
                print(f"Task: {task.__str__()}")
                continue
        
            # # Simulate processing time
            # for i in range(5):
            #     rospy.sleep(0.5)

            #     # Sending feedback
            #     feedback = SquareNumberFeedback()
            #     feedback.current_step = i
            #     action_servers[server_id].feedback_queue.put(feedback)
        
            # # If you want to indicate task completion via result
            # result = SquareNumberResult()
            # result.squared_number = task * task
            # action_servers[server_id].result_queue.put(result)
            # print(f"Result sent: {result}")
        
            # task_queue.task_done()
    else:
        for step in range(10):
            my_world.step(render=True)
            time.sleep(0.5)
            print(f"Waiting for tasks... Step {step}")
        


simulation_app.close() # close Isaac Sim