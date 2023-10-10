#!/usr/bin/env python3
# Copyright (c) 2022, NVIDIA CORPORATION.  All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto.  Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.
#

# This import should be there according to examples
from omni.isaac.kit import SimulationApp

simulation_app = SimulationApp({"headless": False, "renderer": "RayTracedLighting"})

import queue
import sys
import threading
import time

import carb
import numpy as np
import rospy
from omni.isaac.core import World
from omni.isaac.core.materials import PreviewSurface
from omni.isaac.core.objects import DynamicCuboid
from omni.isaac.core.utils import extensions, nucleus, stage
from omni.isaac.core.utils.prims import is_prim_path_valid
from omni.isaac.core.utils.stage import get_current_stage
from omni.isaac.core.utils.string import find_unique_string_name

from src.actions.actions import HuskyController
from src.actions.servers import (
    MoveToActionServer,
    OpenSeedActionServer,
    PickupObjectActionServer,
    PutObjectActionServer,
)
from src.config import get_config
from src.sensors.cameras import setup_cameras
from src.sensors.imu import setup_imu_graph
from src.sensors.lidar import setup_lidar_graph
from src.sensors.tf import setup_tf_graph

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
        color = np.array([1.0, 0, 0])
        goal = world.scene.add(
            DynamicCuboid(
                prim_path=f"/World/goal_cube_{i}",  # The prim path of the cube in the USD stage
                name=f"goal_cube_{i}",  # The unique name used to retrieve the object from the scene later on
                position=np.array(
                    [x, y, 0.1]
                ),  # Using the current stage units which is in meters by default.
                scale=np.array([0.07, 0.07, 0.07]),  # most arguments accept mainly numpy arrays.
                color=color,  # RGB channels, going from 0-1
            )
        )
        scene[f"goal_{i}"] = goal

    complete_goal_prim_path = find_unique_string_name(
        initial_name="/World/Looks/complete_goal_material", is_unique_fn=lambda x: not is_prim_path_valid(x)
    )
    scene["complete_goal_material"] = PreviewSurface(
        prim_path=complete_goal_prim_path, color=np.array([0.0, 1.0, 0])
    )

    complete_goal_prim_path = find_unique_string_name(
        initial_name="/World/Looks/next_goal_material", is_unique_fn=lambda x: not is_prim_path_valid(x)
    )
    scene["next_goal_material"] = PreviewSurface(
        prim_path=complete_goal_prim_path, color=np.array([1.0, 1.0, 0])
    )

    world.reset()


my_world = World(stage_units_in_meters=1.0)


# Locate Isaac Sim assets folder to load environment and robot stages
assets_root_path = nucleus.get_assets_root_path()  # !
if assets_root_path is None:
    carb.log_error("Could not find Isaac Sim assets folder")
    simulation_app.close()
    sys.exit()

# Loading the simple_room environment
if cfg.use_background:
    if cfg.use_omni_background:
        stage.add_reference_to_stage(
            assets_root_path + cfg.background_usd_path, cfg.background_stage_path
        )  # !
    else:
        stage.add_reference_to_stage(cfg.background_usd_path, cfg.background_stage_path)
else:
    stage.add_reference_to_stage(assets_root_path + "/Isaac/Environments/Grid/default_environment.usd")

navigation_finish = False


controller = HuskyController(cfg=cfg, world=my_world)

# ? set up cameras
our_stage = get_current_stage()

simulation_app.update()
setup_cameras(cfg, simulation_app, our_stage)  # , ["zed", "realsense_rear", "realsense_front"]

setup_tf_graph(cfg, simulation_app, our_stage)
setup_lidar_graph(cfg, simulation_app, our_stage)
setup_imu_graph(cfg, simulation_app, our_stage)

# Need to initialize physics getting any articulation..etc
my_world.initialize_physics()

task_queue = queue.Queue()
action_servers = {}


def server_thread():
    rospy.spin()  # This will block and process ROS callbacks


# for i in range(3):  # Let's say we have 3 servers
#     t = threading.Thread(target=server_thread, args=(i,))
#     t.start()

rospy.init_node("HuskyActionServer", anonymous=True)

action_servers["move_to_location"] = MoveToActionServer(
    server_name="move_to_location",
    task_queue=task_queue,
)
action_servers["pick_up_object"] = PickupObjectActionServer(
    server_name="pick_up_object",
    task_queue=task_queue,
)
action_servers["put_object"] = PutObjectActionServer(
    server_name="put_object",
    task_queue=task_queue,
)

action_servers["open_seed_setter"] = OpenSeedActionServer(
    server_name="text_query_generation",
    task_queue=task_queue,
)


t = threading.Thread(target=server_thread)
t.start()

while simulation_app.is_running():
    # print(f"Checking queue...")
    if not task_queue.empty():
        my_world.step(render=True)
        if my_world.is_playing():
            server_name, task = task_queue.get()
            print(f"Main thread processing: task: {task.__str__()} server_id: {server_name}")

            if server_name == "move_to_location":
                controller.move_to_location(task, action_servers[server_name])
                time.sleep(2)
                task_queue.task_done()

            elif server_name == "pick_up_object":
                controller.pickup_object(task, action_servers[server_name])
                time.sleep(2)
                task_queue.task_done()
            elif server_name == "put_object":
                controller.put_object(task, action_servers[server_name])
                time.sleep(2)
                task_queue.task_done()
            elif server_name == "text_query_generation":
                pass
            else:
                print(f"Unknown server name: {server_name}")
                print(f"Task: {task.__str__()}")
                continue
    else:
        print("Waiting for tasks... 1, 2, 3... 10")
        for step in range(10):
            my_world.step(render=True)
            time.sleep(0.5)


simulation_app.close()  # close Isaac Sim
