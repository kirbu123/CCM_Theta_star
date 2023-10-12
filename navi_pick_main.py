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
import threading
import time

import carb
from omni.isaac.core import World
from omni.isaac.core.utils import extensions

from src.actions.base_server import ActionServer
from src.actions.controller import HuskyController
from src.actions.scenario import add_scenario_to_queue, setup_scene_for_scenario
from src.config import get_config
from src.scene_setup.environment import setup_scene_background

# from omni.isaac.core.utils.stage import get_current_stage


cfg = get_config()

# enable ROS bridge extension
extensions.enable_extension(cfg.ros_cfg[cfg.ros].ros_bridge_extension)

if cfg.ros_cfg[cfg.ros].ros_v == 1:
    import rosgraph

    if not rosgraph.is_master_online():
        carb.log_error("Please run roscore before executing this script")
        simulation_app.close()
        exit()

if cfg.mode == "online":
    from src.actions.action_servers import (
        MoveToActionServer,
        OpenSeedActionServer,
        PickupObjectActionServer,
        PutObjectActionServer,
    )

    if cfg.ros == "noetic":
        import rospy


my_world = World(stage_units_in_meters=1.0)

setup_scene_background(simulation_app=simulation_app, cfg=cfg)

controller = HuskyController(cfg=cfg, world=my_world, simulation_app=simulation_app)

# Need to initialize physics getting any articulation..etc
my_world.initialize_physics()

task_queue = queue.Queue()
action_servers = {}

if cfg.mode == "idle":
    pass
elif cfg.mode == "online":

    def server_thread():
        rospy.spin()  # This will block and process ROS callbacks

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

elif cfg.mode == "offline":
    # Fake servers to adhere to a single payline
    action_servers["move_to_location"] = ActionServer(
        server_name="move_to_location",
    )
    action_servers["pick_up_object"] = ActionServer(
        server_name="pick_up_object",
    )
    action_servers["put_object"] = ActionServer(
        server_name="put_object",
    )
    action_servers["open_seed_setter"] = ActionServer(
        server_name="text_query_generation",
    )

    # Add scenario to queue
    add_scenario_to_queue(cfg, task_queue)
    setup_scene_for_scenario(cfg, my_world)
else:
    raise ValueError(f"Unknown mode: {cfg.mode}")

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
