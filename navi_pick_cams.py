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

##!
# import rospy
# import actionlib
# from my_action_package.msg import SquareNumberAction, SquareNumberFeedback, SquareNumberResult
##!

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


cfg = get_config()

# enable ROS bridge extension
extensions.enable_extension("omni.isaac.ros2_bridge-humble")

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
    stage.add_reference_to_stage(assets_root_path + "/Isaac/Environments/Grid/default_environment.usd", cfg.background_stage_path)

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
# zed_left_camera_prim =  UsdGeom.Camera(our_stage.GetPrimAtPath(cfg.cameras.zed.stage_path))
# # zed_left_camera_prim = stage.GetPrimAtPath('/World/Husky_Robot/fence_link/zed/husky_front_right')
# zed_left_camera_prim.GetHorizontalApertureAttr().Set(cfg.cameras.zed.horizontal_aperture)
# zed_left_camera_prim.GetVerticalApertureAttr().Set(cfg.cameras.zed.vertical_aperture)
# zed_left_camera_prim.GetProjectionAttr().Set(cfg.cameras.zed.projection_type)
# zed_left_camera_prim.GetFocalLengthAttr().Set(cfg.cameras.zed.focal_length)
# zed_left_camera_prim.GetFocusDistanceAttr().Set(400)

# camera_prim = UsdGeom.Camera(stage.DefinePrim(CAMERA_STAGE_PATH))
# xform_api = UsdGeom.XformCommonAPI(camera_prim)
# xform_api.SetTranslate(Gf.Vec3d(0.04676, 0.05, 0.05))
# xform_api.SetRotate((90, 0, 0), UsdGeom.XformCommonAPI.RotationOrderXYZ)
# camera_prim.GetHorizontalApertureAttr().Set(21)
# camera_prim.GetVerticalApertureAttr().Set(17)
# camera_prim.GetProjectionAttr().Set("perspective")
# camera_prim.GetFocalLengthAttr().Set(24)
# camera_prim.GetFocusDistanceAttr().Set(400)

simulation_app.update()
setup_cameras(cfg, simulation_app, our_stage) #, ["zed", "realsense_rear", "realsense_front"]

setup_tf_graph(cfg, simulation_app, our_stage)
setup_lidar_graph(cfg, simulation_app, our_stage)
setup_imu_graph(cfg, simulation_app, our_stage)

# Need to initialize physics getting any articulation..etc
my_world.initialize_physics()
# input()

while simulation_app.is_running():
    my_world.step(render=True)
    if my_world.is_playing():
        if not navigation_finish:
            for i, milestone in enumerate(cfg.trajectory):
                milestone = np.asarray(milestone)
                # print(f"\nCurr goal: {milestone}")
                distance = 100
                # while not np.allclose(husky.get_world_pose()[0][:2], milestone, atol=0.1):
                while not (distance < 0.7):
                    # print(f"World pose: {husky.get_world_pose()[0][:2]}\n")
                    # print(f"Distance: {distance}\n")
                    position, orientation = husky.get_world_pose()
                    # print(f"Curr step: {position}")
                    # print(f"Curr position: {position}")
                    # print(f"Curr orientation: {orientation}")
                    # print(f"Curr goal: {milestone}")
                    wheel_actions=husky_controller.forward(start_position=position,
                                                                        start_orientation=orientation,
                                                                        goal_position=milestone,
                                                                        lateral_velocity=cfg.lateral_velocity,
                                                                        position_tol = cfg.position_tol,)
                    # print(f"Wheel actions: {wheel_actions}")
                    wheel_actions.joint_velocities =np.tile(wheel_actions.joint_velocities, 2)
                    # print(f"Modified wheel actions: {wheel_actions}\n")
                    # print(f"Wheel number: {husky._num_wheel_dof} - Actions length: {wheel_actions.get_length()}")
                    husky.apply_wheel_actions(wheel_actions)
                    my_world.step(render=True)
                    distance = np.sum((husky.get_world_pose()[0][:2] - milestone)**2)  # Compute distance between husky and target

            end_position, end_orientation = husky.get_world_pose()
            # print("\n Navigation finish!\n Now Start grasp!! \n")
            navigation_finish = True 
            pick_up_start = False

        #! Very strange code part, is it really needed ?
        wheel_actions=husky_controller.forward(start_position=end_position,
                                                    start_orientation=end_orientation,
                                                    goal_position=end_position,
                                                    lateral_velocity=cfg.lateral_velocity,
                                                    position_tol = cfg.position_tol,)   
        wheel_actions.joint_velocities =np.tile(wheel_actions.joint_velocities, 2)
        # print(f"Modified wheel actions: {wheel_actions}\n")
        husky.apply_wheel_actions(wheel_actions)
        # if not pick_up_start:
            # print(f"Now the coordinates of Husky is : {husky.get_world_pose()[0][:2]}\n")
        #! ##########
        
        ############################################################################################################
        
        if not pick_up_start:
            print(f"Now let's pick up ! \n")
            pick_controller = PickPlaceController(name="controller",
                                        robot_articulation=my_denso, 
                                        gripper=my_denso.gripper,
                                        cfg=cfg,
                                        )
            task_params = my_world.get_task("denso_pick_place").get_params()
            articulation_controller = my_denso.get_articulation_controller()
            pick_up_start = True
        
        observations = my_world.get_observations()
        # forward the observation values to the controller to get the actions
        actions = pick_controller.forward(
            picking_position=observations[task_params["cube_name"]["value"]]["position"],
            placing_position=observations[task_params["cube_name"]["value"]]["target_position"],
            current_joint_positions=observations[task_params["robot_name"]["value"]]["joint_positions"],
            # end_effector_offset=np.array([0, 0, 0.25]),
            end_effector_offset=np.array(cfg.end_effector_offset),
        )
        ee_pose = observations[task_params["robot_name"]["value"]]["end_effector_position"]
        if pick_up_start:
            print(f"Now the ee coordinate is: {ee_pose}")
        if pick_controller.is_done():
            print("done picking and placing")
        articulation_controller.apply_action(actions)



simulation_app.close() # close Isaac Sim