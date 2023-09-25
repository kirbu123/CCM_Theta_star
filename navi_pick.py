# Copyright (c) 2022, NVIDIA CORPORATION.  All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto.  Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.
#

from omni.isaac.kit import SimulationApp

simulation_app = SimulationApp({"headless": False})

import os
import argparse
import numpy as np

from omni.isaac.core import World
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
from pxr import UsdPhysics, Usd, Sdf
from omni.isaac.core.utils.stage import add_reference_to_stage
from omni.isaac.core.prims import RigidPrimView,XFormPrim
from omni.isaac.core.utils.rotations import euler_angles_to_quat

from src.tasks.follow_target import FollowTarget
from src.tasks.pick_place import PickPlace
from src.controllers.pick_place import PickPlaceController
from src.config import get_config, Config


cfg = get_config()

parser = argparse.ArgumentParser()
parser.add_argument("--test", default=False, action="store_true", help="Run in test mode")
args, unknown = parser.parse_known_args()

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

# add table
# mipt_env_path = cfg.table_usd_path
# add_reference_to_stage(usd_path=mipt_env_path, prim_path="/World/table")
# stage = get_current_stage()
# position = np.array(cfg.table_position)
# orientation = np.array(cfg.table_orientation)
# mipt_table = XFormPrim(prim_path = "/World/table", position=position, orientation=orientation)

# add trash can
container_path = cfg.can_usd_path
add_reference_to_stage(usd_path=container_path, prim_path="/World/container")
stage = get_current_stage()
position = np.array(cfg.can_position)
orientation = np.array(cfg.can_orientation)
trash_can = XFormPrim(prim_path = "/World/container", position=position, orientation=orientation)



navigation_finish = False

navigation_one_start = True
navigation_one_finish = False
pick_up_start = False
pick_up_finish = False
navigation_two_start = False
navigation_two_finish = False
place_start = False
place_finish = False

init_pick_controller = False


pick_task = PickPlace(name="denso_pick_place",
                      cfg=cfg,
                      )

my_world.add_task(pick_task)
my_world.reset()
my_denso = my_world.scene.get_object("my_ur5")

my_world.reset()
husky = pick_task.robots["husky"]
ur5 = pick_task.robots["ur5"]
cube = pick_task.obj["cube"]

scene={}

# add_goals(my_world, scene, cfg.trajectory)  # u can add a red cube as the target position of navigation
my_world.reset()

prim_trans_point = pick_task.robots["trans_point"]

####################################### Init Husky controller ################################
husky_controller = WheelBasePoseController(name="cool_controller",
                                                    open_loop_wheel_controller=
                                                    DifferentialController(name="simple_control",
                                                    wheel_radius=cfg.wheel_radius,
                                                    wheel_base=cfg.wheel_base),
                                                    is_holonomic=False,)


while simulation_app.is_running():
    my_world.step(render=True)
    if my_world.is_playing():
        
        ###########################################   Navi One   #######################################################
        if navigation_one_start and (not navigation_one_finish) and (not pick_up_start) and (not pick_up_finish) and (not place_start) and (not place_finish):
            print("Step navi one!\n")
            ####################################### Init UR5 controller ################################
            if not init_pick_controller:                
                pick_place_controller = PickPlaceController(name="controller",
                                            robot_articulation=my_denso, 
                                            gripper=my_denso.gripper,
                                            cfg=cfg,
                                            )
                task_params = my_world.get_task("denso_pick_place").get_params()
                articulation_controller = my_denso.get_articulation_controller()
                init_pick_controller = True

            for i, milestone in enumerate(cfg.husky_pick_position):

                husky_pick_position = np.asarray(milestone)
                print(f"\nCurr goal: {husky_pick_position}")
                distance = 100

                while not (distance < 1.2):
                    # print(f"World pose: {husky.get_world_pose()[0][:2]}\n")
                    print(f"Distance of Navi1: {distance}\n")
                    position, orientation = husky.get_world_pose()

                    wheel_actions=husky_controller.forward(start_position=position,
                                                                        start_orientation=orientation,
                                                                        goal_position=husky_pick_position,
                                                                        lateral_velocity=cfg.lateral_velocity,
                                                                        position_tol = cfg.position_tol,)

                    wheel_actions.joint_velocities =np.tile(wheel_actions.joint_velocities, 2)

                    husky.apply_wheel_actions(wheel_actions)
                    my_world.step(render=True)
                    distance = np.sum((husky.get_world_pose()[0][:2] - husky_pick_position)**2)  # Compute distance between husky and target

            navi_one_end_position, navi_one_end_orientation = husky.get_world_pose()

            navigation_one_start = False
            navigation_one_finish = True 
            pick_up_start = True


            pick_place_controller._cspace_controller.reset(cube) # Very Important! Here init the RMPFlow's pose after husky move!!!!
            
            

        ###########################################   To pick up   #######################################################
        
        if (navigation_one_finish) and (pick_up_start) and (not pick_up_finish) and (not place_start) and (not place_finish):
            
            
            
            # print("Step pick up!\n")
            wheel_actions=husky_controller.forward(start_position=navi_one_end_position,
                                                        # start_orientation=navi_one_end_orientation,
                                                        start_orientation=np.array([0,0,0,1]),
                                                        goal_position=navi_one_end_position,
                                                        lateral_velocity=0.5,
                                                        position_tol = 0.1,)   
            wheel_actions.joint_velocities =np.tile(wheel_actions.joint_velocities, 2)
            # print(f"Modified wheel actions: {wheel_actions}\n")
            husky.apply_wheel_actions(wheel_actions)

            observations = my_world.get_observations()
            # forward the observation values to the controller to get the actions
            actions = pick_place_controller.forward(
                picking_position=cube.get_world_pose()[0],
                placing_position=observations[task_params["cube_name"]["value"]]["target_position"],  ## the placing position is not matter here, because only pick
                current_joint_positions=observations[task_params["robot_name"]["value"]]["joint_positions"],
                # end_effector_offset=np.array([0, 0, 0.25]),
                end_effector_offset=np.array(cfg.end_effector_offset),
                end_effector_orientation = euler_angles_to_quat(np.array([0, np.pi, 0.5*np.pi])),
                prim_trans_point = prim_trans_point,
            )
            ee_pose = observations[task_params["robot_name"]["value"]]["end_effector_position"]

            articulation_controller.apply_action(actions)

            num_event = pick_place_controller._event
            print(f"Now event: {num_event}\n")

            if pick_place_controller.pick_done():
                pick_up_start = False
                pick_up_finish = True
                navigation_two_start = True
                pick_place_controller.pause()
                print("pick done!")
                
            
        ###################################   To next position for placing(Navi Two)    #####################################

        if (navigation_one_finish) and (pick_up_finish) and (navigation_two_start) and (not navigation_two_finish) and (not place_start):    
            print("Step navi 2!\n")

            for i, milestone in enumerate(cfg.husky_place_position):

                distance = 100
                husky_place_position = np.asarray(milestone)
                # while not np.allclose(husky.get_world_pose()[0][:2], milestone, atol=0.1):
                while not (distance < 1.2):
                    print(f"Distance of Navi2: {distance}\n")
                    # print(f"Curr position: {position}\n")
                    # print(f"Curr orientation: {orientation}\n")
                    position, orientation = husky.get_world_pose()
                    wheel_actions=husky_controller.forward(start_position=position,
                                                                        start_orientation=orientation,
                                                                        # start_orientation=np.array([1, 0, 0, 0]),
                                                                        goal_position=husky_place_position,
                                                                        lateral_velocity=0.5,
                                                                        position_tol = 0.1,)

                    wheel_actions.joint_velocities =np.tile(wheel_actions.joint_velocities, 2)

                    husky.apply_wheel_actions(wheel_actions)
                    my_world.step(render=True)
                    distance = np.sum((husky.get_world_pose()[0][:2] - milestone)**2)  # Compute distance between husky and target

            navi_two_end_position, navi_two_end_orientation = husky.get_world_pose()

            navigation_two_start = False
            navigation_two_finish = True
            place_start = True
            place_finish = False

            pick_place_controller._cspace_controller.reset(cube) # Very Important! Here init the RMPFlow's pose after husky move!!!!
        
        ###################################   To Place    #########################################################

        if (navigation_one_finish) and (pick_up_finish) and (navigation_two_finish) and (place_start) and (not place_finish):

            wheel_actions=husky_controller.forward(start_position=navi_two_end_position,
                                                        # start_orientation=navi_two_end_orientation,
                                                        start_orientation=np.array([0,0,0,1]),
                                                        goal_position=navi_two_end_position,
                                                        lateral_velocity=0.5,
                                                        position_tol = 0.1,)   
            wheel_actions.joint_velocities =np.tile(wheel_actions.joint_velocities, 2)
            husky.apply_wheel_actions(wheel_actions)

            pick_place_controller.resume()

            num_event = pick_place_controller._event
            # print(f"Now event: {num_event}\n")

            observations = my_world.get_observations()
            # forward the observation values to the controller to get the actions
            actions = pick_place_controller.forward(
                # picking_position=observations[task_params["cube_name"]["value"]]["position"],   ## the picking position is not matter here, because only pick       
                picking_position=cube.get_world_pose()[0],
                # picking_position=observations[task_params["cube_name"]["value"]]["target_position"],

                placing_position=observations[task_params["cube_name"]["value"]]["target_position"],

                current_joint_positions=observations[task_params["robot_name"]["value"]]["joint_positions"],
                # end_effector_offset=np.array([0, 0, 0.25]),
                end_effector_offset=np.array(cfg.end_effector_offset),
                end_effector_orientation = euler_angles_to_quat(np.array([0, np.pi, 0.8*np.pi])),
                prim_trans_point = prim_trans_point,
                ee_pose = observations[task_params["robot_name"]["value"]]["end_effector_position"],
            )

            # picking_position=observations[task_params["cube_name"]["value"]]["target_position"]
            # print(f"picking_position: {picking_position}\n")


            articulation_controller.apply_action(actions)
            ee_pose = observations[task_params["robot_name"]["value"]]["end_effector_position"]
            print(f"ee world pose: {ee_pose}")
            
            ## place_start = False
            ## place_finish = True




simulation_app.close() # close Isaac Sim