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

from omni.isaac.core import World
from tasks.follow_target import FollowTarget
from tasks.navigation import Navigate
import numpy as np
from controllers.rmpflow import RMPFlowController
from tasks.pick_place import PickPlace
from controllers.pick_place import PickPlaceController
import argparse
from omni.isaac.wheeled_robots.robots import WheeledRobot
from omni.isaac.motion_generation import WheelBasePoseController
from omni.isaac.wheeled_robots.controllers.differential_controller import DifferentialController
from omni.isaac.core.objects import DynamicCuboid
from omni.isaac.core.utils.string import find_unique_string_name
from omni.isaac.core.materials import PreviewSurface
from omni.isaac.core.utils.prims import get_prim_at_path, is_prim_path_valid
from omni.isaac.core.articulations import ArticulationSubset
from pxr import UsdPhysics, Usd, Sdf
from omni.isaac.core.utils.stage import get_current_stage
from omni.isaac.core.utils.stage import get_stage_units
from omni.isaac.core.utils.stage import add_reference_to_stage
from omni.isaac.core.prims import RigidPrimView,XFormPrim


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

# add mipt_env
mipt_env_path = "/home/zhang/save/8-9/mipt_env.usd"
add_reference_to_stage(usd_path=mipt_env_path, prim_path="/World/mipt_env")
stage = get_current_stage()
position = np.array([-18, -1, -0.088])
mipt_wall = XFormPrim(prim_path = "/World/mipt_env/MIPT_env_small_simple", position=position)

# /World/defaultGroundPlane/Enviroment
mipt_env = mipt_wall = XFormPrim(prim_path = "/World/defaultGroundPlane/Enviroment")
mipt_env.set_visibility(visible=False)

# prim_mipt = stage.GetPrimAtPath('/World/mipt_env/MIPT_env_small_simple')
# prim_mipt.set_world_poses(positions=positions,)

traj=[[2,2]]
# traj=[[3,3], [3, -3], [-3, -3], [-3, 3], [0, 0]]

navigation_end = np.array([1.4883547, 1.3499106, 0])
target_position = np.array([-0.3, 0.6, 0])
target_position[2] = 0.0515 / 2.0

# traj_completed = False
navigation_one_start = True
navigation_one_finish = False
pick_up_start = False
pick_up_finish = False
navigation_two_start = False
navigation_two_finish = False
place_start = False
place_finish = False

init_pick_controller = False

# Relative coordinates

# cube_size = np.array([0.0515, 0.0515, 0.0515]) / get_stage_units()
# relative_cube_init_position = np.array([0.3, 0.3, 0.3]) / get_stage_units()
# relative_cube_init_orientation = np.array([1, 0, 0, 0])
# relative_target_position = np.array([-0.3, -0.3, 0]) / get_stage_units()
# relative_target_position[2] = cube_size[2] / 2.0

pick_task = PickPlace(name="denso_pick_place",
                    cube_initial_position=np.array([2.1, 1.7, 0]),
                    cube_initial_orientation=np.array([1, 0, 0, 0]),
                    # target_position=np.array([1.6, 2.2, 0.02]),
                    target_position=np.array([1.6, 2.2, 1]),
                    )

my_world.add_task(pick_task)
my_world.reset()
my_denso = my_world.scene.get_object("my_ur5")

my_world.reset()
husky=pick_task.robots["husky"]

scene={}

add_goals(my_world, scene, traj)
my_world.reset()



####################################### Init Husky controller ################################
husky_controller = WheelBasePoseController(name="cool_controller",
                                                    open_loop_wheel_controller=
                                                    DifferentialController(name="simple_control",
                                                    wheel_radius=0.3, wheel_base=0.5),
                                                    is_holonomic=False,)
articulation_controller = my_denso.get_articulation_controller()



while simulation_app.is_running():
    my_world.step(render=True)
    if my_world.is_playing():
        if navigation_one_start and (not navigation_one_finish) and (not pick_up_start) and (not pick_up_finish) and (not place_start) and (not place_finish):
            print("Step navi one!\n")
            for i, milestone in enumerate(traj):
                milestone = np.asarray(milestone)
                print(f"\nCurr goal: {milestone}")
                distance = 100
                # while not np.allclose(husky.get_world_pose()[0][:2], milestone, atol=0.1):
                while not (distance < 0.7):
                    print(f"Distance to target1: {distance}\n")
                    position, orientation = husky.get_world_pose()
                    wheel_actions=husky_controller.forward(start_position=position,
                                                                        start_orientation=orientation,
                                                                        goal_position=milestone,
                                                                        lateral_velocity=0.5,
                                                                        position_tol = 0.1,)

                    wheel_actions.joint_velocities =np.tile(wheel_actions.joint_velocities, 2)
                    # print(f"Modified wheel actions: {wheel_actions}\n")
                    # print(f"Wheel number: {husky._num_wheel_dof} - Actions length: {wheel_actions.get_length()}")
                    husky.apply_wheel_actions(wheel_actions)
                    my_world.step(render=True)
                    distance = np.sum((husky.get_world_pose()[0][:2] - milestone)**2)  # Compute distance between husky and target

            navi_one_end_position, navi_one_end_orientation = husky.get_world_pose()
            # print("\n Navigation finish!\n Now Start grasp!! \n")
            navigation_one_start = False
            navigation_one_finish = True 
            pick_up_start = True

        # if not pick_up_start:
        #     print(f"Now the coordinates of Husky is : {husky.get_world_pose()[0][:2]}\n")
        
        ###########################################   To pick up   #######################################################
        
        if (navigation_one_finish) and (pick_up_start) and (not pick_up_finish) and (not place_start) and (not place_finish):
            print("Step pick up!\n")

            wheel_actions=husky_controller.forward(start_position=navi_one_end_position,
                                                        start_orientation=navi_one_end_orientation,
                                                        goal_position=navi_one_end_position,
                                                        lateral_velocity=0.5,
                                                        position_tol = 0.1,)   
            wheel_actions.joint_velocities =np.tile(wheel_actions.joint_velocities, 2)
            # print(f"Modified wheel actions: {wheel_actions}\n")
            husky.apply_wheel_actions(wheel_actions)

            # husky_controller.reset
            if not init_pick_controller:
                print(f"Now let's pick up ! \n")
                pick_place_controller = PickPlaceController(name="controller",
                                            robot_articulation=my_denso, 
                                            gripper=my_denso.gripper,
                                            )
                task_params = my_world.get_task("denso_pick_place").get_params()
                articulation_controller = my_denso.get_articulation_controller()
                init_pick_controller = True         

            observations = my_world.get_observations()
            # forward the observation values to the controller to get the actions
            actions = pick_place_controller.forward(
                picking_position=observations[task_params["cube_name"]["value"]]["position"],
                placing_position=observations[task_params["cube_name"]["value"]]["target_position"],
                current_joint_positions=observations[task_params["robot_name"]["value"]]["joint_positions"],
                # end_effector_offset=np.array([0, 0, 0.25]),
                end_effector_offset=np.array([0, 0, 0.05]),
            )
            ee_pose = observations[task_params["robot_name"]["value"]]["end_effector_position"]
            # if pick_up_start:
            #     print(f"Now the ee coordinate is: {ee_pose}")
            articulation_controller.apply_action(actions)
            
            if pick_place_controller.is_done():
                # print("Now to next target\n")
                pick_up_start = False
                pick_up_finish = True
                navigation_two_start = True
        ###################################   To next position for placing    #####################################

        if (navigation_one_finish) and (pick_up_finish) and (navigation_two_start) and (not navigation_two_finish) and (not place_start) and (not place_finish):    
            print("Step navi 2!\n")

            distance = 100
            # while not np.allclose(husky.get_world_pose()[0][:2], milestone, atol=0.1):
            while not (distance < 0.7):
                print(f"Distance: {distance}\n")
                position, orientation = husky.get_world_pose()
                wheel_actions=husky_controller.forward(start_position=position,
                                                                    start_orientation=orientation,
                                                                    goal_position=np.array([3, 2, 0]),
                                                                    lateral_velocity=0.5,
                                                                    position_tol = 0.1,)

                wheel_actions.joint_velocities =np.tile(wheel_actions.joint_velocities, 2)
                print(f"Modified wheel actions: {wheel_actions}\n")
                # print(f"Wheel number: {husky._num_wheel_dof} - Actions length: {wheel_actions.get_length()}")
                husky.apply_wheel_actions(wheel_actions)
                my_world.step(render=True)
                distance = np.sum((husky.get_world_pose()[0][:2] - milestone)**2)  # Compute distance between husky and target
            
            navigation_two_start = False
            navigation_two_finish = True
            place_start = True
        
        ###################################   To Place    #########################################################



simulation_app.close() # close Isaac Sim