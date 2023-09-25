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
try:
    from omni.usd.utils import get_world_transform_matrix, get_local_transform_matrix
except:
    from omni.usd import get_world_transform_matrix, get_local_transform_matrix


from src.tasks.follow_target import FollowTarget
from src.tasks.pick_place import PickPlace
from src.controllers.pick_place import PickPlaceController
from src.config import get_config, Config

# ur rtde communication
import rtde_control
import rtde_receive
import sys

cfg = get_config()

parser = argparse.ArgumentParser()
# parser.add_argument("--test", default=False, action="store_true", help="Run in test mode")
parser.add_argument(
    "--robot-ip",
    type=str,
    default="127.0.0.1",
    help="IP adress of robot Real world UR Polyscope or VM UR Polyscope",
)
# args, unknown = parser.parse_known_args()
arg = parser.parse_args()



###################################### Init Scene #############################################
my_world = World(stage_units_in_meters=1.0)

# add table
mipt_env_path = cfg.table_usd_path
add_reference_to_stage(usd_path=mipt_env_path, prim_path="/World/table")
stage = get_current_stage()
position = np.array(cfg.table_position)
orientation = np.array(cfg.table_orientation)
mipt_table = XFormPrim(prim_path = "/World/table", position=position, orientation=orientation)

# add trash can
# container_path = cfg.can_usd_path
# add_reference_to_stage(usd_path=container_path, prim_path="/World/can")
# stage = get_current_stage()
# position = np.array(cfg.can_position)
# orientation = np.array(cfg.can_orientation)
# trash_can = XFormPrim(prim_path = "/World/can", position=position, orientation=orientation)

# add opened drawer
container_path = cfg.drawer_usd_path
add_reference_to_stage(usd_path=container_path, prim_path="/World/drawer")
stage = get_current_stage()
position = np.array(cfg.drawer_position)
orientation = np.array(cfg.drawer_orientation)
trash_can = XFormPrim(prim_path = "/World/drawer", position=position, orientation=orientation)


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

obj_on_hand = ""

my_world.reset()

prim_trans_point = pick_task.robots["trans_point"]

####################################### Init Husky controller ################################
husky_controller = WheelBasePoseController(name="cool_controller",
                                                    open_loop_wheel_controller=
                                                    DifferentialController(name="simple_control",
                                                    wheel_radius=cfg.wheel_radius,
                                                    wheel_base=cfg.wheel_base),
                                                    is_holonomic=False,)
####################################### Init UR5 controller ################################
pick_place_controller = PickPlaceController(name="controller",
                            robot_articulation=my_denso, 
                            gripper=my_denso.gripper,
                            cfg=cfg,
                            )

# IP adress of robot Real world UR Polyscope or VM UR Polyscope
try:
    rtde_r = rtde_receive.RTDEReceiveInterface(arg.robot_ip)
    rtde_c = rtde_control.RTDEControlInterface(arg.robot_ip)
    # ur5.set_joint_positions(np.array(rtde_r.getActualQ()))
    connect_to_rtde = True
except:
    print("[ERROR] Robot is not connected")
    # close isaac sim
    # simulation_app.close()
    # sys.exit()
    connect_to_rtde = False
                      
# Parameters for rtde
velocity = 0.1
acceleration = 0.1
dt = 1.0/500  # 2ms
lookahead_time = 0.1
gain = 300



task_params = my_world.get_task("denso_pick_place").get_params()
articulation_controller = my_denso.get_articulation_controller()



while simulation_app.is_running():
    my_world.step(render=True)
    if my_world.is_playing():
        
        com = input("Choose one action: (1.move_to; 2.pick_up; 3.put):")


        com = int(com)    
        if com == 1: ## Move
            obj_move_str = input("Input the object to move: ")
            stage = get_current_stage()
            prim_obj_move = stage.GetPrimAtPath(f"/World/{obj_move_str}")
            pose_obj_move = get_world_transform_matrix(prim_obj_move)
            position_obj_move = pose_obj_move.ExtractTranslation()
            print(f"Now move to {obj_move_str}, position {position_obj_move}\n")
            
            distance = 100
            if (obj_move_str == "table") or (obj_move_str == "drawer"):
                stop_distance = 1.5
            else:
                stop_distance = 1.2
            while not (distance < stop_distance):
                print(f"Distance to {obj_move_str}: {distance}\n")
                position, orientation = husky.get_world_pose()
                wheel_actions=husky_controller.forward(start_position=position,
                                                                    start_orientation=orientation,
                                                                    goal_position=position_obj_move[:2],
                                                                    lateral_velocity=cfg.lateral_velocity,
                                                                    position_tol = cfg.position_tol,)

                wheel_actions.joint_velocities =np.tile(wheel_actions.joint_velocities, 2)
                husky.apply_wheel_actions(wheel_actions)
                my_world.step(render=True)
                distance = np.sum((husky.get_world_pose()[0][:2] - position_obj_move[:2])**2)  # Compute distance between husky and target

                if connect_to_rtde: 
                    # jointq = get joints positions
                    joint_q = ur5.get_joint_positions()[:6]
                    # print(f"joint_q: {joint_q}")

                    # time start period
                    t_start = rtde_c.initPeriod()

                    # run servoJ
                    rtde_c.servoJ(joint_q, velocity, acceleration, dt, lookahead_time, gain)
                    rtde_c.waitPeriod(t_start)  

        elif com == 2: ## Pick
            obj_pick_str = input("Input the object to pick: ")
            stage = get_current_stage()
            prim_obj_pick = stage.GetPrimAtPath(f"/World/{obj_pick_str}")
            pose_obj_pick = get_world_transform_matrix(prim_obj_pick)
            position_obj_pick = pose_obj_pick.ExtractTranslation()
            print(f"Now move to {obj_pick_str}, position {position_obj_pick}\n")

            position, orientation = husky.get_world_pose()
            pick_place_controller._cspace_controller.reset()
            
            while not pick_place_controller.pick_done():
                wheel_actions=husky_controller.forward(start_position=position,
                                                        start_orientation=orientation,
                                                        goal_position=position[:2],
                                                        lateral_velocity=cfg.lateral_velocity,
                                                        position_tol = cfg.position_tol,)
                wheel_actions.joint_velocities =np.tile(wheel_actions.joint_velocities, 2)
                husky.apply_wheel_actions(wheel_actions)
                my_world.step(render=True)                

                observations = my_world.get_observations()
                actions = pick_place_controller.forward(
                    picking_position=cube.get_world_pose()[0],
                    placing_position=observations[task_params["cube_name"]["value"]]["target_position"],  ## the placing position is not matter here, because only pick
                    current_joint_positions=observations[task_params["robot_name"]["value"]]["joint_positions"],
                    # end_effector_offset=np.array([0, 0, 0.25]),
                    end_effector_offset=np.array(cfg.end_effector_offset),
                    end_effector_orientation = euler_angles_to_quat(np.array([0, np.pi, 0.5*np.pi])),
                    prim_trans_point = prim_trans_point,
                )

                if connect_to_rtde: 
                    # jointq = get joints positions
                    joint_q = ur5.get_joint_positions()[:6]
                    # print(f"joint_q: {joint_q}")

                    # time start period
                    t_start = rtde_c.initPeriod()

                    # run servoJ
                    rtde_c.servoJ(joint_q, velocity, acceleration, dt, lookahead_time, gain)
                    rtde_c.waitPeriod(t_start)  

                articulation_controller.apply_action(actions)


            
            # ee_pose = observations[task_params["robot_name"]["value"]]["end_effector_position"]

            pick_place_controller.pause()
            print("pick done!")
            obj_on_hand = obj_pick_str

        elif com == 3: ## Put
            obj_put_str = input("Input the objects to put (obj1 on obj2): ")
            obj1_str = obj_put_str.split(" ")[0]
            obj2_str = obj_put_str.split(" ")[1]
            print(f"Now put {obj1_str} on {obj2_str}\n")
            stage = get_current_stage()
            prim_obj_put_1 = stage.GetPrimAtPath(f"/World/{obj1_str}")
            prim_obj_put_2 = stage.GetPrimAtPath(f"/World/{obj2_str}")
            pose_obj_1 = get_world_transform_matrix(prim_obj_put_1)
            pose_obj_2 = get_world_transform_matrix(prim_obj_put_2)
            position_obj_1 = pose_obj_1.ExtractTranslation()
            position_obj_2 = pose_obj_2.ExtractTranslation()
            if (obj2_str == "table") or (obj2_str == "drawer"):
                prim_obj_put_2 = stage.GetPrimAtPath(f"/World/{obj2_str}/point")
                pose_obj_2 = get_world_transform_matrix(prim_obj_put_2)
                position_obj_2 = pose_obj_2.ExtractTranslation()                
                position_obj_2[-1] = position_obj_2[-1] + 0.036 if obj2_str == "table" else position_obj_2[-1] + 0.06 # offset of place
                print(f"position_obj_2: {position_obj_2}\n")
                

            position, orientation = husky.get_world_pose()
            pick_place_controller._cspace_controller.reset()
            
            pick_place_controller.resume()
            print(f"is done: {pick_place_controller.is_done()}")
            while not pick_place_controller.is_done():
                wheel_actions=husky_controller.forward(start_position=position,
                                                        start_orientation=orientation,
                                                        goal_position=position[:2],
                                                        lateral_velocity=cfg.lateral_velocity,
                                                        position_tol = cfg.position_tol,)
                wheel_actions.joint_velocities =np.tile(wheel_actions.joint_velocities, 2)
                husky.apply_wheel_actions(wheel_actions)
                my_world.step(render=True)                

                observations = my_world.get_observations()
                end_effector_orientation = euler_angles_to_quat(np.array([0, np.pi, 0.5*np.pi])) if obj2_str == "table" else euler_angles_to_quat(np.array([0, np.pi, 0.8*np.pi])) 
                
                actions = pick_place_controller.forward(     
                picking_position=cube.get_world_pose()[0],
                # placing_position=observations[task_params["cube_name"]["value"]]["target_position"],
                placing_position = position_obj_2,
                current_joint_positions=observations[task_params["robot_name"]["value"]]["joint_positions"],
                # end_effector_offset=np.array([0, 0, 0.25]),
                end_effector_offset=np.array(cfg.end_effector_offset),
                end_effector_orientation = end_effector_orientation,
                prim_trans_point = prim_trans_point,
                ee_pose = observations[task_params["robot_name"]["value"]]["end_effector_position"],
                )
                if connect_to_rtde: 
                    # jointq = get joints positions
                    joint_q = ur5.get_joint_positions()[:6]
                    # print(f"joint_q: {joint_q}")

                    # time start period
                    t_start = rtde_c.initPeriod()

                    # run servoJ
                    rtde_c.servoJ(joint_q, velocity, acceleration, dt, lookahead_time, gain)
                    rtde_c.waitPeriod(t_start)  
                    
                articulation_controller.apply_action(actions)
            pick_place_controller.reset()



# rtde control stop script and disconnect
rtde_c.servoStop()
rtde_c.stopScript()
rtde_r.disconnect()

# close Isaac Sim
simulation_app.close() 