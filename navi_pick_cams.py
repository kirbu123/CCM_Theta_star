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
import argparse
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

# enable ROS bridge extension
extensions.enable_extension("omni.isaac.ros2_bridge-humble")

CAMERA_STAGE_PATH = "/World/Husky_Robot/fence_link/zed/husky_front_left" # 
LIDAR_STAGE_PATH = "/World/Husky_Robot/fence_link/fence_link_small/VLP_16/vlp16/rtx_lidar" # 
IMU_STAGE_PATH = "/World/Husky_Robot/base_link/imu" #
HUSKY_STAGE_PATH = "/World/Husky_Robot"
ROS_CAMERA_GRAPH_PATH = "/World/Husky_Robot/ROS_Cameras"
BACKGROUND_STAGE_PATH = "/background" # !
BACKGROUND_USD_PATH = "/Isaac/Environments/Simple_Warehouse/warehouse_with_forklifts.usd" #!

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


# Locate Isaac Sim assets folder to load environment and robot stages
assets_root_path = nucleus.get_assets_root_path() #!
if assets_root_path is None:
    carb.log_error("Could not find Isaac Sim assets folder")
    simulation_app.close()
    sys.exit()

# Loading the simple_room environment
stage.add_reference_to_stage(assets_root_path + BACKGROUND_USD_PATH, BACKGROUND_STAGE_PATH) #!

traj=[[-2,-2]]
# traj=[[3,3], [3, -3], [-3, -3], [-3, 3], [0, 0]]

navigation_end = np.array([1.4883547, 1.3499106, 0])
target_position = np.array([-0.3, 0.6, 0])
target_position[2] = 0.0515 / 2.0

# traj_completed = False
navigation_finish = False

# Relative coordinates
cube_size = np.array([0.0515, 0.0515, 0.0515]) / get_stage_units()
relative_cube_init_position = np.array([0.3, 0.3, 0.3]) / get_stage_units()
relative_cube_init_orientation = np.array([1, 0, 0, 0])
relative_target_position = np.array([-0.3, -0.3, 0]) / get_stage_units()
relative_target_position[2] = cube_size[2] / 2.0

pick_task = PickPlace(name="denso_pick_place",
                      husky_asset_path=os.path.abspath("exts/omni.isaac.examples/omni/isaac/examples/virtual-husky/assets/husky/husky_with_sensors.usd"),
                      cube_initial_position=np.array([2.1, 1.7, 0]),
                      cube_initial_orientation=np.array([1, 0, 0, 0]),
                      target_position=np.array([1.6, 2.2, 0.02]),
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



##? set up cameras
our_stage = get_current_stage()
zed_left_camera_prim =  UsdGeom.Camera(our_stage.GetPrimAtPath(CAMERA_STAGE_PATH))
# zed_left_camera_prim = stage.GetPrimAtPath('/World/Husky_Robot/fence_link/zed/husky_front_right')
zed_left_camera_prim.GetHorizontalApertureAttr().Set(5.12)
zed_left_camera_prim.GetVerticalApertureAttr().Set(2.88)
zed_left_camera_prim.GetProjectionAttr().Set("perspective")
zed_left_camera_prim.GetFocalLengthAttr().Set( 2.733857912)
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



##? Set up LiDAR

##? ##############

##? #######################

# Creating a action graph with ROS component nodes
try:
    og.Controller.edit(
        {"graph_path": "/ActionGraph", "evaluator_name": "execution"},
        {
            og.Controller.Keys.CREATE_NODES: [
                ##* TF Tree
                ("OnTick", "omni.graph.action.OnTick"),
                ("PublishClock", "omni.isaac.ros2_bridge.ROS2PublishClock"),
                ("rosContext", "omni.isaac.ros2_bridge.ROS2Context"),
                ("tfPublisher", "omni.isaac.ros2_bridge.ROS2PublishTransformTree"),
                ("ReadSimTime", "omni.isaac.core_nodes.IsaacReadSimulationTime"),

                ("lidarTfPublisher", "omni.isaac.ros2_bridge.ROS2PublishTransformTree"),
            ],
            og.Controller.Keys.CONNECT: [
                ("OnTick.outputs:tick", "PublishClock.inputs:execIn"),
                ("rosContext.outputs:context", "PublishClock.inputs:context"),
                ("OnTick.outputs:tick", "tfPublisher.inputs:execIn"),
                ("OnTick.outputs:tick", "lidarTfPublisher.inputs:execIn"),
                ("rosContext.outputs:context", "tfPublisher.inputs:context"),
                ("rosContext.outputs:context", "lidarTfPublisher.inputs:context"),
                ("ReadSimTime.outputs:simulationTime", "tfPublisher.inputs:timeStamp"),
                ("ReadSimTime.outputs:simulationTime", "lidarTfPublisher.inputs:timeStamp"),
                ("ReadSimTime.outputs:simulationTime", "PublishClock.inputs:timeStamp"),
            ],
            # og.Controller.Keys.SET_VALUES: [],
        },
    )
except Exception as e:
    print(e)

##* TF Tree
set_targets(
    prim=our_stage.GetPrimAtPath("/ActionGraph" + "/tfPublisher"),
    attribute="inputs:targetPrims",
    target_prim_paths=[HUSKY_STAGE_PATH],
)
set_targets(
    prim=our_stage.GetPrimAtPath("/ActionGraph" + "/lidarTfPublisher"),
    attribute="inputs:parentPrim",
    target_prim_paths=["/World/Husky_Robot/fence_link"],
)
set_targets(
    prim=our_stage.GetPrimAtPath("/ActionGraph" + "/lidarTfPublisher"),
    attribute="inputs:targetPrims",
    target_prim_paths=[LIDAR_STAGE_PATH],
)

simulation_app.update()


##* Set up Action Graph
keys = og.Controller.Keys
(ros_camera_graph, _, _, _) = og.Controller.edit(
    {
        "graph_path": ROS_CAMERA_GRAPH_PATH,
        "evaluator_name": "execution",
        # "pipeline_stage": og.GraphPipelineStage.GRAPH_PIPELINE_STAGE_ONDEMAND,
    },
    {
        keys.CREATE_NODES: [
            ("OnTick", "omni.graph.action.OnPlaybackTick"),
            ("createViewport", "omni.isaac.core_nodes.IsaacCreateViewport"),
            ("getRenderProduct", "omni.isaac.core_nodes.IsaacGetViewportRenderProduct"),
            ("setCamera", "omni.isaac.core_nodes.IsaacSetCameraOnRenderProduct"),
            ("cameraHelperRgb", "omni.isaac.ros2_bridge.ROS2CameraHelper"),
            ("cameraHelperInfo", "omni.isaac.ros2_bridge.ROS2CameraHelper"),
            ("cameraHelperDepth", "omni.isaac.ros2_bridge.ROS2CameraHelper"),

            ##? IMU
            ("imuReader", "omni.isaac.sensor.IsaacReadIMU"),
            ("publishImu", "omni.isaac.ros2_bridge.ROS2PublishImu"),
            ("readSimTime", "omni.isaac.core_nodes.IsaacReadSimulationTime"),

            ##? LiDAR
            ("createLiRenderProduct", "omni.isaac.core_nodes.IsaacCreateRenderProduct"),
            ("lidarHelperMsg", "omni.isaac.ros2_bridge.ROS2RtxLidarHelper"),
            ("lidarHelperPointcloud", "omni.isaac.ros2_bridge.ROS2RtxLidarHelper"),

            # ##* TF Tree
            # ("PublishClock", "omni.isaac.ros2_bridge.ROS2PublishClock"),
            # ("rosContext", "omni.isaac.ros2_bridge.ROS2Context"),
            # ("tfPublisher", "omni.isaac.ros2_bridge.ROS2PublishTransformTree"),
            # ("readSimTime", "omni.isaac.core_nodes.IsaacReadSimulationTime"),
        ],
        keys.CONNECT: [
            ("OnTick.outputs:tick", "createViewport.inputs:execIn"),
            ("createViewport.outputs:execOut", "getRenderProduct.inputs:execIn"),
            ("createViewport.outputs:viewport", "getRenderProduct.inputs:viewport"),
            ("getRenderProduct.outputs:execOut", "setCamera.inputs:execIn"),
            ("getRenderProduct.outputs:renderProductPath", "setCamera.inputs:renderProductPath"),
            ("setCamera.outputs:execOut", "cameraHelperRgb.inputs:execIn"),
            ("setCamera.outputs:execOut", "cameraHelperInfo.inputs:execIn"),
            ("setCamera.outputs:execOut", "cameraHelperDepth.inputs:execIn"),
            ("getRenderProduct.outputs:renderProductPath", "cameraHelperRgb.inputs:renderProductPath"),
            ("getRenderProduct.outputs:renderProductPath", "cameraHelperInfo.inputs:renderProductPath"),
            ("getRenderProduct.outputs:renderProductPath", "cameraHelperDepth.inputs:renderProductPath"),

            ##? IMU
            ("OnTick.outputs:tick", "imuReader.inputs:execIn"),
            ("imuReader.outputs:execOut", "publishImu.inputs:execIn"),
            ("imuReader.outputs:angVel", "publishImu.inputs:angularVelocity"),
            ("imuReader.outputs:linAcc", "publishImu.inputs:linearAcceleration"),
            ("imuReader.outputs:orientation", "publishImu.inputs:orientation"),
            ("readSimTime.outputs:simulationTime", "publishImu.inputs:timeStamp"),
            

            ##? LiDAR
            ("OnTick.outputs:tick", "createLiRenderProduct.inputs:execIn"),
            ("createLiRenderProduct.outputs:execOut", "lidarHelperMsg.inputs:execIn"),
            ("createLiRenderProduct.outputs:execOut", "lidarHelperPointcloud.inputs:execIn"),
            ("createLiRenderProduct.outputs:renderProductPath", "lidarHelperMsg.inputs:renderProductPath"),
            ("createLiRenderProduct.outputs:renderProductPath", "lidarHelperPointcloud.inputs:renderProductPath"),

             ##* TF Tree
            #  ("OnTick.outputs:tick", "PublishClock.inputs:execIn"),
            #  ("rosContext.outputs:context", "PublishClock.inputs:context"),
            #  ("OnTick.outputs:tick", "tfPublisher.inputs:execIn"),
            #  ("rosContext.outputs:context", "tfPublisher.inputs:context"),
            #  ("rosContext.outputs:context", "lidarHelperMsg.inputs:context"),
            #  ("rosContext.outputs:context", "lidarHelperPointcloud.inputs:context"),
            #  ("rosContext.outputs:context", "cameraHelperRgb.inputs:context"),
            #  ("rosContext.outputs:context", "cameraHelperInfo.inputs:context"),
            #  ("rosContext.outputs:context", "cameraHelperDepth.inputs:context"),
            #  ("readSimTime.outputs:simulationTime", "tfPublisher.inputs:timeStamp"),
            #  ("readSimTime.outputs:simulationTime", "PublishClock.inputs:timeStamp"),
        ],
        keys.SET_VALUES: [
            ("createViewport.inputs:viewportId", 0),
            ("cameraHelperRgb.inputs:frameId", "sim_camera"),
            ("cameraHelperRgb.inputs:topicName", "rgb"),
            ("cameraHelperRgb.inputs:type", "rgb"),
            ("cameraHelperInfo.inputs:frameId", "sim_camera"),
            ("cameraHelperInfo.inputs:topicName", "camera_info"),
            ("cameraHelperInfo.inputs:type", "camera_info"),
            ("cameraHelperDepth.inputs:frameId", "sim_camera"),
            ("cameraHelperDepth.inputs:topicName", "depth"),
            ("cameraHelperDepth.inputs:type", "depth"),

            ##? IMU
            ("publishImu.inputs:topicName", "imu"),
            ("publishImu.inputs:frameId", "sim_imu"),

            ##? LiDAR
            ("lidarHelperPointcloud.inputs:topicName", "point_cloud"),
            ("lidarHelperPointcloud.inputs:frameId", "rtx_lidar"),
            ("lidarHelperPointcloud.inputs:type", "point_cloud"),
        ],
    },
)

##* ##########################

set_targets(
    prim=our_stage.GetPrimAtPath(ROS_CAMERA_GRAPH_PATH + "/setCamera"),
    attribute="inputs:cameraPrim",
    target_prim_paths=[CAMERA_STAGE_PATH],
)

##? IMU
set_targets(
    prim=our_stage.GetPrimAtPath(ROS_CAMERA_GRAPH_PATH + "/imuReader"),
    attribute="inputs:imuPrim",
    target_prim_paths=[IMU_STAGE_PATH],
)

##? LiDAR
set_targets(
    prim=our_stage.GetPrimAtPath(ROS_CAMERA_GRAPH_PATH + "/createLiRenderProduct"),
    attribute="inputs:cameraPrim",
    target_prim_paths=[LIDAR_STAGE_PATH],
)



# Run the ROS Camera graph once to generate ROS image publishers in SDGPipeline
og.Controller.evaluate_sync(ros_camera_graph)

simulation_app.update()

# Inside the SDGPipeline graph, Isaac Simulation Gate nodes are added to control the execution rate of each of the ROS image and camera info publishers.
# By default the step input of each Isaac Simulation Gate node is set to a value of 1 to execute every frame.
# We can change this value to N for each Isaac Simulation Gate node individually to publish every N number of frames.
viewport_api = get_active_viewport()

if viewport_api is not None:
    import omni.syntheticdata._syntheticdata as sd

    # Get name of rendervar for RGB sensor type
    rv_rgb = omni.syntheticdata.SyntheticData.convert_sensor_type_to_rendervar(sd.SensorType.Rgb.name)

    # Get path to IsaacSimulationGate node in RGB pipeline
    rgb_camera_gate_path = omni.syntheticdata.SyntheticData._get_node_path(
        rv_rgb + "IsaacSimulationGate", viewport_api.get_render_product_path()
    )

    # Get name of rendervar for DistanceToImagePlane sensor type
    rv_depth = omni.syntheticdata.SyntheticData.convert_sensor_type_to_rendervar(
        sd.SensorType.DistanceToImagePlane.name
    )

    # Get path to IsaacSimulationGate node in Depth pipeline
    depth_camera_gate_path = omni.syntheticdata.SyntheticData._get_node_path(
        rv_depth + "IsaacSimulationGate", viewport_api.get_render_product_path()
    )

    # Get path to IsaacSimulationGate node in CameraInfo pipeline
    camera_info_gate_path = omni.syntheticdata.SyntheticData._get_node_path(
        "PostProcessDispatch" + "IsaacSimulationGate", viewport_api.get_render_product_path()
    )

    # Set Rgb execution step to 5 frames
    rgb_step_size = 5

    # Set Depth execution step to 60 frames
    depth_step_size = 10

    # Set Camera info execution step to every frame
    info_step_size = 1

    # Set step input of the Isaac Simulation Gate nodes upstream of ROS publishers to control their execution rate
    # og.Controller.attribute(rgb_camera_gate_path + ".inputs:step").set(rgb_step_size)
    # og.Controller.attribute(depth_camera_gate_path + ".inputs:step").set(depth_step_size)
    # og.Controller.attribute(camera_info_gate_path + ".inputs:step").set(info_step_size)

# Need to initialize physics getting any articulation..etc
my_world.initialize_physics()
# input()

while simulation_app.is_running():
    my_world.step(render=True)
    if my_world.is_playing():
        if not navigation_finish:
            for i, milestone in enumerate(traj):
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
                                                                        lateral_velocity=0.5,
                                                                        position_tol = 0.1,)
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
                                                    lateral_velocity=0.5,
                                                    position_tol = 0.1,)   
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
            end_effector_offset=np.array([0, 0, 0.05]),
        )
        ee_pose = observations[task_params["robot_name"]["value"]]["end_effector_position"]
        if pick_up_start:
            print(f"Now the ee coordinate is: {ee_pose}")
        if pick_controller.is_done():
            print("done picking and placing")
        articulation_controller.apply_action(actions)



simulation_app.close() # close Isaac Sim