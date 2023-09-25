from pxr.Usd import Stage
import omni.graph.core as og
from omni.isaac.kit import SimulationApp
from omni.isaac.core.utils.prims import set_targets

from src.config import Config


def setup_camera_tf_graph(cfg: Config,
                       simulation_app: SimulationApp,
                       stage: Stage,
                       controller: og.Controller,
                       camera_name: str):
    '''Setup the action graph for publishing Images, Depths and CameraInfo to ROS1/2'''

    keys = og.Controller.Keys
    graph = og.get_graph_by_path(cfg.tf.action_graph_path)

    ros_bridge = cfg.ros_cfg[cfg.ros].ros_bridge_extension.split('-')[0]
    ros_v = cfg.ros_cfg[cfg.ros].ros_v

    # controller = og.Controller(graph_id=cfg.cameras.action_graph_id)
    controller.edit(
        graph,
        {
            keys.CREATE_NODES: [
                # ("OnTick", "omni.graph.action.OnPlaybackTick"),
                (f"{camera_name}_TfPublisher", f"{ros_bridge}.ROS{ros_v}PublishTransformTree"),
            ],
            keys.CONNECT: [
                ("ReadSimTime.outputs:simulationTime", f"{camera_name}_TfPublisher.inputs:timeStamp"),
                ("OnTick.outputs:tick", f"{camera_name}_TfPublisher.inputs:execIn"),

            ],
        },
    )

    ##* ##########################

    set_targets(
        prim=stage.GetPrimAtPath(cfg.tf.action_graph_path + f"/{camera_name}_TfPublisher"),
        attribute="inputs:parentPrim",
        target_prim_paths=[f"{cfg.husky_stage_path}/fence_link"],
    )
    set_targets(
        prim=stage.GetPrimAtPath(cfg.tf.action_graph_path + f"/{camera_name}_TfPublisher"),
        attribute="inputs:targetPrims",
        target_prim_paths=[cfg.cameras[camera_name].stage_path],
    )
    simulation_app.update()
    
    controller.evaluate_sync(graph)


def setup_tf_graph(cfg: Config, simulation_app: SimulationApp, stage: Stage):
    '''Setup the action graph for publishing Husky and LIDAR tf transforms to ROS2'''
    try:
        og.Controller.edit(
            {"graph_path": cfg.tf.action_graph_path, "evaluator_name": "execution"},
            {
                og.Controller.Keys.CREATE_NODES: [
                    ##* TF Tree
                    ("OnTick", "omni.graph.action.OnTick"),
                    ("PublishClock", "omni.isaac.ros2_bridge.ROS2PublishClock"),
                    ("rosContext", "omni.isaac.ros2_bridge.ROS2Context"),
                    ("ReadSimTime", "omni.isaac.core_nodes.IsaacReadSimulationTime"),
                    # husky
                    ("tfPublisher", "omni.isaac.ros2_bridge.ROS2PublishTransformTree"),
                    # lidar
                    ("lidarTfPublisher", "omni.isaac.ros2_bridge.ROS2PublishTransformTree"),
                    # ur5
                    ("ur5TfPublisher", "omni.isaac.ros2_bridge.ROS2PublishTransformTree"),
                ],
                og.Controller.Keys.CONNECT: [
                    ("OnTick.outputs:tick", "PublishClock.inputs:execIn"),
                    ("rosContext.outputs:context", "PublishClock.inputs:context"),

                    ("OnTick.outputs:tick", "tfPublisher.inputs:execIn"),
                    ("OnTick.outputs:tick", "lidarTfPublisher.inputs:execIn"),
                    ("OnTick.outputs:tick", "ur5TfPublisher.inputs:execIn"),

                    ("rosContext.outputs:context", "tfPublisher.inputs:context"),
                    ("rosContext.outputs:context", "lidarTfPublisher.inputs:context"),
                    ("rosContext.outputs:context", "ur5TfPublisher.inputs:context"),
                    
                    ("ReadSimTime.outputs:simulationTime", "tfPublisher.inputs:timeStamp"),
                    ("ReadSimTime.outputs:simulationTime", "lidarTfPublisher.inputs:timeStamp"),
                    ("ReadSimTime.outputs:simulationTime", "ur5TfPublisher.inputs:timeStamp"),

                    ("ReadSimTime.outputs:simulationTime", "PublishClock.inputs:timeStamp"),
                ],
                # og.Controller.Keys.SET_VALUES: [],
            },
        )
    except Exception as e:
        print(e)

    ##* TF Tree
    set_targets(
        prim=stage.GetPrimAtPath(cfg.tf.action_graph_path + "/tfPublisher"),
        attribute="inputs:targetPrims",
        target_prim_paths=[cfg.husky_stage_path],
    )
    set_targets(
        prim=stage.GetPrimAtPath(cfg.tf.action_graph_path + "/lidarTfPublisher"),
        attribute="inputs:parentPrim",
        target_prim_paths=[f"{cfg.husky_stage_path}/fence_link"],
    )
    set_targets(
        prim=stage.GetPrimAtPath(cfg.tf.action_graph_path + "/lidarTfPublisher"),
        attribute="inputs:targetPrims",
        target_prim_paths=[cfg.lidar.lidar_stage_path],
    )
    set_targets(
        prim=stage.GetPrimAtPath(cfg.tf.action_graph_path + "/ur5TfPublisher"),
        attribute="inputs:parentPrim",
        target_prim_paths=[f"{cfg.husky_stage_path}/put_ur5"],
    )
    set_targets(
        prim=stage.GetPrimAtPath(cfg.tf.action_graph_path + "/ur5TfPublisher"),
        attribute="inputs:targetPrims",
        target_prim_paths=[cfg.ur5_stage_path],
    )

    simulation_app.update()

    for camera_name in cfg.cameras.cameras_list:
        setup_camera_tf_graph(cfg, simulation_app, stage, controller, camera_name)
