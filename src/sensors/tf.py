import omni.graph.core as og
from omni.isaac.core.utils.prims import set_targets
from omni.isaac.kit import SimulationApp
from pxr.Usd import Stage

from src.config import Config


def setup_camera_tf_graph(
    cfg: Config, simulation_app: SimulationApp, stage: Stage, controller: og.Controller, camera_name: str
):
    """Setup the action graph for publishing Images, Depths and CameraInfo to ROS1/2"""

    keys = og.Controller.Keys
    graph = og.get_graph_by_path(cfg.tf.action_graph_path)

    ros_bridge = cfg.ros_cfg[cfg.ros].ros_bridge_extension.split("-")[0]
    ros_v = cfg.ros_cfg[cfg.ros].ros_v

    controller.edit(
        graph,
        {
            keys.CREATE_NODES: [
                (f"{camera_name}_TfPublisher", f"{ros_bridge}.ROS{ros_v}PublishTransformTree"),
            ],
            keys.CONNECT: [
                ("ReadSimTime.outputs:simulationTime", f"{camera_name}_TfPublisher.inputs:timeStamp"),
                ("OnTick.outputs:tick", f"{camera_name}_TfPublisher.inputs:execIn"),
            ],
        },
    )

    # * ##########################

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
    """Setup the action graph for publishing Husky and LIDAR tf transforms to ROS1/2"""

    ros_bridge = cfg.ros_cfg[cfg.ros].ros_bridge_extension.split("-")[0]
    ros_v = cfg.ros_cfg[cfg.ros].ros_v

    controller = og.Controller(graph_id="ros_tf_graph")

    (graph, _, _, _) = controller.edit(
        {"graph_path": cfg.tf.action_graph_path, "evaluator_name": "execution"},
        {
            og.Controller.Keys.CREATE_NODES: [
                # * TF Tree
                ("OnTick", "omni.graph.action.OnTick"),
                ("PublishClock", f"{ros_bridge}.ROS{ros_v}PublishClock"),
                ("ReadSimTime", "omni.isaac.core_nodes.IsaacReadSimulationTime"),
                # husky
                ("tfPublisher", f"{ros_bridge}.ROS{ros_v}PublishTransformTree"),
                # lidar
                ("lidarTfPublisher", f"{ros_bridge}.ROS{ros_v}PublishTransformTree"),
                # ur5
                ("ur5TfPublisher", f"{ros_bridge}.ROS{ros_v}PublishTransformTree"),
            ],
            og.Controller.Keys.CONNECT: [
                ("OnTick.outputs:tick", "PublishClock.inputs:execIn"),
                ("OnTick.outputs:tick", "tfPublisher.inputs:execIn"),
                ("OnTick.outputs:tick", "lidarTfPublisher.inputs:execIn"),
                ("OnTick.outputs:tick", "ur5TfPublisher.inputs:execIn"),
                ("ReadSimTime.outputs:simulationTime", "tfPublisher.inputs:timeStamp"),
                ("ReadSimTime.outputs:simulationTime", "lidarTfPublisher.inputs:timeStamp"),
                ("ReadSimTime.outputs:simulationTime", "ur5TfPublisher.inputs:timeStamp"),
                ("ReadSimTime.outputs:simulationTime", "PublishClock.inputs:timeStamp"),
            ],
        },
    )

    if ros_v == 2:
        controller.edit(
            graph,
            {
                og.Controller.Keys.CREATE_NODES: [
                    ("rosContext", f"{ros_bridge}.ROS{ros_v}Context"),
                ],
                og.Controller.Keys.CONNECT: [
                    ("rosContext.outputs:context", "PublishClock.inputs:context"),
                    ("rosContext.outputs:context", "tfPublisher.inputs:context"),
                    ("rosContext.outputs:context", "ur5TfPublisher.inputs:context"),
                ],
            },
        )

    # * TF Tree


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
    '''''
    
    set_targets(
        prim=stage.GetPrimAtPath(cfg.tf.action_graph_path + "/tfPublisher"),
        attribute="inputs:parentPrim",
        target_prim_paths=[f"{cfg.husky_stage_path}/base_link"],
    )
    set_targets(
        prim=stage.GetPrimAtPath(cfg.tf.action_graph_path + "/tfPublisher"),
        attribute="inputs:targetPrims",
        target_prim_paths=[f"{cfg.husky_stage_path}"],
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
    '''''

    simulation_app.update()

    for camera_name in cfg.cameras.cameras_list:
        setup_camera_tf_graph(cfg, simulation_app, stage, controller, camera_name)
