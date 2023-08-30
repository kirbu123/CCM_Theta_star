from pxr.Usd import Stage
from pxr import UsdGeom
import omni.graph.core as og
from omni.isaac.kit import SimulationApp
from omni.isaac.core.utils.prims import set_targets

from src.config import Config


def setup_cameras_prims(cfg: Config, simulation_app: SimulationApp, stage: Stage):
    zed_left_camera_prim =  UsdGeom.Camera(stage.GetPrimAtPath(cfg.cameras.zed.stage_path))
    # zed_left_camera_prim = stage.GetPrimAtPath('/World/Husky_Robot/fence_link/zed/husky_front_right')
    zed_left_camera_prim.GetHorizontalApertureAttr().Set(cfg.cameras.zed.horizontal_aperture)
    zed_left_camera_prim.GetVerticalApertureAttr().Set(cfg.cameras.zed.vertical_aperture)
    zed_left_camera_prim.GetProjectionAttr().Set(cfg.cameras.zed.projection_type)
    zed_left_camera_prim.GetFocalLengthAttr().Set(cfg.cameras.zed.focal_length)
    # zed_left_camera_prim.GetFocusDistanceAttr().Set(400)

    simulation_app.update()

def setup_cameras_graph(cfg: Config, simulation_app: SimulationApp, stage: Stage):
    '''Setup the action graph for publishing Images, Depths and CameraInfo to ROS2'''

    keys = og.Controller.Keys
    (ros_camera_graph, _, _, _) = og.Controller.edit(
        {
            "graph_path": cfg.cameras.action_graph_stage_path,
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
            ],
        },
    )

    ##* ##########################

    set_targets(
        prim=stage.GetPrimAtPath(cfg.cameras.action_graph_stage_path + "/setCamera"),
        attribute="inputs:cameraPrim",
        target_prim_paths=[cfg.cameras.zed.stage_path],
    )
    simulation_app.update()
    
    og.Controller.evaluate_sync(ros_camera_graph)