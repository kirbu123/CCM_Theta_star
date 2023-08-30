from pxr.Usd import Stage
from pxr import UsdGeom
import omni.graph.core as og
from omni.isaac.kit import SimulationApp
from omni.isaac.core.utils.prims import set_targets

from src.config import Config


def setup_cameras(cfg: Config, 
                  simulation_app: SimulationApp,
                  stage: Stage,
                  cameras_list: list = None):

    if cameras_list is None:
        cameras_list = cfg.cameras.cameras_list

    graph_controller = setup_cameras_graph(cfg, simulation_app)

    for camera_name in cameras_list:

        zed_left_camera_prim =  UsdGeom.Camera(stage.GetPrimAtPath(cfg.cameras.zed.stage_path))
        # zed_left_camera_prim = stage.GetPrimAtPath('/World/Husky_Robot/fence_link/zed/husky_front_right')
        zed_left_camera_prim.GetHorizontalApertureAttr().Set(cfg.cameras[camera_name].horizontal_aperture)
        zed_left_camera_prim.GetVerticalApertureAttr().Set(cfg.cameras[camera_name].vertical_aperture)
        zed_left_camera_prim.GetProjectionAttr().Set(cfg.cameras[camera_name].projection_type)
        zed_left_camera_prim.GetFocalLengthAttr().Set(cfg.cameras[camera_name].focal_length)
        # zed_left_camera_prim.GetFocusDistanceAttr().Set(400)
        simulation_app.update()

        setup_camera_graph(cfg, simulation_app, stage, graph_controller, camera_name)


def setup_cameras_graph(cfg: Config,
                        simulation_app: SimulationApp) -> og.Controller:
    keys = og.Controller.Keys
    controller = og.Controller(graph_id ="ros_cameras_graph")

    (graph, _, _, _) = controller.edit(
        {
            "graph_path": cfg.cameras.action_graph_stage_path,
            "evaluator_name": "execution",
            # "pipeline_stage": og.GraphPipelineStage.GRAPH_PIPELINE_STAGE_ONDEMAND,
        },
        {
            keys.CREATE_NODES: [
                ("OnTick", "omni.graph.action.OnPlaybackTick"),
            ]
        }
    )

    simulation_app.update()
    return controller

def setup_camera_graph(cfg: Config,
                       simulation_app: SimulationApp,
                       stage: Stage,
                       controller: og.Controller,
                       camera_name: str):
    '''Setup the action graph for publishing Images, Depths and CameraInfo to ROS2'''

    keys = og.Controller.Keys
    graph = og.get_graph_by_path(cfg.cameras.action_graph_stage_path)
    # controller = og.Controller(graph_id=cfg.cameras.action_graph_id)
    controller.edit(
        graph,
        {
            keys.CREATE_NODES: [
                # ("OnTick", "omni.graph.action.OnPlaybackTick"),
                (f"create_{camera_name}Viewport", "omni.isaac.core_nodes.IsaacCreateViewport"),
                (f"get_{camera_name}RenderProduct", "omni.isaac.core_nodes.IsaacGetViewportRenderProduct"),
                (f"set_{camera_name}Camera", "omni.isaac.core_nodes.IsaacSetCameraOnRenderProduct"),
                (f"camera_{camera_name}HelperRgb", "omni.isaac.ros2_bridge.ROS2CameraHelper"),
                (f"camera_{camera_name}HelperInfo", "omni.isaac.ros2_bridge.ROS2CameraHelper"),
                (f"camera_{camera_name}HelperDepth", "omni.isaac.ros2_bridge.ROS2CameraHelper"),
            ],
            keys.CONNECT: [
                ("OnTick.outputs:tick", f"create_{camera_name}Viewport.inputs:execIn"),
                (f"create_{camera_name}Viewport.outputs:execOut", f"get_{camera_name}RenderProduct.inputs:execIn"),
                (f"create_{camera_name}Viewport.outputs:viewport", f"get_{camera_name}RenderProduct.inputs:viewport"),
                (f"get_{camera_name}RenderProduct.outputs:execOut", f"set_{camera_name}Camera.inputs:execIn"),
                (f"get_{camera_name}RenderProduct.outputs:renderProductPath", f"set_{camera_name}Camera.inputs:renderProductPath"),
                (f"set_{camera_name}Camera.outputs:execOut", f"camera_{camera_name}HelperRgb.inputs:execIn"),
                (f"set_{camera_name}Camera.outputs:execOut", f"camera_{camera_name}HelperInfo.inputs:execIn"),
                (f"set_{camera_name}Camera.outputs:execOut", f"camera_{camera_name}HelperDepth.inputs:execIn"),
                (f"get_{camera_name}RenderProduct.outputs:renderProductPath", f"camera_{camera_name}HelperRgb.inputs:renderProductPath"),
                (f"get_{camera_name}RenderProduct.outputs:renderProductPath", f"camera_{camera_name}HelperInfo.inputs:renderProductPath"),
                (f"get_{camera_name}RenderProduct.outputs:renderProductPath", f"camera_{camera_name}HelperDepth.inputs:renderProductPath"),

            ],
            keys.SET_VALUES: [
                (f"create_{camera_name}Viewport.inputs:viewportId", cfg.cameras.cameras_list.index(camera_name)),
                (f"camera_{camera_name}HelperRgb.inputs:frameId", f"{camera_name}_sim_camera"),
                (f"camera_{camera_name}HelperRgb.inputs:topicName", f"{camera_name}_rgb"),
                (f"camera_{camera_name}HelperRgb.inputs:type", "rgb"),
                (f"camera_{camera_name}HelperInfo.inputs:frameId", "sim_camera"),
                (f"camera_{camera_name}HelperInfo.inputs:topicName", f"{camera_name}_camera_info"),
                (f"camera_{camera_name}HelperInfo.inputs:type", "camera_info"),
                (f"camera_{camera_name}HelperDepth.inputs:frameId", f"{camera_name}_sim_camera"),
                (f"camera_{camera_name}HelperDepth.inputs:topicName", f"{camera_name}_depth"),
                (f"camera_{camera_name}HelperDepth.inputs:type", "depth"),
            ],
        },
    )

    ##* ##########################

    set_targets(
        prim=stage.GetPrimAtPath(cfg.cameras.action_graph_stage_path + f"/set_{camera_name}Camera"),
        attribute="inputs:cameraPrim",
        target_prim_paths=[cfg.cameras[camera_name].stage_path],
    )
    simulation_app.update()
    
    controller.evaluate_sync(graph)