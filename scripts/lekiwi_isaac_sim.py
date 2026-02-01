#!/usr/bin/env python3
import argparse
import math
import os
import sys

from isaacsim import SimulationApp


def _str_to_bool(value: str) -> bool:
    return str(value).strip().lower() in ("1", "true", "yes", "y")


def main() -> int:
    parser = argparse.ArgumentParser(description="LeKiwi Isaac Sim URDF runner")
    parser.add_argument("--urdf", required=True, help="Absolute path to LeKiwi URDF")
    parser.add_argument("--headless", default="false", help="Run Isaac Sim headless [true/false]")
    parser.add_argument("--renderer", default="RaytracedLighting", help="Renderer backend")
    parser.add_argument("--robot-name", default="lekiwi", help="Robot name label")
    parser.add_argument("--x", type=float, default=0.0, help="Spawn X [m]")
    parser.add_argument("--y", type=float, default=0.0, help="Spawn Y [m]")
    parser.add_argument("--z", type=float, default=0.2, help="Spawn Z [m]")
    parser.add_argument("--roll", type=float, default=0.0, help="Spawn roll [rad]")
    parser.add_argument("--pitch", type=float, default=0.0, help="Spawn pitch [rad]")
    parser.add_argument("--yaw", type=float, default=0.0, help="Spawn yaw [rad]")
    parser.add_argument("--fix-base", default="false", help="Fix base link [true/false]")
    parser.add_argument("--merge-fixed-joints", default="false", help="Merge fixed joints [true/false]")
    parser.add_argument("--convex-decomp", default="false", help="Use convex decomposition [true/false]")
    parser.add_argument("--distance-scale", type=float, default=1.0, help="URDF distance scale")
    parser.add_argument(
        "--ros-package-path",
        default="",
        help="ROS_PACKAGE_PATH prefix to resolve package:// URDF meshes",
    )

    args = parser.parse_args()

    if not os.path.exists(args.urdf):
        print(f"URDF not found: {args.urdf}", file=sys.stderr)
        return 2

    if args.ros_package_path:
        existing = os.environ.get("ROS_PACKAGE_PATH", "")
        if existing:
            os.environ["ROS_PACKAGE_PATH"] = f"{args.ros_package_path}:{existing}"
        else:
            os.environ["ROS_PACKAGE_PATH"] = args.ros_package_path

    headless = _str_to_bool(args.headless)
    kit = SimulationApp({"renderer": args.renderer, "headless": headless})

    import omni.kit.app
    import omni.kit.commands
    import omni.usd
    import omni.graph.core as og
    from pxr import Gf, PhysicsSchemaTools, PhysxSchema, Sdf, Usd, UsdGeom, UsdLux, UsdPhysics

    app = omni.kit.app.get_app()
    em = app.get_extension_manager()
    em.set_extension_enabled_immediate("isaacsim.ros2.bridge", True)
    em.set_extension_enabled_immediate("isaacsim.ros2.nodes", True)
    em.set_extension_enabled_immediate("isaacsim.core.nodes", True)

    status, import_config = omni.kit.commands.execute("URDFCreateImportConfig")
    if not status:
        print("Failed to create URDF import config", file=sys.stderr)
        kit.close()
        return 1

    import_config.merge_fixed_joints = _str_to_bool(args.merge_fixed_joints)
    import_config.convex_decomp = _str_to_bool(args.convex_decomp)
    import_config.import_inertia_tensor = True
    import_config.fix_base = _str_to_bool(args.fix_base)
    import_config.distance_scale = args.distance_scale

    status, prim_path = omni.kit.commands.execute(
        "URDFParseAndImportFile",
        urdf_path=args.urdf,
        import_config=import_config,
        get_articulation_root=True,
    )

    print(f"URDF import status: {status}")
    print(f"Articulation prim path: {prim_path}")

    stage = omni.usd.get_context().get_stage()

    # Physics scene
    scene = UsdPhysics.Scene.Define(stage, Sdf.Path("/physicsScene"))
    scene.CreateGravityDirectionAttr().Set(Gf.Vec3f(0.0, 0.0, -1.0))
    scene.CreateGravityMagnitudeAttr().Set(9.81)

    PhysxSchema.PhysxSceneAPI.Apply(stage.GetPrimAtPath("/physicsScene"))
    physx_scene = PhysxSchema.PhysxSceneAPI.Get(stage, "/physicsScene")
    physx_scene.CreateEnableCCDAttr(True)
    physx_scene.CreateEnableStabilizationAttr(True)
    physx_scene.CreateEnableGPUDynamicsAttr(False)
    physx_scene.CreateBroadphaseTypeAttr("MBP")
    physx_scene.CreateSolverTypeAttr("TGS")

    # Ground plane
    PhysicsSchemaTools.addGroundPlane(
        stage, "/groundPlane", "Z", 1500, Gf.Vec3f(0, 0, -0.25), Gf.Vec3f(0.5)
    )

    # Basic lighting
    light = UsdLux.DistantLight.Define(stage, Sdf.Path("/DistantLight"))
    light.CreateIntensityAttr(500)

    if prim_path:
        prim = stage.GetPrimAtPath(prim_path)
        if prim.IsValid():
            xform = UsdGeom.XformCommonAPI(prim)
            xform.SetTranslate((args.x, args.y, args.z))
            roll_deg = math.degrees(args.roll)
            pitch_deg = math.degrees(args.pitch)
            yaw_deg = math.degrees(args.yaw)
            xform.SetRotate((roll_deg, pitch_deg, yaw_deg), UsdGeom.XformCommonAPI.RotationOrderXYZ)

        def _build_ros2_control_graph(stage, robot_prim_path: str):
            old_graph_path = f"{robot_prim_path}/ros2_control_graph"
            if stage.GetPrimAtPath(old_graph_path).IsValid():
                stage.RemovePrim(old_graph_path)
            graph_path = f"{robot_prim_path}/ros2_control_graph_v2"
            if stage.GetPrimAtPath(graph_path).IsValid():
                stage.RemovePrim(graph_path)

            og.Controller.edit(
                {"graph_path": graph_path, "evaluator_name": "execution"},
                {
                    og.Controller.Keys.CREATE_NODES: [
                        ("tick", "omni.graph.action.OnPlaybackTick"),
                        ("sim_time", "isaacsim.core.nodes.IsaacReadSimulationTime"),
                        ("pub_js", "isaacsim.ros2.bridge.ROS2PublishJointState"),
                        ("sub_js_wheels", "isaacsim.ros2.bridge.ROS2SubscribeJointState"),
                        ("sub_js_arm", "isaacsim.ros2.bridge.ROS2SubscribeJointState"),
                        ("art_ctl_wheels", "isaacsim.core.nodes.IsaacArticulationController"),
                        ("art_ctl_arm", "isaacsim.core.nodes.IsaacArticulationController"),
                        ("pub_clock", "isaacsim.ros2.bridge.ROS2PublishClock"),
                    ],
                    og.Controller.Keys.CONNECT: [
                        ("tick.outputs:tick", "pub_js.inputs:execIn"),
                        ("tick.outputs:tick", "sub_js_wheels.inputs:execIn"),
                        ("tick.outputs:tick", "sub_js_arm.inputs:execIn"),
                        ("tick.outputs:tick", "art_ctl_wheels.inputs:execIn"),
                        ("tick.outputs:tick", "art_ctl_arm.inputs:execIn"),
                        ("tick.outputs:tick", "pub_clock.inputs:execIn"),
                        ("sim_time.outputs:simulationTime", "pub_js.inputs:timeStamp"),
                        ("sim_time.outputs:simulationTime", "pub_clock.inputs:timeStamp"),
                        ("sub_js_wheels.outputs:jointNames", "art_ctl_wheels.inputs:jointNames"),
                        ("sub_js_wheels.outputs:velocityCommand", "art_ctl_wheels.inputs:velocityCommand"),
                        ("sub_js_arm.outputs:jointNames", "art_ctl_arm.inputs:jointNames"),
                        ("sub_js_arm.outputs:positionCommand", "art_ctl_arm.inputs:positionCommand"),
                    ],
                    og.Controller.Keys.SET_VALUES: [
                        ("pub_js.inputs:nodeNamespace", ""),
                        ("pub_js.inputs:topicName", "/topic_based_joint_states"),
                        ("pub_js.inputs:targetPrim", robot_prim_path),
                        ("sub_js_wheels.inputs:nodeNamespace", ""),
                        ("sub_js_wheels.inputs:topicName", "/isaac_joint_commands_wheels"),
                        ("sub_js_arm.inputs:nodeNamespace", ""),
                        ("sub_js_arm.inputs:topicName", "/isaac_joint_commands_arm"),
                        ("art_ctl_wheels.inputs:robotPath", robot_prim_path),
                        ("art_ctl_arm.inputs:robotPath", robot_prim_path),
                        ("pub_clock.inputs:nodeNamespace", ""),
                        ("pub_clock.inputs:topicName", "/clock"),
                    ],
                },
            )
            import carb

            carb.log_warn(f"LeKiwi Isaac script: {__file__}")
            carb.log_warn(f"ROS2 control graph created at {graph_path}")
            carb.log_warn("ROS2 joint command topics: /isaac_joint_commands_wheels, /isaac_joint_commands_arm")

        def _tune_joint_drives(stage, robot_prim_path: str):
            wheel_joints = {"joint7", "joint8", "joint9"}
            wheel_stiffness = 0.0
            wheel_damping = 200.0
            arm_stiffness = 1.0e5
            arm_damping = 1.0e3
            max_force = 1.0e6

            root = stage.GetPrimAtPath(robot_prim_path)
            if not root.IsValid():
                return

            for prim in Usd.PrimRange(root):
                if prim.IsA(UsdPhysics.RevoluteJoint):
                    drive = UsdPhysics.DriveAPI.Apply(prim, "angular")
                    name = prim.GetName()
                    if name in wheel_joints:
                        drive.GetStiffnessAttr().Set(wheel_stiffness)
                        drive.GetDampingAttr().Set(wheel_damping)
                    else:
                        drive.GetStiffnessAttr().Set(arm_stiffness)
                        drive.GetDampingAttr().Set(arm_damping)
                    drive.GetMaxForceAttr().Set(max_force)

        _tune_joint_drives(stage, prim_path)
        _build_ros2_control_graph(stage, prim_path)
    timeline = omni.timeline.get_timeline_interface()
    timeline.play()

    try:
        while kit.is_running():
            kit.update()
    finally:
        timeline.stop()
        kit.close()

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
