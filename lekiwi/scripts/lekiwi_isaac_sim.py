#!/usr/bin/env python3
import argparse
import math
import os
import sys
import time

from isaacsim import SimulationApp


def _str_to_bool(value: str) -> bool:
    return str(value).strip().lower() in ("1", "true", "yes", "y")


def _parse_wheel_velocity_arg(raw_value: str):
    text = str(raw_value).strip()
    if text == "":
        return None

    parts = [p.strip() for p in text.split(",") if p.strip() != ""]
    if len(parts) == 1:
        v = float(parts[0])
        return [v, v, v]
    if len(parts) == 3:
        return [float(parts[0]), float(parts[1]), float(parts[2])]
    raise ValueError(
        "--wheel-direct-velocity must be empty, one value ('3.0') or three values ('3.0,3.0,3.0')"
    )


def main() -> int:
    # Keep defaults internal so launch stays minimal.
    ground_z = 0.0
    physics_fps = 240.0
    physics_substeps = 2
    physics_min_position_iterations = 8
    physics_max_position_iterations = 32
    physics_min_velocity_iterations = 2
    physics_max_velocity_iterations = 8

    parser = argparse.ArgumentParser(description="LeKiwi Isaac Sim URDF runner")
    parser.add_argument("--urdf", required=True, help="Absolute path to LeKiwi URDF")
    parser.add_argument("--headless", default="false", help="Run Isaac Sim headless [true/false]")
    parser.add_argument("--renderer", default="RaytracedLighting", help="Renderer backend")
    parser.add_argument("--robot-name", default="lekiwi", help="Robot name label")
    parser.add_argument("--x", type=float, default=0.0, help="Spawn X [m]")
    parser.add_argument("--y", type=float, default=0.0, help="Spawn Y [m]")
    parser.add_argument("--z", type=float, default=0.0, help="Spawn Z [m]")
    parser.add_argument("--roll", type=float, default=0.0, help="Spawn roll [rad]")
    parser.add_argument("--pitch", type=float, default=0.0, help="Spawn pitch [rad]")
    parser.add_argument("--yaw", type=float, default=0.0, help="Spawn yaw [rad]")
    parser.add_argument("--fix-base", default="false", help="Fix base link [true/false]")
    parser.add_argument("--merge-fixed-joints", default="false", help="Merge fixed joints [true/false]")
    parser.add_argument("--convex-decomp", default="false", help="Use convex decomposition [true/false]")
    parser.add_argument("--distance-scale", type=float, default=1.0, help="URDF distance scale")
    parser.add_argument(
        "--wheel-direct-velocity",
        default="",
        help="Bypass ROS wheel commands and drive wheels directly in Isaac. "
        "Format: '' (disabled), '3.0' (all wheels), or '3.0,3.0,3.0' "
        "(joint7,joint8,joint9) in rad/s",
    )
    parser.add_argument(
        "--ros-package-path",
        default="",
        help="ROS_PACKAGE_PATH prefix to resolve package:// URDF meshes",
    )
    parser.add_argument(
        "--debug-disable-roller-collisions",
        default="false",
        help="Debug: disable collision shapes for omni3 roller links [true/false]",
    )
    parser.add_argument(
        "--debug-disable-rim-collisions",
        default="false",
        help="Debug: disable collision shapes for omni3 rim links [true/false]",
    )
    parser.add_argument(
        "--debug-disable-all-wheel-collisions",
        default="false",
        help="Debug: disable collision shapes for all wheel bodies (roller + rim + legacy single-body) [true/false]",
    )
    parser.add_argument(
        "--startup-brake-seconds",
        type=float,
        default=0.0,
        help="Duration [s] after pressing Play where base/body velocities and wheel targets are clamped to zero. Set >0 to enable.",
    )

    args = parser.parse_args()
    wheel_joint_order = ["joint7", "joint8", "joint9"]
    startup_brake_seconds = max(0.0, float(args.startup_brake_seconds))
    debug_disable_roller_collisions = _str_to_bool(args.debug_disable_roller_collisions)
    debug_disable_rim_collisions = _str_to_bool(args.debug_disable_rim_collisions)
    debug_disable_all_wheel_collisions = _str_to_bool(
        args.debug_disable_all_wheel_collisions
    )
    try:
        direct_wheel_velocity = _parse_wheel_velocity_arg(args.wheel_direct_velocity)
    except ValueError as exc:
        print(str(exc), file=sys.stderr)
        return 2

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

    enable_ros_features = direct_wheel_velocity is None
    app = omni.kit.app.get_app()
    em = app.get_extension_manager()
    if enable_ros_features:
        em.set_extension_enabled_immediate("isaacsim.ros2.bridge", True)
        em.set_extension_enabled_immediate("isaacsim.core.nodes", True)

    status, import_config = omni.kit.commands.execute("URDFCreateImportConfig")
    if not status:
        print("Failed to create URDF import config", file=sys.stderr)
        kit.close()
        return 1

    import_config.merge_fixed_joints = _str_to_bool(args.merge_fixed_joints)
    import_config.convex_decomp = _str_to_bool(args.convex_decomp)
    import_config.import_inertia_tensor = True
    # Keep imported links from internally colliding by default.
    # We also enforce this later on the PhysX articulation API.
    import_config.self_collision = False
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
    import carb

    # Improve contact stability for omni-wheel rollers with fixed timestep + substeps.
    carb_settings = carb.settings.get_settings()
    carb_settings.set("/app/player/useFixedTimeStepping", True)
    carb_settings.set("/app/player/timelineSubsampleRate", physics_substeps)

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
    physx_scene.CreateTimeStepsPerSecondAttr().Set(physics_fps)
    physx_scene.CreateMinPositionIterationCountAttr().Set(
        physics_min_position_iterations
    )
    physx_scene.CreateMaxPositionIterationCountAttr().Set(
        physics_max_position_iterations
    )
    physx_scene.CreateMinVelocityIterationCountAttr().Set(
        physics_min_velocity_iterations
    )
    physx_scene.CreateMaxVelocityIterationCountAttr().Set(
        physics_max_velocity_iterations
    )
    print(
        "PhysX tuning: "
        f"fps={physics_fps}, substeps={physics_substeps}, "
        f"pos_iters=[{physics_min_position_iterations},{physics_max_position_iterations}], "
        f"vel_iters=[{physics_min_velocity_iterations},{physics_max_velocity_iterations}]"
    )

    # Ground plane
    PhysicsSchemaTools.addGroundPlane(
        stage, "/groundPlane", "Z", 1500, Gf.Vec3f(0, 0, ground_z), Gf.Vec3f(0.5)
    )

    # Basic lighting
    light = UsdLux.DistantLight.Define(stage, Sdf.Path("/DistantLight"))
    light.CreateIntensityAttr(500)

    if prim_path:
        wheel_target_velocity_attrs = {}
        wheel_target_position_attrs = {}
        wheel_target_positions_deg = {}
        wheel_visual_spin_ops = []
        wheel_visual_spin_angles_deg = {}
        revolute_joint_names = []
        # Apply spawn pose to the robot root xform and auto-correct Z so args.z is the
        # ground-contact height of the lowest robot point (z=0 => starts on the ground).
        prim_path_sdf = Sdf.Path(prim_path)
        parent_path = prim_path_sdf.GetParentPath()
        spawn_prim = stage.GetPrimAtPath(str(parent_path))
        if not spawn_prim.IsValid():
            spawn_prim = stage.GetPrimAtPath(prim_path)
        if spawn_prim.IsValid():
            xformable = UsdGeom.Xformable(spawn_prim)

            cr = math.cos(args.roll * 0.5)
            sr = math.sin(args.roll * 0.5)
            cp = math.cos(args.pitch * 0.5)
            sp = math.sin(args.pitch * 0.5)
            cy = math.cos(args.yaw * 0.5)
            sy = math.sin(args.yaw * 0.5)
            quat = Gf.Quatd(
                (cr * cp * cy) + (sr * sp * sy),
                Gf.Vec3d(
                    (sr * cp * cy) - (cr * sp * sy),
                    (cr * sp * cy) + (sr * cp * sy),
                    (cr * cp * sy) - (sr * sp * cy),
                ),
            )

            def _set_xform_op(op_type, value):
                for op in xformable.GetOrderedXformOps():
                    if op.GetOpType() == op_type:
                        op.Set(value)
                        return
                if op_type == UsdGeom.XformOp.TypeTranslate:
                    xformable.AddTranslateOp().Set(value)
                elif op_type == UsdGeom.XformOp.TypeOrient:
                    xformable.AddOrientOp().Set(value)

            initial_t = Gf.Vec3d(float(args.x), float(args.y), float(args.z))
            _set_xform_op(UsdGeom.XformOp.TypeOrient, quat)
            _set_xform_op(UsdGeom.XformOp.TypeTranslate, initial_t)

            bbox_cache = UsdGeom.BBoxCache(Usd.TimeCode.Default(), [UsdGeom.Tokens.default_])
            bounds_before = bbox_cache.ComputeWorldBound(spawn_prim).ComputeAlignedRange()
            min_before = bounds_before.GetMin()
            max_before = bounds_before.GetMax()
            min_z_before = float(min_before[2])
            max_z_before = float(max_before[2])
            z_correction = 0.0
            if math.isfinite(min_z_before) and math.isfinite(max_z_before) and max_z_before >= min_z_before:
                # Keep requested x/y and ensure the lowest point of the robot is exactly at args.z.
                z_correction = float(args.z) - min_z_before

            final_t = Gf.Vec3d(float(args.x), float(args.y), float(args.z) + z_correction)
            _set_xform_op(UsdGeom.XformOp.TypeTranslate, final_t)

            bbox_cache.Clear()
            bounds_after = bbox_cache.ComputeWorldBound(spawn_prim).ComputeAlignedRange()
            min_after = bounds_after.GetMin()
            min_z_after = float(min_after[2])

            world_tf = xformable.ComputeLocalToWorldTransform(Usd.TimeCode.Default())
            world_t = world_tf.ExtractTranslation()
            print(
                "Spawn pose applied on prim "
                f"{spawn_prim.GetPath()}: x={float(world_t[0]):.4f}, y={float(world_t[1]):.4f}, "
                f"z={float(world_t[2]):.4f} "
                f"(input z={args.z:.4f}, auto_correction={z_correction:.4f}, "
                f"min_z_before={min_z_before:.4f}, min_z_after={min_z_after:.4f})"
            )

        def _get_robot_search_root(stage, robot_prim_path: str):
            root_path = Sdf.Path(robot_prim_path)
            parent_path = root_path.GetParentPath()
            if parent_path and str(parent_path) != "":
                parent_prim = stage.GetPrimAtPath(parent_path)
                if parent_prim.IsValid():
                    return parent_prim
            return stage.GetPrimAtPath(robot_prim_path)

        def _find_base_link_prim_path(stage, robot_prim_path: str):
            root = _get_robot_search_root(stage, robot_prim_path)
            if not root.IsValid():
                return ""
            for prim in Usd.PrimRange(root):
                if prim.GetName().lower() == "base_link":
                    return str(prim.GetPath())
            return ""

        def _force_disable_articulation_self_collision(stage, robot_prim_path: str):
            import carb

            root = _get_robot_search_root(stage, robot_prim_path)
            if not root.IsValid():
                carb.log_warn(
                    "Self-collision hard-disable skipped: robot search root is invalid"
                )
                return

            updated_paths = []
            for prim in Usd.PrimRange(root):
                if not prim.HasAPI(UsdPhysics.ArticulationRootAPI):
                    continue
                prim_path = str(prim.GetPath())
                try:
                    art_api = PhysxSchema.PhysxArticulationAPI.Apply(prim)
                    attr = art_api.GetEnabledSelfCollisionsAttr()
                    if not attr.IsValid():
                        attr = art_api.CreateEnabledSelfCollisionsAttr()
                    attr.Set(False)
                    updated_paths.append(prim_path)
                except Exception as exc:
                    carb.log_warn(
                        "Failed to hard-disable self-collision on "
                        + prim_path
                        + ": "
                        + str(exc)
                    )

            if updated_paths:
                carb.log_warn(
                    "Forced articulation self-collision OFF on: "
                    + ", ".join(updated_paths)
                )
            else:
                carb.log_warn(
                    "No articulation root prim found for self-collision hard-disable"
                )

        def _zero_robot_body_velocities(stage, robot_prim_path: str):
            root = _get_robot_search_root(stage, robot_prim_path)
            if not root.IsValid():
                return
            for prim in Usd.PrimRange(root):
                if not prim.HasAPI(UsdPhysics.RigidBodyAPI):
                    continue
                rb = UsdPhysics.RigidBodyAPI.Apply(prim)
                v_attr = rb.GetVelocityAttr()
                if not v_attr.IsValid():
                    v_attr = rb.CreateVelocityAttr()
                v_attr.Set(Gf.Vec3f(0.0, 0.0, 0.0))
                w_attr = rb.GetAngularVelocityAttr()
                if not w_attr.IsValid():
                    w_attr = rb.CreateAngularVelocityAttr()
                w_attr.Set(Gf.Vec3f(0.0, 0.0, 0.0))

        def _set_debug_collision_enabled(
            stage, robot_prim_path: str, name_tokens, enabled: bool, label: str
        ):
            import carb

            root = _get_robot_search_root(stage, robot_prim_path)
            if not root.IsValid():
                carb.log_warn(
                    f"Collision debug '{label}' skipped: robot search root is invalid"
                )
                return

            touched = []
            for prim in Usd.PrimRange(root):
                name_lower = prim.GetName().lower()
                if not any(token in name_lower for token in name_tokens):
                    continue
                if not prim.HasAPI(UsdPhysics.CollisionAPI):
                    continue

                collision_api = UsdPhysics.CollisionAPI.Apply(prim)
                attr = collision_api.GetCollisionEnabledAttr()
                if not attr.IsValid():
                    attr = collision_api.CreateCollisionEnabledAttr()
                attr.Set(bool(enabled))
                touched.append(str(prim.GetPath()))

            carb.log_warn(
                f"Collision debug '{label}': set enabled={enabled} on {len(touched)} prims"
            )

        def _build_ros2_control_graph(
            stage,
            robot_prim_path: str,
            enable_ros_wheel_commands: bool,
        ):
            old_graph_path = f"{robot_prim_path}/ros2_control_graph"
            if stage.GetPrimAtPath(old_graph_path).IsValid():
                stage.RemovePrim(old_graph_path)
            graph_path = f"{robot_prim_path}/ros2_control_graph_v2"
            if stage.GetPrimAtPath(graph_path).IsValid():
                stage.RemovePrim(graph_path)

            create_nodes = [
                ("tick", "omni.graph.action.OnPlaybackTick"),
                ("sim_time", "isaacsim.core.nodes.IsaacReadSimulationTime"),
                ("pub_js", "isaacsim.ros2.bridge.ROS2PublishJointState"),
                ("sub_js_arm", "isaacsim.ros2.bridge.ROS2SubscribeJointState"),
                ("art_ctl_arm", "isaacsim.core.nodes.IsaacArticulationController"),
                ("pub_clock", "isaacsim.ros2.bridge.ROS2PublishClock"),
            ]
            connect = [
                ("tick.outputs:tick", "pub_js.inputs:execIn"),
                ("tick.outputs:tick", "sub_js_arm.inputs:execIn"),
                ("tick.outputs:tick", "art_ctl_arm.inputs:execIn"),
                ("tick.outputs:tick", "pub_clock.inputs:execIn"),
                ("sim_time.outputs:simulationTime", "pub_js.inputs:timeStamp"),
                ("sim_time.outputs:simulationTime", "pub_clock.inputs:timeStamp"),
                ("sub_js_arm.outputs:jointNames", "art_ctl_arm.inputs:jointNames"),
                ("sub_js_arm.outputs:positionCommand", "art_ctl_arm.inputs:positionCommand"),
            ]
            set_values = [
                ("pub_js.inputs:nodeNamespace", ""),
                ("pub_js.inputs:topicName", "/topic_based_joint_states"),
                ("pub_js.inputs:targetPrim", robot_prim_path),
                ("sub_js_arm.inputs:nodeNamespace", ""),
                ("sub_js_arm.inputs:topicName", "/isaac_joint_commands_arm"),
                ("art_ctl_arm.inputs:robotPath", robot_prim_path),
                ("pub_clock.inputs:nodeNamespace", ""),
                ("pub_clock.inputs:topicName", "/clock"),
            ]
            if enable_ros_wheel_commands:
                create_nodes.extend(
                    [
                        ("sub_js_wheels", "isaacsim.ros2.bridge.ROS2SubscribeJointState"),
                        ("art_ctl_wheels", "isaacsim.core.nodes.IsaacArticulationController"),
                    ]
                )
                connect.extend(
                    [
                        ("tick.outputs:tick", "sub_js_wheels.inputs:execIn"),
                        ("tick.outputs:tick", "art_ctl_wheels.inputs:execIn"),
                        ("sub_js_wheels.outputs:jointNames", "art_ctl_wheels.inputs:jointNames"),
                        ("sub_js_wheels.outputs:velocityCommand", "art_ctl_wheels.inputs:velocityCommand"),
                    ]
                )
                set_values.extend(
                    [
                        ("sub_js_wheels.inputs:nodeNamespace", ""),
                        ("sub_js_wheels.inputs:topicName", "/isaac_joint_commands_wheels"),
                        ("art_ctl_wheels.inputs:robotPath", robot_prim_path),
                    ]
                )

            og.Controller.edit(
                {"graph_path": graph_path, "evaluator_name": "execution"},
                {
                    og.Controller.Keys.CREATE_NODES: create_nodes,
                    og.Controller.Keys.CONNECT: connect,
                    og.Controller.Keys.SET_VALUES: set_values,
                },
            )
            import carb

            carb.log_warn(f"LeKiwi Isaac script: {__file__}")
            carb.log_warn(f"ROS2 control graph created at {graph_path}")
            if enable_ros_wheel_commands:
                carb.log_warn(
                    "ROS2 joint command topics: /isaac_joint_commands_wheels, /isaac_joint_commands_arm"
                )
            else:
                carb.log_warn("ROS2 wheel topic disabled; wheel velocities are driven directly in Isaac")

        def _tune_joint_drives(stage, robot_prim_path: str):
            wheel_joints = {"joint7", "joint8", "joint9"}
            # Keep wheel joints in velocity-drive mode (no position spring pullback).
            wheel_stiffness = 0.0
            wheel_damping = 2.0e3
            arm_joints = {
                "STS3215_03a_v1_Revolute_45",
                "STS3215_03a_v1_1_Revolute_49",
                "STS3215_03a_v1_2_Revolute_51",
                "STS3215_03a_v1_3_Revolute_53",
                "STS3215_03a_Wrist_Roll_v1_Revolute_55",
                "STS3215_03a_v1_4_Revolute_57",
            }
            arm_stiffness = 1.0e5
            arm_damping = 1.0e3
            max_force = 1.0e6

            root = _get_robot_search_root(stage, robot_prim_path)
            if not root.IsValid():
                return

            for prim in Usd.PrimRange(root):
                if prim.IsA(UsdPhysics.RevoluteJoint):
                    name = prim.GetName()
                    revolute_joint_names.append(name)
                    drive = UsdPhysics.DriveAPI.Apply(prim, "angular")
                    if name in wheel_joints:
                        if direct_wheel_velocity is not None:
                            # In direct mode, use a position ramp for robust visible rotation.
                            drive.GetStiffnessAttr().Set(5.0e4)
                            drive.GetDampingAttr().Set(2.0e3)
                        else:
                            drive.GetStiffnessAttr().Set(wheel_stiffness)
                            drive.GetDampingAttr().Set(wheel_damping)
                        if direct_wheel_velocity is not None:
                            wheel_idx = wheel_joint_order.index(name)
                            target_velocity_rad_s = float(
                                direct_wheel_velocity[wheel_idx]
                            )
                            target_velocity_deg_s = math.degrees(
                                target_velocity_rad_s
                            )
                            target_velocity_attr = drive.GetTargetVelocityAttr()
                            if not target_velocity_attr.IsValid():
                                target_velocity_attr = drive.CreateTargetVelocityAttr()
                            target_velocity_attr.Set(target_velocity_deg_s)
                            wheel_target_velocity_attrs[name] = target_velocity_attr
                            target_position_attr = drive.GetTargetPositionAttr()
                            if not target_position_attr.IsValid():
                                target_position_attr = drive.CreateTargetPositionAttr()
                            current_target_position = target_position_attr.Get()
                            if current_target_position is None:
                                current_target_position = 0.0
                            target_position_attr.Set(float(current_target_position))
                            wheel_target_position_attrs[name] = target_position_attr
                            wheel_target_positions_deg[name] = float(
                                current_target_position
                            )
                        else:
                            target_velocity_attr = drive.GetTargetVelocityAttr()
                            if not target_velocity_attr.IsValid():
                                target_velocity_attr = drive.CreateTargetVelocityAttr()
                            target_velocity_attr.Set(0.0)
                            wheel_target_velocity_attrs[name] = target_velocity_attr
                    elif name in arm_joints:
                        drive.GetStiffnessAttr().Set(arm_stiffness)
                        drive.GetDampingAttr().Set(arm_damping)
                    else:
                        continue
                    drive.GetMaxForceAttr().Set(max_force)

        def _setup_wheel_visual_spin(stage, robot_prim_path: str, enabled_joint_names=None):
            root = _get_robot_search_root(stage, robot_prim_path)
            if not root.IsValid():
                return

            for prim in Usd.PrimRange(root):
                name_lower = prim.GetName().lower()
                joint_name = None
                if "omni3_rim_joint" in name_lower:
                    if "joint8" in name_lower:
                        joint_name = "joint8"
                    elif "joint9" in name_lower:
                        joint_name = "joint9"
                    elif "joint7" in name_lower:
                        joint_name = "joint7"
                    else:
                        continue
                elif "omni_directional_wheel_single_body" in name_lower:
                    if "_v1_2" in name_lower:
                        joint_name = "joint8"
                    elif "_v1_1" in name_lower:
                        joint_name = "joint9"
                    else:
                        joint_name = "joint7"
                else:
                    continue
                if not prim.IsA(UsdGeom.Xform):
                    continue

                if enabled_joint_names is not None and joint_name not in enabled_joint_names:
                    continue

                target_prim = prim
                for child in prim.GetChildren():
                    if "visual" in child.GetName().lower() and child.IsA(UsdGeom.Xform):
                        target_prim = child
                        break

                xformable = UsdGeom.Xformable(target_prim)
                spin_op = xformable.AddRotateXYZOp(opSuffix="direct_spin")
                spin_op.Set(Gf.Vec3d(0.0, 0.0, 0.0))
                wheel_visual_spin_ops.append((joint_name, spin_op))
                wheel_visual_spin_angles_deg[spin_op] = 0.0

        _force_disable_articulation_self_collision(stage, prim_path)
        _zero_robot_body_velocities(stage, prim_path)
        if debug_disable_all_wheel_collisions:
            _set_debug_collision_enabled(
                stage,
                prim_path,
                ["omni3_roller_", "omni3_rim_joint", "omni_directional_wheel_single_body"],
                False,
                "all-wheel-collisions",
            )
        else:
            if debug_disable_roller_collisions:
                _set_debug_collision_enabled(
                    stage,
                    prim_path,
                    ["omni3_roller_"],
                    False,
                    "roller-collisions",
                )
            if debug_disable_rim_collisions:
                _set_debug_collision_enabled(
                    stage,
                    prim_path,
                    ["omni3_rim_joint", "omni_directional_wheel_single_body"],
                    False,
                    "rim-collisions",
                )
        _tune_joint_drives(stage, prim_path)
        if enable_ros_features:
            if wheel_target_velocity_attrs:
                print(
                    "Wheel joints configured for startup brake: "
                    + ", ".join(sorted(wheel_target_velocity_attrs.keys()))
                )
            else:
                print(
                    "Warning: no wheel joints detected for startup brake",
                    file=sys.stderr,
                )
        base_tf_prim_path = _find_base_link_prim_path(stage, prim_path)
        if enable_ros_features:
            _build_ros2_control_graph(stage, prim_path, True)
        else:
            print("Direct wheel mode: ROS2 graph disabled")

        if direct_wheel_velocity is not None:
            direct_wheel_velocity_deg = [math.degrees(v) for v in direct_wheel_velocity]
            print(
                "Direct wheel velocity mode enabled "
                "(rad/s -> deg/s, joint7/joint8/joint9): "
                f"{direct_wheel_velocity} -> {direct_wheel_velocity_deg}"
            )
            missing = [j for j in wheel_joint_order if j not in wheel_target_velocity_attrs]
            if missing:
                print(
                    "Warning: some wheel joints were not found for direct velocity mode: "
                    + ", ".join(missing),
                    file=sys.stderr,
                )
                # Visual spin is only a fallback for wheel joints that were not detected as physics joints.
                _setup_wheel_visual_spin(stage, prim_path, set(missing))
            print(
                "Wheel joints found in stage: "
                + ", ".join(sorted(wheel_target_velocity_attrs.keys()))
            )
            print(
                "All revolute joints seen in stage: "
                + ", ".join(sorted(set(revolute_joint_names)))
            )
            print(
                "Wheel visual spin prims attached: "
                + str(len(wheel_visual_spin_ops))
            )
    timeline = omni.timeline.get_timeline_interface()
    timeline.play()

    try:
        last_wall_time = time.perf_counter()
        startup_brake_end_wall_time = None
        while kit.is_running():
            now_wall_time = time.perf_counter()
            if (
                prim_path
                and startup_brake_seconds > 0.0
                and timeline.is_playing()
            ):
                if startup_brake_end_wall_time is None:
                    startup_brake_end_wall_time = (
                        now_wall_time + startup_brake_seconds
                    )
                    print(
                        f"Startup brake active for {startup_brake_seconds:.2f}s after Play"
                    )
                if now_wall_time <= startup_brake_end_wall_time:
                    _zero_robot_body_velocities(stage, prim_path)
                    for velocity_attr in wheel_target_velocity_attrs.values():
                        if velocity_attr is not None and velocity_attr.IsValid():
                            velocity_attr.Set(0.0)

            if direct_wheel_velocity is not None:
                dt_s = max(0.0, min(0.1, now_wall_time - last_wall_time))
                last_wall_time = now_wall_time
                for joint_name, target_velocity_rad_s in zip(
                    wheel_joint_order, direct_wheel_velocity
                ):
                    velocity_attr = wheel_target_velocity_attrs.get(joint_name)
                    if velocity_attr is not None and velocity_attr.IsValid():
                        target_velocity_deg_s = math.degrees(
                            float(target_velocity_rad_s)
                        )
                        velocity_attr.Set(target_velocity_deg_s)
                        position_attr = wheel_target_position_attrs.get(joint_name)
                        if position_attr is not None and position_attr.IsValid():
                            wheel_target_positions_deg[joint_name] += (
                                target_velocity_deg_s * dt_s
                            )
                            position_attr.Set(
                                float(wheel_target_positions_deg[joint_name])
                            )
                for joint_name, spin_op in wheel_visual_spin_ops:
                    try:
                        wheel_idx = wheel_joint_order.index(joint_name)
                        target_velocity_deg_s = math.degrees(
                            float(direct_wheel_velocity[wheel_idx])
                        )
                        wheel_visual_spin_angles_deg[spin_op] += (
                            target_velocity_deg_s * dt_s
                        )
                        spin_op.Set(
                            Gf.Vec3d(
                                0.0,
                                0.0,
                                float(wheel_visual_spin_angles_deg[spin_op]),
                            )
                        )
                    except Exception:
                        pass
            else:
                last_wall_time = now_wall_time
            kit.update()
    finally:
        timeline.stop()
        kit.close()

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
