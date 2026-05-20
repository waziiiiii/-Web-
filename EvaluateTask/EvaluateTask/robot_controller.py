"""Clean PyBullet robot controller with FK-based link position computation.

Supports two control modes:
- "precise": resetJointState (deterministic positioning, no physics)
- "motor": setJointMotorControl2 (realistic physics-based motor control)
"""

from __future__ import annotations

import threading
import time
from pathlib import Path

import numpy as np

try:
    import pybullet as p
    import pybullet_data
except ImportError:
    raise SystemExit("pybullet is required. Install with: pip install pybullet")

try:
    import cv2
except ImportError:
    cv2 = None

import config as cfg

ROOT_DIR = Path(__file__).resolve().parent
URDF_PATH = ROOT_DIR / "autolife_desktop" / "urdfs" / "robot_v0_1.urdf"


class RobotController:
    """PyBullet robot controller with background simulation for 60 FPS link positions."""

    def __init__(self, urdf_path: str | Path = URDF_PATH, fps: int = 60):
        self._target_fps = cfg.TARGET_FPS
        self._urdf_path = Path(urdf_path)
        self._control_mode = cfg.DEFAULT_CONTROL_MODE

        # Connect PyBullet
        self._client_id = p.connect(p.DIRECT)
        if self._client_id < 0:
            raise RuntimeError("Failed to connect PyBullet")

        # Setup world
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        p.setGravity(0, 0, cfg.GRAVITY)
        p.setTimeStep(cfg.TIME_STEP)
        p.setRealTimeSimulation(0)
        self._ground_id = p.loadURDF("plane.urdf", physicsClientId=self._client_id)

        # Load robot (with self-collision enabled)
        self._robot_id = p.loadURDF(
            str(self._urdf_path),
            basePosition=cfg.BASE_POSITION,
            useFixedBase=True,
            flags=p.URDF_USE_SELF_COLLISION | p.URDF_USE_SELF_COLLISION_EXCLUDE_PARENT,
            physicsClientId=self._client_id,
        )

        # Collect controllable joints
        self._controllable_joints: list[dict] = []
        num_joints = p.getNumJoints(self._robot_id, physicsClientId=self._client_id)
        for idx in range(num_joints):
            info = p.getJointInfo(self._robot_id, idx, physicsClientId=self._client_id)
            jtype = info[2]
            if jtype == p.JOINT_REVOLUTE:
                self._controllable_joints.append({
                    "index": idx,
                    "name": info[1].decode("utf-8"),
                    "lower": float(info[8]),
                    "upper": float(info[9]),
                })

        # Set home pose
        self._target_angles = list(cfg.HOME_POSE)
        self._apply_pose(cfg.HOME_POSE)

        # Shared state
        self._lock = threading.Lock()
        self._latest_state: dict = {}

        # PyBullet operation lock - ensures IK solver and background thread don't interleave
        self._pybullet_lock = threading.Lock()

        # Camera state (on-demand single-frame capture)
        self._cam_lock = threading.Lock()
        self._latest_rgb_bytes: bytes = b""
        self._latest_depth_bytes: bytes = b""
        self._cam_width = cfg.CAM_WIDTH
        self._cam_height = cfg.CAM_HEIGHT
        self._cam_capturing = False  # True while a capture thread is running
        self._last_capture_error: str = ""  # Error message from last capture attempt
        self._last_save_path: str = ""  # Path where the last frame was saved

        # Workspace objects for PyBullet camera (synced from frontend)
        self._wksp_objects: list[dict] = [dict(obj) for obj in cfg.WORKSPACE_OBJECTS]
        self._wksp_lock = threading.Lock()

        # Collision detection state
        self._collision_lock = threading.Lock()
        self._collisions: list[dict] = []
        self._collision_detected: bool = False
        self._wksp_body_ids: list[int] = []
        self._spawn_collision_bodies()

        # Command queue
        self._cmd_lock = threading.Lock()
        self._pending_cmds: list[tuple[str, dict]] = []

        # Background thread (simulation only — no camera thread)
        self._running = True
        self._thread = threading.Thread(target=self._loop, daemon=True)
        self._thread.start()

    def _apply_pose(self, angles: list[float]) -> None:
        """Set joint positions directly via resetJointState (stable, no gravity drift).
        Used for camera capture and IK verification where deterministic positioning is needed."""
        for i, joint in enumerate(self._controllable_joints):
            a = max(joint["lower"], min(joint["upper"], angles[i]))
            p.resetJointState(
                self._robot_id, joint["index"], a, 0.0,
                physicsClientId=self._client_id,
            )

    def set_control_mode(self, mode: str) -> None:
        """Switch between 'precise' and 'motor' control modes.

        Args:
            mode: "precise" (resetJointState) or "motor" (setJointMotorControl2)
        """
        if mode not in ("precise", "motor"):
            raise ValueError(f"Invalid control mode: {mode}. Use 'precise' or 'motor'.")
        self._control_mode = mode

    def get_control_mode(self) -> str:
        """Return the current control mode."""
        return self._control_mode

    def _apply_motor_control(self, angles: list[float]) -> None:
        """Set joint target positions via setJointMotorControl2 (POSITION_CONTROL).
        Uses the physics engine's motor controller for realistic joint movement."""
        for i, joint in enumerate(self._controllable_joints):
            a = max(joint["lower"], min(joint["upper"], angles[i]))
            p.setJointMotorControl2(
                bodyIndex=self._robot_id,
                jointIndex=joint["index"],
                controlMode=p.POSITION_CONTROL,
                targetPosition=a,
                force=cfg.MOTOR_FORCE,
                positionGain=cfg.MOTOR_POSITION_GAIN,
                velocityGain=cfg.MOTOR_VELOCITY_GAIN,
                maxVelocity=cfg.MOTOR_MAX_VELOCITY,
                physicsClientId=self._client_id,
            )

    def _loop(self) -> None:
        interval = 1.0 / self._target_fps
        while self._running:
            t0 = time.monotonic()

            # Process commands
            with self._cmd_lock:
                cmds = self._pending_cmds
                self._pending_cmds = []
            for ctype, cdata in cmds:
                if ctype == "set_joint":
                    idx = cdata["index"]
                    angle = cdata["angle"]
                    for i, j in enumerate(self._controllable_joints):
                        if j["index"] == idx:
                            self._target_angles[i] = angle
                            break
                elif ctype == "set_joints":
                    self._target_angles = list(cdata["angles"])

            # Apply joint targets, detect collisions, then step simulation
            with self._pybullet_lock:
                if self._control_mode == "motor":
                    self._apply_motor_control(self._target_angles)
                else:
                    self._apply_pose(self._target_angles)

                # Collision detection BEFORE stepSimulation (detect overlaps at reset positions)
                if cfg.COLLISION_DETECTION_ENABLED:
                    collisions = self._detect_collisions()
                    with self._collision_lock:
                        self._collisions = collisions
                        self._collision_detected = len(collisions) > 0
                else:
                    with self._collision_lock:
                        self._collisions = []
                        self._collision_detected = False

                for _ in range(cfg.SIM_STEPS_PER_FRAME):
                    p.stepSimulation(physicsClientId=self._client_id)
                state = self._capture_state()

            with self._lock:
                self._latest_state = state

            # Sleep
            elapsed = time.monotonic() - t0
            if elapsed < interval:
                time.sleep(interval - elapsed)

    def _capture_state(self) -> dict:
        """Capture joint states + link world poses (position + quaternion)."""
        # Joint states
        joints = []
        for i, j in enumerate(self._controllable_joints):
            js = p.getJointState(self._robot_id, j["index"], physicsClientId=self._client_id)
            joints.append({
                "index": j["index"],
                "name": j["name"],
                "position": round(float(js[0]), 4),
                "lower": j["lower"],
                "upper": j["upper"],
            })

        # Link poses (world frame) — use worldLinkFramePosition/Orientation
        # getLinkState returns: 0=pos(COM), 1=orn, 2=localInertialPos, 3=localInertialOrn,
        #                       4=worldLinkFramePos, 5=worldLinkFrameOrn
        # We need indices 4,5 for STL mesh rendering (link visual frame)
        base_pos, base_orn = p.getBasePositionAndOrientation(
            self._robot_id, physicsClientId=self._client_id
        )
        links = [{
            "px": round(base_pos[0], 5), "py": round(base_pos[1], 5), "pz": round(base_pos[2], 5),
            "qx": round(base_orn[0], 5), "qy": round(base_orn[1], 5),
            "qz": round(base_orn[2], 5), "qw": round(base_orn[3], 5),
        }]

        num_joints = p.getNumJoints(self._robot_id, physicsClientId=self._client_id)
        for idx in range(num_joints):
            ls = p.getLinkState(
                self._robot_id, idx,
                computeForwardKinematics=True,
                physicsClientId=self._client_id,
            )
            # Use worldLinkFrame position/orientation (indices 4,5)
            wpos = ls[4]
            wori = ls[5]
            links.append({
                "px": round(wpos[0], 5), "py": round(wpos[1], 5), "pz": round(wpos[2], 5),
                "qx": round(wori[0], 5), "qy": round(wori[1], 5),
                "qz": round(wori[2], 5), "qw": round(wori[3], 5),
            })

        # End-effector pose
        ee_ls = p.getLinkState(self._robot_id, num_joints - 1, physicsClientId=self._client_id)
        ee_rpy = p.getEulerFromQuaternion(ee_ls[1])
        ee_pose = {
            "x": round(ee_ls[0][0], 4), "y": round(ee_ls[0][1], 4), "z": round(ee_ls[0][2], 4),
            "roll": round(ee_rpy[0], 4), "pitch": round(ee_rpy[1], 4), "yaw": round(ee_rpy[2], 4),
        }

        # Mesh file names (matching URDF order)
        mesh_names = [
            "Link_Base_to_Shoulder_Inner.STL",
            "Link_Shoulder_Inner_to_Shoulder_Outer.STL",
            "Link_Shoulder_Outer_to_UpperArm.STL",
            "Link_UpperArm_to_Elbow.STL",
            "Link_Elbow_to_Forearm.STL",
            "Link_Forearm_to_Wrist_Upper.STL",
            "Link_Wrist_Upper_to_Wrist_Lower.STL",
            "Link_Wrist_Lower_to_Gripper.STL",
        ]

        return {"joints": joints, "links": links, "ee_pose": ee_pose, "meshes": mesh_names}

    # Public API (non-blocking)
    def get_latest_state(self) -> dict:
        with self._lock:
            return dict(self._latest_state)

    def set_joint(self, joint_index: int, angle: float) -> None:
        with self._cmd_lock:
            self._pending_cmds.append(("set_joint", {"index": joint_index, "angle": angle}))

    def set_joints(self, angles: list[float]) -> None:
        with self._cmd_lock:
            self._pending_cmds.append(("set_joints", {"angles": angles}))

    def solve_ik(self, target_x: float, target_y: float, target_z: float,
                 target_roll: float = None, target_pitch: float = None, target_yaw: float = None) -> dict:
        """Solve IK for target XYZ position (and optionally orientation).

        Returns dict with solved angles and error info. Does NOT apply the result.
        If target_roll/pitch/yaw are provided, orientation is also constrained.
        """
        target_pos = [target_x, target_y, target_z]

        # Build joint limits for IK solver
        ll, ul, jr, rp = [], [], [], []
        for i, joint in enumerate(self._controllable_joints):
            ll.append(joint["lower"])
            ul.append(joint["upper"])
            jr.append(joint["upper"] - joint["lower"])
            rp.append(self._target_angles[i])

        ee_link = p.getNumJoints(self._robot_id, physicsClientId=self._client_id) - 1

        ik_kwargs = dict(
            bodyUniqueId=self._robot_id,
            endEffectorLinkIndex=ee_link,
            targetPosition=target_pos,
            lowerLimits=ll,
            upperLimits=ul,
            jointRanges=jr,
            restPoses=rp,
            maxNumIterations=500,
            residualThreshold=1e-5,
            physicsClientId=self._client_id,
        )

        # Add orientation target if all RPY values provided
        if target_roll is not None and target_pitch is not None and target_yaw is not None:
            target_orn = p.getQuaternionFromEuler([target_roll, target_pitch, target_yaw])
            ik_kwargs["targetOrientation"] = target_orn

        # All PyBullet operations under lock to prevent background thread interleaving
        saved_targets = list(self._target_angles)
        with self._pybullet_lock:
            # Make sure we're in the current target state before IK
            self._apply_pose(self._target_angles)
            for _ in range(2):
                p.stepSimulation(physicsClientId=self._client_id)

            # Solve IK
            joint_poses = p.calculateInverseKinematics(**ik_kwargs)

            # Clamp angles to limits
            solved_angles = []
            for i, joint in enumerate(self._controllable_joints):
                angle = max(joint["lower"], min(joint["upper"], joint_poses[i]))
                solved_angles.append(round(angle, 4))

            # Verify by applying solved angles temporarily
            self._apply_pose(solved_angles)
            for _ in range(4):
                p.stepSimulation(physicsClientId=self._client_id)
            ee_state = p.getLinkState(
                self._robot_id, ee_link,
                computeForwardKinematics=True,
                physicsClientId=self._client_id,
            )
            actual_pos = ee_state[4]
            actual_rpy = p.getEulerFromQuaternion(ee_state[5])
            error = sum((actual_pos[i] - target_pos[i]) ** 2 for i in range(3)) ** 0.5

            # Restore original targets
            self._apply_pose(saved_targets)
            for _ in range(4):
                p.stepSimulation(physicsClientId=self._client_id)

        result = {
            "angles": solved_angles,
            "actual_pos": {
                "x": round(actual_pos[0], 4),
                "y": round(actual_pos[1], 4),
                "z": round(actual_pos[2], 4),
            },
            "target_pos": {"x": target_x, "y": target_y, "z": target_z},
            "ik_error": round(error, 5),
        }

        if target_roll is not None:
            result["actual_rpy"] = {
                "roll": round(actual_rpy[0], 4),
                "pitch": round(actual_rpy[1], 4),
                "yaw": round(actual_rpy[2], 4),
            }

        return result

    def apply_ik_and_set(self, target_x: float, target_y: float, target_z: float,
                         target_roll: float = None, target_pitch: float = None,
                         target_yaw: float = None, error_threshold: float = 0.01) -> dict:
        """Solve IK and apply result ONLY if error is below threshold.

        Returns result dict with a 'success' field indicating if the move was applied.
        """
        result = self.solve_ik(target_x, target_y, target_z,
                               target_roll, target_pitch, target_yaw)
        if result["ik_error"] <= error_threshold:
            self.set_joints(result["angles"])
            result["success"] = True
        else:
            result["success"] = False
        return result

    def _spawn_collision_bodies(self) -> None:
        """Create collision bodies for workspace objects in the main simulation.
        Enables getContactPoints() to detect robot-object collisions."""
        with self._wksp_lock:
            objects = list(self._wksp_objects)
        self._wksp_body_ids = []
        for obj in objects:
            pos = obj["pos"]
            r, g, b = obj["color"]
            rgba = [r, g, b, 1.0]
            if obj["type"] == "sphere":
                radius = obj.get("radius", 0.025)
                cs = p.createCollisionShape(shapeType=p.GEOM_SPHERE, radius=radius, physicsClientId=self._client_id)
                vs = p.createVisualShape(shapeType=p.GEOM_SPHERE, radius=radius, rgbaColor=rgba, physicsClientId=self._client_id)
                bid = p.createMultiBody(baseMass=0, baseCollisionShapeIndex=cs, baseVisualShapeIndex=vs, basePosition=pos, physicsClientId=self._client_id)
                self._wksp_body_ids.append(bid)
            elif obj["type"] == "box":
                he = [s / 2.0 for s in obj.get("size", [0.04, 0.04, 0.04])]
                cs = p.createCollisionShape(shapeType=p.GEOM_BOX, halfExtents=he, physicsClientId=self._client_id)
                vs = p.createVisualShape(shapeType=p.GEOM_BOX, halfExtents=he, rgbaColor=rgba, physicsClientId=self._client_id)
                bid = p.createMultiBody(baseMass=0, baseCollisionShapeIndex=cs, baseVisualShapeIndex=vs, basePosition=pos, physicsClientId=self._client_id)
                self._wksp_body_ids.append(bid)

    # Links to skip in self-collision detection.
    # URDF_USE_SELF_COLLISION_EXCLUDE_PARENT already handles direct parent-child
    # links. We only additionally filter base link vs body contacts (link index -1).
    _ADJACENT_LINKS = {
        (-1, 0), (0, -1), (-1, 1), (1, -1), (-1, 2), (2, -1),
    }

    # Link index to human-readable name mapping
    _LINK_NAMES = {
        -1: "Base",
        0: "Shoulder_Inner",
        1: "Shoulder_Outer",
        2: "UpperArm",
        3: "Elbow",
        4: "Forearm",
        5: "Wrist_Upper",
        6: "Wrist_Lower",
        7: "Gripper",
    }

    def _get_link_name(self, link_index: int) -> str:
        """Return human-readable link name from index."""
        return self._LINK_NAMES.get(link_index, f"Link_{link_index}")

    def _detect_collisions(self) -> list[dict]:
        """Detect collisions: robot vs workspace objects + robot self-collisions."""
        if not cfg.COLLISION_DETECTION_ENABLED:
            return []

        # Force PyBullet to recompute broadphase/narrowphase after resetJointState
        p.performCollisionDetection(physicsClientId=self._client_id)

        collisions = []

        # 1. Robot vs workspace objects
        for body_id in self._wksp_body_ids:
            contacts = p.getContactPoints(bodyA=self._robot_id, bodyB=body_id, physicsClientId=self._client_id)
            for c in contacts:
                normal_force = abs(c[9])
                if normal_force < cfg.COLLISION_FORCE_THRESHOLD:
                    continue
                # Determine which object was hit
                obj_idx = -1
                for i, bid in enumerate(self._wksp_body_ids):
                    if bid == body_id:
                        obj_idx = i
                        break
                obj_name = "unknown"
                if obj_idx >= 0 and obj_idx < len(self._wksp_objects):
                    obj_info = self._wksp_objects[obj_idx]
                    obj_name = f"{obj_info['type']}_{obj_idx}"

                collisions.append({
                    "type": "object",
                    "robot_link": c[3],
                    "robot_link_name": self._get_link_name(c[3]),
                    "object_body": body_id,
                    "object_name": obj_name,
                    "force": round(normal_force, 3),
                    "position": [round(v, 4) for v in c[5]],
                })

        # 2. Robot vs ground plane
        ground_contacts = p.getContactPoints(
            bodyA=self._robot_id, bodyB=self._ground_id,
            physicsClientId=self._client_id,
        )
        for c in ground_contacts:
            normal_force = abs(c[9])
            if normal_force < cfg.COLLISION_FORCE_THRESHOLD:
                continue
            collisions.append({
                "type": "ground",
                "robot_link": c[3],
                "robot_link_name": self._get_link_name(c[3]),
                "force": round(normal_force, 3),
                "position": [round(v, 4) for v in c[5]],
            })

        # 3. Robot self-collision (link vs link)
        self_contacts = p.getContactPoints(
            bodyA=self._robot_id, bodyB=self._robot_id,
            physicsClientId=self._client_id,
        )
        for c in self_contacts:
            link_a, link_b = c[3], c[4]
            # Skip same link and directly adjacent links
            if link_a == link_b:
                continue
            if (link_a, link_b) in self._ADJACENT_LINKS:
                continue
            if (link_a, link_b) in cfg.COLLISION_IGNORE_LINK_PAIRS:
                continue
            normal_force = abs(c[9])
            if normal_force < cfg.COLLISION_FORCE_THRESHOLD:
                continue
            collisions.append({
                "type": "self",
                "link_a": link_a, "link_b": link_b,
                "link_a_name": self._get_link_name(link_a),
                "link_b_name": self._get_link_name(link_b),
                "force": round(normal_force, 3),
                "position": [round(v, 4) for v in c[5]],
            })

        return collisions

    def get_collision_state(self) -> dict:
        """Return the latest collision state."""
        with self._collision_lock:
            return {"detected": self._collision_detected, "count": len(self._collisions), "collisions": list(self._collisions)}

    def get_debug_self_contacts(self) -> dict:
        """Debug: return ALL raw self-contact points before filtering."""
        with self._pybullet_lock:
            p.performCollisionDetection(physicsClientId=self._client_id)
            all_contacts = p.getContactPoints(bodyA=self._robot_id, bodyB=self._robot_id, physicsClientId=self._client_id)
        result = []
        for c in all_contacts:
            link_a, link_b = c[3], c[4]
            is_adjacent = (link_a, link_b) in self._ADJACENT_LINKS or link_a == link_b
            result.append({
                "linkA": link_a, "linkB": link_b, "force": round(abs(c[9]), 4),
                "filtered_as_adjacent": is_adjacent,
            })
        return {"total": len(result), "contacts": result}

    def update_collision_body_positions(self, objects: list[dict]) -> None:
        """Update collision body positions in PyBullet without recreating them.
        Called from frontend when workspace objects are dragged."""
        with self._wksp_lock:
            for i, obj in enumerate(objects):
                if i < len(self._wksp_objects) and "pos" in obj:
                    new_pos = list(obj["pos"])
                    self._wksp_objects[i]["pos"] = new_pos
                    if i < len(self._wksp_body_ids):
                        try:
                            p.resetBasePositionAndOrientation(
                                self._wksp_body_ids[i],
                                new_pos,
                                [0, 0, 0, 1],
                                physicsClientId=self._client_id,
                            )
                        except Exception:
                            pass

    def set_workspace_objects(self, objects: list[dict]) -> None:
        """Update workspace object positions from frontend.

        Args:
            objects: list of dicts with keys:
                - "pos": [x, y, z]
                - optionally "type", "radius", "size", "color" (used for defaults only)
        """
        self.update_collision_body_positions(objects)

    def _spawn_workspace_objects(self, cam_client: int) -> None:
        """Create workspace objects (spheres, boxes) in a PyBullet scene."""
        with self._wksp_lock:
            objects = list(self._wksp_objects)

        for obj in objects:
            pos = obj["pos"]
            r, g, b = obj["color"]
            rgba = [r, g, b, 1.0]

            if obj["type"] == "sphere":
                radius = obj.get("radius", 0.025)
                vs = p.createVisualShape(
                    shapeType=p.GEOM_SPHERE,
                    radius=radius,
                    rgbaColor=rgba,
                    physicsClientId=cam_client,
                )
                p.createMultiBody(
                    baseMass=0,
                    baseVisualShapeIndex=vs,
                    basePosition=pos,
                    physicsClientId=cam_client,
                )
            elif obj["type"] == "box":
                half_extents = [s / 2.0 for s in obj.get("size", [0.04, 0.04, 0.04])]
                vs = p.createVisualShape(
                    shapeType=p.GEOM_BOX,
                    halfExtents=half_extents,
                    rgbaColor=rgba,
                    physicsClientId=cam_client,
                )
                p.createMultiBody(
                    baseMass=0,
                    baseVisualShapeIndex=vs,
                    basePosition=pos,
                    physicsClientId=cam_client,
                )

    def _save_frame_to_disk(self, rgb_bytes: bytes, depth_bytes: bytes) -> str:
        """Save RGB and depth JPEG bytes to a timestamped folder under captures/.
        Returns the folder path, or empty string on failure."""
        if not rgb_bytes:
            return ""
        try:
            import datetime
            timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
            captures_dir = ROOT_DIR / "captures"
            frame_dir = captures_dir / timestamp
            frame_dir.mkdir(parents=True, exist_ok=True)

            rgb_path = frame_dir / "rgb.jpg"
            depth_path = frame_dir / "depth.jpg"

            rgb_path.write_bytes(rgb_bytes)
            depth_path.write_bytes(depth_bytes)

            rel_path = str(frame_dir.relative_to(ROOT_DIR))
            print(f"[camera] Saved frames to {frame_dir}")
            return rel_path
        except Exception as e:
            print(f"[camera] Failed to save frames: {e}")
            return ""

    def capture_single_frame(self) -> None:
        """Spawn a background thread that creates its own PyBullet connection,
        captures ONE frame, stores the result, then disconnects and exits.
        Non-blocking — call this from the API/button handler.
        """
        if self._cam_capturing:
            return  # already in progress

        def _worker():
            self._cam_capturing = True
            self._last_capture_error = ""
            cam_client = -1
            try:
                # 1. Connect a fresh PyBullet instance (DIRECT = headless)
                cam_client = p.connect(p.DIRECT)
                print(f"[camera] Connected PyBullet client={cam_client}")

                p.setAdditionalSearchPath(pybullet_data.getDataPath(), physicsClientId=cam_client)

                # 2. Load the same robot + ground plane
                cam_robot = p.loadURDF(
                    str(self._urdf_path),
                    basePosition=[0, 0, 0.05],
                    useFixedBase=True,
                    physicsClientId=cam_client,
                )
                p.loadURDF("plane.urdf", physicsClientId=cam_client)
                print(f"[camera] Loaded robot id={cam_robot}, urdf={self._urdf_path}")

                # 3. Spawn workspace objects (spheres, boxes)
                self._spawn_workspace_objects(cam_client)
                print(f"[camera] Spawned {len(self._wksp_objects)} workspace objects")

                # 4. Apply current joint targets
                with self._cmd_lock:
                    targets = list(self._target_angles)
                for i, joint in enumerate(self._controllable_joints):
                    a = max(joint["lower"], min(joint["upper"], targets[i]))
                    p.resetJointState(cam_robot, joint["index"], a, 0.0, physicsClientId=cam_client)

                print(f"[camera] Applied {len(targets)} joint targets")

                # 5. Render one frame
                rgb_bytes, depth_bytes = self._render_camera(cam_client, cam_robot)
                print(f"[camera] Render done: rgb={len(rgb_bytes)} bytes, depth={len(depth_bytes)} bytes")

                if not rgb_bytes:
                    self._last_capture_error = "渲染返回空数据（OpenCV未安装或渲染失败）"
                    print(f"[camera] WARNING: empty frame data!")

                # 6. Store result
                with self._cam_lock:
                    self._latest_rgb_bytes = rgb_bytes
                    self._latest_depth_bytes = depth_bytes

                # 7. Save to disk
                save_path = self._save_frame_to_disk(rgb_bytes, depth_bytes)
                self._last_save_path = save_path

                print("[camera] Single frame captured successfully")
            except Exception as e:
                import traceback
                tb = traceback.format_exc()
                self._last_capture_error = str(e)
                print(f"[camera] Single-frame capture error: {e}\n{tb}")
            finally:
                # 8. Disconnect — thread exits
                if cam_client >= 0:
                    try:
                        p.disconnect(physicsClientId=cam_client)
                    except Exception:
                        pass
                self._cam_capturing = False

        threading.Thread(target=_worker, daemon=True).start()

    def _render_camera(self, cam_client: int, cam_robot: int) -> tuple[bytes, bytes]:
        """Render camera from a separate PyBullet instance. Returns (rgb_jpeg_bytes, depth_jpeg_bytes)."""
        try:
            ee_link = p.getNumJoints(cam_robot, physicsClientId=cam_client) - 1
            ee_state = p.getLinkState(
                cam_robot, ee_link,
                computeForwardKinematics=True,
                physicsClientId=cam_client,
            )
            ee_pos = ee_state[4]
            ee_orn = ee_state[5]

            rot = p.getMatrixFromQuaternion(ee_orn)
            local_x = [rot[0], rot[3], rot[6]]

            cam_eye = [
                ee_pos[0] + local_x[0] * 0.15,
                ee_pos[1] + local_x[1] * 0.15,
                ee_pos[2] + local_x[2] * 0.15,
            ]
            if cam_eye[2] < 0.02:
                cam_eye[2] = 0.02

            cam_target = [
                cam_eye[0] + local_x[0] * 1.0,
                cam_eye[1] + local_x[1] * 1.0,
                cam_eye[2] + local_x[2] * 1.0,
            ]

            up = [0, 0, 1]
            if abs(local_x[0] * up[0] + local_x[1] * up[1] + local_x[2] * up[2]) > 0.95:
                up = [0, 1, 0]

            view_matrix = p.computeViewMatrix(cam_eye, cam_target, up)
            proj_matrix = p.computeProjectionMatrixFOV(
                fov=60, aspect=self._cam_width / self._cam_height,
                nearVal=0.01, farVal=5.0
            )

            w, h, rgba, depth_raw, _ = p.getCameraImage(
                width=self._cam_width, height=self._cam_height,
                viewMatrix=view_matrix, projectionMatrix=proj_matrix,
                renderer=p.ER_TINY_RENDERER,
                physicsClientId=cam_client,
            )

            rgba_arr = np.array(rgba, dtype=np.uint8).reshape(h, w, 4)
            rgb_arr = rgba_arr[:, :, :3]

            depth_arr = np.array(depth_raw, dtype=np.float32).reshape(h, w)
            far, near = 5.0, 0.01
            real_depth = (far * near) / (far - (far - near) * depth_arr)
            finite_mask = np.isfinite(real_depth)
            if finite_mask.any():
                dmin, dmax = real_depth[finite_mask].min(), real_depth[finite_mask].max()
                if dmax - dmin > 1e-6:
                    norm = (real_depth - dmin) / (dmax - dmin)
                else:
                    norm = np.ones_like(real_depth)
                depth_gray = ((1.0 - norm) * 255.0).astype(np.uint8)
                depth_gray[~finite_mask] = 0
            else:
                depth_gray = np.zeros((h, w), dtype=np.uint8)

            if cv2 is not None:
                # RGB: use OpenCV JPEG encoding (faster than PIL)
                _, rgb_encoded = cv2.imencode('.jpg', cv2.cvtColor(rgb_arr, cv2.COLOR_RGB2BGR),
                                              [cv2.IMWRITE_JPEG_QUALITY, 40])
                rgb_bytes = rgb_encoded.tobytes()

                # Depth: apply JET colormap for visualization (matches Three.js heatmap)
                depth_colormap = cv2.applyColorMap(depth_gray, cv2.COLORMAP_JET)
                _, depth_encoded = cv2.imencode('.jpg', depth_colormap,
                                                [cv2.IMWRITE_JPEG_QUALITY, 40])
                depth_bytes = depth_encoded.tobytes()
            else:
                rgb_bytes = b''
                depth_bytes = b''

            return rgb_bytes, depth_bytes
        except Exception as e:
            print(f'[camera] Error: {e}')
            return b'', b''

    def get_latest_camera_bytes(self) -> tuple[bytes, bytes]:
        """Return raw JPEG bytes for RGB and depth."""
        with self._cam_lock:
            return self._latest_rgb_bytes, self._latest_depth_bytes

    def set_camera_enabled(self, enabled: bool) -> None:
        """Legacy compat — no-op since camera is now on-demand only."""
        pass

    def shutdown(self) -> None:
        self._running = False
        self._thread.join(timeout=2)
        p.disconnect(physicsClientId=self._client_id)
