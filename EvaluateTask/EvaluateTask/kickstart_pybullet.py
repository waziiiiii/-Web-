from __future__ import annotations

import argparse
import math
import time
import xml.etree.ElementTree as ET
from pathlib import Path

try:
    import pybullet as p
    import pybullet_data
except ImportError as exc:
    raise SystemExit(
        "pybullet is required. Install it with: pip install pybullet"
    ) from exc

try:
    from PIL import Image
except ImportError:
    Image = None


ROOT_DIR = Path(__file__).resolve().parent
DEFAULT_URDF = ROOT_DIR / "autolife_desktop" / "urdfs" / "robot_v0_1.urdf"
DEFAULT_OUTPUT_DIR = ROOT_DIR / "artifacts"

JOINT_TYPE_NAMES = {
    p.JOINT_REVOLUTE: "revolute",
    p.JOINT_PRISMATIC: "prismatic",
    p.JOINT_SPHERICAL: "spherical",
    p.JOINT_PLANAR: "planar",
    p.JOINT_FIXED: "fixed",
    p.JOINT_POINT2POINT: "point2point",
    p.JOINT_GEAR: "gear",
}


def iter_scalar_values(buffer: object):
    if hasattr(buffer, "flat"):
        for value in buffer.flat:
            yield float(value)
        return

    for value in buffer:
        if isinstance(value, (list, tuple)):
            for nested_value in value:
                yield float(nested_value)
            continue
        yield float(value)


def reshape_buffer(buffer: object, width: int, height: int, channels: int = 1) -> list:
    flat_values = list(iter_scalar_values(buffer))
    expected_size = width * height * channels
    if len(flat_values) != expected_size:
        raise RuntimeError(
            f"Unexpected camera buffer size: expected {expected_size}, got {len(flat_values)}"
        )

    if channels == 1:
        return [
            flat_values[row_index * width : (row_index + 1) * width]
            for row_index in range(height)
        ]

    rows = []
    for row_index in range(height):
        row = []
        row_offset = row_index * width * channels
        for col_index in range(width):
            pixel_offset = row_offset + col_index * channels
            row.append(
                tuple(
                    int(flat_values[pixel_offset + channel_index])
                    for channel_index in range(channels)
                )
            )
        rows.append(row)
    return rows


def save_ppm_image(rgb_pixels: list[list[tuple[int, int, int]]], output_path: Path) -> None:
    height = len(rgb_pixels)
    width = len(rgb_pixels[0]) if height else 0
    with output_path.open("w", encoding="ascii", newline="\n") as handle:
        handle.write(f"P3\n{width} {height}\n255\n")
        for row in rgb_pixels:
            pixel_values = []
            for red, green, blue in row:
                pixel_values.append(f"{red} {green} {blue}")
            handle.write(" ".join(pixel_values))
            handle.write("\n")


def save_pgm_image(gray_pixels: list[list[int]], output_path: Path) -> None:
    height = len(gray_pixels)
    width = len(gray_pixels[0]) if height else 0
    with output_path.open("w", encoding="ascii", newline="\n") as handle:
        handle.write(f"P2\n{width} {height}\n255\n")
        for row in gray_pixels:
            handle.write(" ".join(str(value) for value in row))
            handle.write("\n")


def write_rgb_image(rgb_pixels: list[list[tuple[int, int, int]]], output_path: Path) -> Path:
    if Image is not None:
        image = Image.new("RGB", (len(rgb_pixels[0]), len(rgb_pixels)))
        image.putdata([pixel for row in rgb_pixels for pixel in row])
        png_path = output_path.with_suffix(".png")
        image.save(png_path)
        return png_path

    ppm_path = output_path.with_suffix(".ppm")
    save_ppm_image(rgb_pixels, ppm_path)
    return ppm_path


def write_depth_image(gray_pixels: list[list[int]], output_path: Path) -> Path:
    if Image is not None:
        image = Image.new("L", (len(gray_pixels[0]), len(gray_pixels)))
        image.putdata([value for row in gray_pixels for value in row])
        png_path = output_path.with_suffix(".png")
        image.save(png_path)
        return png_path

    pgm_path = output_path.with_suffix(".pgm")
    save_pgm_image(gray_pixels, pgm_path)
    return pgm_path


def normalize_depth_to_grayscale(depth_rows: list[list[float]]) -> list[list[int]]:
    finite_depths = [depth for row in depth_rows for depth in row if math.isfinite(depth)]
    if not finite_depths:
        return [[0 for _ in row] for row in depth_rows]

    min_depth = min(finite_depths)
    max_depth = max(finite_depths)
    if math.isclose(min_depth, max_depth):
        return [[255 for _ in row] for row in depth_rows]

    grayscale_rows: list[list[int]] = []
    for row in depth_rows:
        grayscale_row = []
        for depth in row:
            if not math.isfinite(depth):
                grayscale_row.append(0)
                continue
            normalized = (depth - min_depth) / (max_depth - min_depth)
            grayscale_row.append(int(round((1.0 - normalized) * 255.0)))
        grayscale_rows.append(grayscale_row)
    return grayscale_rows


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Kickstart PyBullet demo for URDF loading and basic simulation checks."
    )
    parser.add_argument(
        "--urdf",
        type=Path,
        default=DEFAULT_URDF,
        help="Path to the URDF file to load.",
    )
    parser.add_argument(
        "--mode",
        choices=("gui", "direct"),
        default="gui",
        help="Use GUI for visual inspection or DIRECT for headless verification.",
    )
    parser.add_argument(
        "--steps",
        type=int,
        default=480,
        help="Simulation steps to run after the robot has loaded.",
    )
    parser.add_argument(
        "--time-step",
        type=float,
        default=1.0 / 240.0,
        help="Physics simulation time step in seconds.",
    )
    parser.add_argument(
        "--width",
        type=int,
        default=640,
        help="Camera image width.",
    )
    parser.add_argument(
        "--height",
        type=int,
        default=480,
        help="Camera image height.",
    )
    parser.add_argument(
        "--base-height",
        type=float,
        default=0.05,
        help="Initial base height for the robot above the plane.",
    )
    parser.add_argument(
        "--output-dir",
        type=Path,
        default=DEFAULT_OUTPUT_DIR,
        help="Directory used to store captured RGB and depth images.",
    )
    return parser.parse_args()


def validate_urdf_assets(urdf_path: Path) -> list[Path]:
    urdf_path = urdf_path.resolve()
    if not urdf_path.exists():
        raise FileNotFoundError(f"URDF file not found: {urdf_path}")

    try:
        root = ET.parse(urdf_path).getroot()
    except ET.ParseError as exc:
        raise RuntimeError(f"URDF XML parse failed: {exc}") from exc

    if root.tag != "robot":
        raise RuntimeError(f"Unexpected URDF root tag: {root.tag}")

    mesh_paths: list[Path] = []
    for mesh in root.findall(".//mesh"):
        filename = mesh.attrib.get("filename")
        if not filename:
            continue
        mesh_path = (urdf_path.parent / filename).resolve()
        mesh_paths.append(mesh_path)

    missing_meshes = [path for path in mesh_paths if not path.exists()]
    if missing_meshes:
        missing_preview = "\n".join(f"  - {path}" for path in missing_meshes[:10])
        raise FileNotFoundError(
            "Missing mesh assets referenced by the URDF:\n" + missing_preview
        )

    return mesh_paths


def connect_pybullet(mode: str) -> int:
    client_id = p.connect(p.GUI if mode == "gui" else p.DIRECT)
    if client_id < 0:
        raise RuntimeError(f"Failed to connect to PyBullet in {mode} mode")
    return client_id


def configure_world(time_step: float, mode: str) -> None:
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.setGravity(0, 0, -9.81)
    p.setTimeStep(time_step)
    p.setRealTimeSimulation(0)
    p.loadURDF("plane.urdf")

    if mode == "gui":
        p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0)
        p.resetDebugVisualizerCamera(
            cameraDistance=1.4,
            cameraYaw=45,
            cameraPitch=-25,
            cameraTargetPosition=[0.0, 0.0, 0.35],
        )


def collect_joint_info(robot_id: int) -> list[dict[str, float | int | str]]:
    joint_info_list: list[dict[str, float | int | str]] = []
    joint_count = p.getNumJoints(robot_id)
    for joint_index in range(joint_count):
        joint_info = p.getJointInfo(robot_id, joint_index)
        joint_info_list.append(
            {
                "index": joint_index,
                "name": joint_info[1].decode("utf-8"),
                "type": JOINT_TYPE_NAMES.get(joint_info[2], str(joint_info[2])),
                "link_name": joint_info[12].decode("utf-8"),
                "lower": float(joint_info[8]),
                "upper": float(joint_info[9]),
                "max_force": float(joint_info[10]),
                "max_velocity": float(joint_info[11]),
            }
        )
    return joint_info_list


def print_joint_summary(joint_info_list: list[dict[str, float | int | str]]) -> None:
    print("[info] Joint summary:")
    for joint in joint_info_list:
        print(
            "  - "
            f"#{joint['index']}: {joint['name']} "
            f"({joint['type']}, child={joint['link_name']}, "
            f"limit=[{joint['lower']:.3f}, {joint['upper']:.3f}], "
            f"max_force={joint['max_force']:.3f})"
        )


def compute_target_position(joint: dict[str, float | int | str], phase: float) -> float:
    lower = float(joint["lower"])
    upper = float(joint["upper"])
    offset = 0.35 * math.sin(phase + int(joint["index"]) * 0.45)

    if math.isfinite(lower) and math.isfinite(upper) and lower < upper:
        center = (lower + upper) * 0.5
        amplitude = min((upper - lower) * 0.25, 0.7)
        return center + amplitude * math.sin(phase + int(joint["index"]) * 0.45)

    return offset


def run_joint_demo(
    robot_id: int,
    joint_info_list: list[dict[str, float | int | str]],
    steps: int,
    time_step: float,
    mode: str,
) -> None:
    controllable_joints = [
        joint
        for joint in joint_info_list
        if joint["type"] in {"revolute", "prismatic"}
    ]

    if not controllable_joints:
        print("[warn] No controllable joints were found in the URDF.")
        return

    for step_index in range(max(steps, 0)):
        phase = 2.0 * math.pi * step_index / max(steps, 1)
        for joint in controllable_joints:
            p.setJointMotorControl2(
                bodyIndex=robot_id,
                jointIndex=int(joint["index"]),
                controlMode=p.POSITION_CONTROL,
                targetPosition=compute_target_position(joint, phase),
                force=max(float(joint["max_force"]), 2.0),
            )
        p.stepSimulation()
        if mode == "gui":
            time.sleep(time_step)


def capture_camera(
    robot_id: int,
    width: int,
    height: int,
    mode: str,
    output_dir: Path,
) -> None:
    base_position, _ = p.getBasePositionAndOrientation(robot_id)
    near_val = 0.02
    far_val = 4.0
    renderer = (
        p.ER_BULLET_HARDWARE_OPENGL if mode == "gui" else p.ER_TINY_RENDERER
    )
    renderer_name = "ER_BULLET_HARDWARE_OPENGL" if mode == "gui" else "ER_TINY_RENDERER"

    view_matrix = p.computeViewMatrixFromYawPitchRoll(
        cameraTargetPosition=[base_position[0], base_position[1], base_position[2] + 0.30],
        distance=1.20,
        yaw=45,
        pitch=-25,
        roll=0,
        upAxisIndex=2,
    )
    projection_matrix = p.computeProjectionMatrixFOV(
        fov=60.0,
        aspect=float(width) / float(height),
        nearVal=near_val,
        farVal=far_val,
    )

    image_width, image_height, rgba_buffer, depth_buffer, segmentation = p.getCameraImage(
        width=width,
        height=height,
        viewMatrix=view_matrix,
        projectionMatrix=projection_matrix,
        renderer=renderer,
    )

    rgb_rows = reshape_buffer(rgba_buffer, image_width, image_height, channels=4)
    rgb_pixels = [
        [(red, green, blue) for red, green, blue, _alpha in row]
        for row in rgb_rows
    ]

    depth_rows = []
    depth_values = []
    for raw_row in reshape_buffer(depth_buffer, image_width, image_height):
        depth_row = []
        for raw_depth in raw_row:
            depth_value = (far_val * near_val) / (
                far_val - (far_val - near_val) * raw_depth
            )
            depth_row.append(depth_value)
            if math.isfinite(depth_value):
                depth_values.append(depth_value)
        depth_rows.append(depth_row)

    output_dir.mkdir(parents=True, exist_ok=True)
    rgb_path = write_rgb_image(rgb_pixels, output_dir / "pybullet_rgb")
    depth_path = write_depth_image(
        normalize_depth_to_grayscale(depth_rows),
        output_dir / "pybullet_depth",
    )

    visible_segments = sorted(
        {
            int(segment_id)
            for segment_id in iter_scalar_values(segmentation)
            if int(segment_id) >= 0
        }
    )

    print(
        f"[ok] Camera image captured: {image_width}x{image_height}, "
        f"renderer={renderer_name}, visible_segments={visible_segments[:10]}"
    )
    print(f"[ok] RGB image saved: {rgb_path}")
    print(f"[ok] Depth image saved: {depth_path}")
    if depth_values:
        print(
            f"[ok] Depth range: min={min(depth_values):.4f} m, "
            f"max={max(depth_values):.4f} m"
        )


def print_end_effector_pose(robot_id: int) -> None:
    end_link_index = p.getNumJoints(robot_id) - 1
    if end_link_index < 0:
        return

    link_state = p.getLinkState(robot_id, end_link_index)
    position = link_state[0]
    roll, pitch, yaw = p.getEulerFromQuaternion(link_state[1])
    print(
        "[ok] End-effector pose: "
        f"xyz=({position[0]:.4f}, {position[1]:.4f}, {position[2]:.4f}), "
        f"rpy=({roll:.4f}, {pitch:.4f}, {yaw:.4f})"
    )


def main() -> None:
    args = parse_args()
    urdf_path = args.urdf.resolve()
    mesh_paths = validate_urdf_assets(urdf_path)
    client_id = connect_pybullet(args.mode)

    try:
        configure_world(args.time_step, args.mode)
        robot_id = p.loadURDF(
            str(urdf_path),
            basePosition=[0.0, 0.0, args.base_height],
            useFixedBase=True,
            flags=p.URDF_USE_INERTIA_FROM_FILE,
        )

        print(f"[ok] PyBullet connected in {args.mode.upper()} mode (client_id={client_id})")
        print(f"[ok] URDF XML parsed successfully: {urdf_path}")
        print(f"[ok] Mesh assets resolved: {len(mesh_paths)} entries")
        print(f"[ok] Robot loaded: body_id={robot_id}, joint_count={p.getNumJoints(robot_id)}")

        joint_info_list = collect_joint_info(robot_id)
        print_joint_summary(joint_info_list)
        run_joint_demo(robot_id, joint_info_list, args.steps, args.time_step, args.mode)
        print_end_effector_pose(robot_id)
        capture_camera(robot_id, args.width, args.height, args.mode, args.output_dir)
        print("[done] Demo completed. URDF loading and basic PyBullet features are working.")
    finally:
        p.disconnect()


if __name__ == "__main__":
    main()