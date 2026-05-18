import os
import time
import pybullet as p
import pybullet_data

class RobotSim:
    def __init__(self, gui=True):
        flags = p.GUI if gui else p.DIRECT
        self.client = p.connect(flags)
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        # Add the project mesh folder to PyBullet search paths for URDF mesh references.
        mesh_search_dir = os.path.join(os.path.dirname(__file__), "autolife_desktop", "meshes", "robot_v0_1")
        if os.path.isdir(mesh_search_dir):
            p.setAdditionalSearchPath(mesh_search_dir)
        p.setGravity(0, 0, -9.81)
        p.setTimeStep(1.0 / 240.0)

        # Locate URDF from project structure, including autolife_desktop/urdfs
        urdf_path = os.path.join(os.path.dirname(__file__), "autolife_desktop", "urdfs", "robot_v0_1.urdf")
        if not os.path.exists(urdf_path):
            urdf_path = os.path.join(os.path.dirname(__file__), "urdfs", "robot_v0_1.urdf")
        if not os.path.exists(urdf_path):
            urdf_path = os.path.join(os.getcwd(), "autolife_desktop", "urdfs", "robot_v0_1.urdf")
        if not os.path.exists(urdf_path):
            urdf_path = os.path.join(os.getcwd(), "urdfs", "robot_v0_1.urdf")

        if not os.path.exists(urdf_path):
            raise FileNotFoundError(f"URDF not found at {urdf_path}")

        self.robot_id = p.loadURDF(urdf_path, [0, 0, 0.5])
        self.num_joints = p.getNumJoints(self.robot_id)
        self.end_effector_link = self.num_joints - 1

    def step(self):
        p.stepSimulation()

    def get_joint_states(self):
        states = []
        for i in range(self.num_joints):
            js = p.getJointState(self.robot_id, i)
            states.append(js[0])
        return states

    def set_joint_angles_list(self, angles_dict):
        for i, angle in angles_dict.items():
            p.setJointMotorControl2(
                bodyIndex=self.robot_id,
                jointIndex=int(i),
                controlMode=p.POSITION_CONTROL,
                targetPosition=float(angle),
                force=100,
            )

    def get_end_effector_pose(self):
        state = p.getLinkState(self.robot_id, self.end_effector_link)
        return {"position": state[0], "orientation": state[1]}

    def get_camera_image(self, width=640, height=480):
        link_state = p.getLinkState(self.robot_id, self.end_effector_link, computeForwardKinematics=True)
        pos = link_state[0]

        # Simple camera mounted at end effector looking forward
        view_matrix = p.computeViewMatrixFromYawPitchRoll(cameraTargetPosition=pos, distance=0.2, yaw=0, pitch=-30, roll=0, upAxisIndex=2)
        proj_matrix = p.computeProjectionMatrixFOV(fov=60, aspect=float(width) / float(height), nearVal=0.01, farVal=10)

        w, h, rgbImg, depthImg, segImg = p.getCameraImage(
            width=width,
            height=height,
            viewMatrix=view_matrix,
            projectionMatrix=proj_matrix,
            renderer=p.ER_BULLET_HARDWARE_OPENGL,
        )

        return rgbImg, depthImg

    def get_camera_image_base64(self, width=640, height=480):
        import numpy as np
        import cv2
        import base64

        rgb, depth = self.get_camera_image(width=width, height=height)

        arr = np.array(rgb, dtype=np.uint8)
        # PyBullet may return a flat list or a shaped array; try to reshape
        if arr.ndim == 1:
            arr = arr.reshape((height, width, 4))

        if arr.shape[2] == 4:
            img_rgb = arr[:, :, :3]
        else:
            img_rgb = arr

        img_bgr = cv2.cvtColor(img_rgb, cv2.COLOR_RGB2BGR)
        _, buf = cv2.imencode('.jpg', img_bgr, [int(cv2.IMWRITE_JPEG_QUALITY), 80])
        jpg_b64 = base64.b64encode(buf.tobytes()).decode('ascii')
        return jpg_b64
