import base64
import numpy as np
from PIL import Image
import io

class RobotSimMock:
    """Mock PyBullet sim for frontend development while real pybullet compiles"""
    
    def __init__(self, gui=False):
        self.gui = gui
        self.num_joints = 6
        self.end_effector_link = 5
        self.joint_angles = [0.0] * self.num_joints
        self.frame_count = 0
    
    def step(self):
        self.frame_count += 1
    
    def get_joint_states(self):
        # Return current joint angles with some oscillation for demo
        import math
        offset = math.sin(self.frame_count * 0.01) * 0.3
        return [angle + offset for angle in self.joint_angles]
    
    def set_joint_angles_list(self, angles_dict):
        for i, angle in angles_dict.items():
            if int(i) < self.num_joints:
                self.joint_angles[int(i)] = float(angle)
    
    def get_end_effector_pose(self):
        # Mock end effector position based on joint angles
        x = sum([np.sin(a) * 0.2 for a in self.joint_angles[:3]])
        y = sum([np.cos(a) * 0.2 for a in self.joint_angles[:3]])
        z = 0.5 + sum([np.sin(a) * 0.1 for a in self.joint_angles[3:]])
        return {
            "position": (x, y, z),
            "orientation": (0, 0, 0, 1)
        }
    
    def get_camera_image(self, width=640, height=480):
        # Return mock images (solid colors)
        rgb_array = np.zeros((height, width, 4), dtype=np.uint8)
        # Simple gradient pattern
        for i in range(height):
            rgb_array[i, :, 0] = int(255 * i / height)  # Red gradient
            rgb_array[i, :, 1] = 100  # Green
            rgb_array[i, :, 2] = int(255 * (1 - i / height))  # Blue gradient
            rgb_array[i, :, 3] = 255  # Alpha
        
        depth_array = np.ones((height, width), dtype=np.float32) * 5.0
        return rgb_array, depth_array
    
    def get_camera_image_base64(self, width=640, height=480):
        rgb, depth = self.get_camera_image(width=width, height=height)
        
        # Create PIL image from RGBA array
        img = Image.fromarray(rgb[:, :, :3], 'RGB')
        
        # Encode to JPEG and base64
        buf = io.BytesIO()
        img.save(buf, format='JPEG', quality=80)
        jpg_b64 = base64.b64encode(buf.getvalue()).decode('ascii')
        return jpg_b64
