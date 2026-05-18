from robot_sim import RobotSim

class Camera:
    def __init__(self, sim: RobotSim):
        self.sim = sim

    def get_rgb_base64(self, width=640, height=480):
        return self.sim.get_camera_image_base64(width=width, height=height)
