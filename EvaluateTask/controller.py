from robot_sim_mock import RobotSimMock as RobotSim
# from robot_sim import RobotSim  # Switch to real pybullet once installed

class RobotController:
    def __init__(self, sim: RobotSim):
        self.sim = sim

    def set_joint_angles(self, joint_angles):
        # joint_angles: list or dict
        if isinstance(joint_angles, dict):
            self.sim.set_joint_angles_list(joint_angles)
        else:
            # assume list indexed from 0
            d = {i: a for i, a in enumerate(joint_angles)}
            self.sim.set_joint_angles_list(d)

    def move_to_position(self, target_pos):
        # use PyBullet IK directly
        import pybullet as p
        joint_angles = p.calculateInverseKinematics(self.sim.robot_id, self.sim.end_effector_link, target_pos)
        # apply angles to first N joints
        angles_dict = {i: joint_angles[i] for i in range(len(joint_angles))}
        self.sim.set_joint_angles_list(angles_dict)
        return joint_angles
