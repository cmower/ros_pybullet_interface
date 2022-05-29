from .pybullet_object import PybulletObject
from .pybullet_robot_urdf import URDF

class PybulletURDF(PybulletObject):

    def init(self):
        self.is_visual_robot = False  # HACK: so we can use the URDF class in pybullet_robot_urdf
        self.urdf = URDF(self)
        self.body_unique_id = self.urdf.load()

    @property
    def loadURDF(self):
        return self.config['loadURDF']

    
