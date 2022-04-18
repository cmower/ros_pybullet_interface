from .pybullet_object import PybulletObject
from .pybullet_object_pose import PybulletObjectPose

class PybulletVisualObject(PybulletObject):

    """Simply visualizes an object in Pybullet."""

    def init(self):

        # Get visual shape index
        self.base_visual_shape_index = self.create_visual_shape(self.createVisualShape)

        # Setup body unique id
        self.body_unique_id = self.pb.createMultiBody(baseVisualShapeIndex=self.base_visual_shape_index)

        # Setup object pose handler
        self.pose = PybulletObjectPose(self)
        if self.pose.tf_specified():
            self.pose.start_reset_pose()

    @property
    def createVisualShape(self):
        return self.config['createVisualShape']
