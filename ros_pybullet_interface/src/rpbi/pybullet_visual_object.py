from .pybullet_object import PybulletObject
from .pybullet_object_pose import PybulletObjectPose

class PybulletVisualObject(PybulletObject):

    """Simply visualizes an object in Pybullet."""

    def init(self):

        # Get visual shape index
        self.base_visual_shape_index = self.create_visual_shape(self.config['createVisualShape'])

        # Setup object pose handler
        self.pose = PybulletObjectPose(self)

        # Check if pose is static
        if self.pose.is_static:
            # object base is specified as static -> get base frame and apply
            self.pose.get_base_from_tf()
            pos, rot = self.pose.get()
            self.body_unique_id = self.pb.createMultiBody(baseVisualShapeIndex=self.base_visual_shape_index, basePosition=pos, baseOrientation=rot)
        else:
            # object base is non static
            self.body_unique_id = self.pb.createMultiBody(baseVisualShapeIndex=self.base_visual_shape_index)
            self.pose.start_resetter()
