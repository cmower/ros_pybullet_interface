from .pybullet_object import PybulletObject
from .pybullet_object_pose import PybulletObjectPose

class PybulletVisualObject(PybulletObject):

    """Simply visualizes an object in Pybullet."""

    def init(self):

        # Get visual shape index
        self.base_visual_shape_index = self.create_visual_shape(self.createVisualShape)

        # Setup object pose handler
        self.pose = PybulletObjectPose(self)

        # Check if pose is static
        if self.pose.is_static:
            # object base is specified as static -> get base frame and apply
            self.pose.get_base_from_tf()
            pos, rot = self.pose.get()
            self.body_unique_id = self.pb.createMultiBody(baseVisualShapeIndex=self.base_visual_shape_index, basePosition=pos, baseOrientation=rot)
            if self.pose.broadcast_tf:
                self.pose.start_pose_broadcaster()
        else:
            # object base is non static
            self.body_unique_id = self.pb.createMultiBody(baseVisualShapeIndex=self.base_visual_shape_index)
            self.pose.start_resetter()
            if self.pose.broadcast_tf:
                self.node.logwarn("can not broadcast a non-static pose")

    @property
    def createVisualShape(self):
        return self.config['createVisualShape']
