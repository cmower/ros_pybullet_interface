from .pybullet_object import PybulletObject
from .pybullet_object_pose import PybulletObjectPose


class PybulletCollisionObject(PybulletObject):

    """Visualizes an object. Other objects will react to this, but it will remain unaffected."""

    def init(self):

        # Get visual and collision shape indices
        self.base_visual_shape_index = self.create_visual_shape(self.config['createVisualShape'])
        self.base_collision_shape_index = self.create_collision_shape(self.config['createCollisionShape'])

        # Setup object pose handler
        self.pose = PybulletObjectPose(self)

        if self.pose.is_static:
            self.pose.get_base_from_tf()
            pos, rot = self.pose.get()
            self.body_unique_id = self.pb.createMultiBody(
                baseMass=0.0,
                baseVisualShapeIndex=self.base_visual_shape_index,
                baseCollisionShapeIndex=self.base_collision_shape_index,
                basePosition=pos,
                baseOrientation=rot
            )
        else:
            self.body_unique_id = self.pb.createMultiBody(
                baseMass=0.0,
                baseVisualShapeIndex=self.base_visual_shape_index,
                baseCollisionShapeIndex=self.base_collision_shape_index
            )
            self.pose.start_resetter()

        # Set dynamics
        self.change_dynamics(self.config['changeDynamics'])
