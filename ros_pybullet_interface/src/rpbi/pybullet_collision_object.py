from .pybullet_object import PybulletObject
from .pybullet_object_pose import PybulletObjectPose


class PybulletCollisionObject(PybulletObject):

    """Visualizes an object. Other objects will react to this, but it will remain unaffected."""

    def init(self):

        # Get visual and collision shape indices
        self.base_visual_shape_index = self.create_visual_shape(self.createVisualShape)
        self.base_collision_shape_index = self.create_collision_shape(self.createCollisionShape)

        # Setup object pose handler
        self.pose = PybulletObjectPose(self)

        if self.pose.config.base.is_static:
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
            self.pose.start_reset_base_position_and_orientation()

        # Set dynamics
        self.change_dynamics(self.changeDynamics)

    @property
    def createVisualShape(self):
        return self.config['createVisualShape']

    @property
    def createCollisionShape(self):
        return self.config['createCollisionShape']

    @property
    def changeDynamics(self):
        return self.config['changeDynamics']
