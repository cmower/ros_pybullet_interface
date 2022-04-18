from .pybullet_object import PybulletObject
from .pybullet_object_pose import PybulletObjectPose


class PybulletCollisionObject(PybulletObject):

    """Visualizes an object. Other objects will react to this, but it will remain unaffected."""

    def init(self):

        # Get visual and collision shape indices
        self.base_visual_shape_index = self.create_visual_shape(self.createVisualShape)
        self.base_collision_shape_index = self.create_collision_shape(self.createCollisionShape)

        # Create body unique id
        self.body_unique_id = self.pb.createMultiBody(
            baseMass=0.0,
            baseVisualShapeIndex=self.base_visual_shape_index,
            baseCollisionShapeIndex=self.base_collision_shape_index,
        )

        # Set dynamics
        self.change_dynamics(self.changeDynamics)

        # Setup object pose handler
        self.pose = PybulletObjectPose(self)
        if self.pose.tf_specified():
            self.pose.start_reset_pose()

    @property
    def createVisualShape(self):
        return self.config['createVisualShape']

    @property
    def createCollisionShape(self):
        return self.config['createCollisionShape']

    @property
    def changeDynamics(self):
        return self.config['changeDynamics']
