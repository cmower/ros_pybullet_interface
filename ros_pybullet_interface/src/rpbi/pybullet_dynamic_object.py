import numpy as np
import tf_conversions
from .pybullet_object import PybulletObject
from .pybullet_object_pose import PybulletObjectPose


class PybulletDynamicObject(PybulletObject):

    """Objects motion defined by Pybullet."""

    def init(self):

        # Get visual and collision shape indices
        self.base_visual_shape_index = self.create_visual_shape(self.createVisualShape)
        self.base_collision_shape_index = self.create_collision_shape(createCollisionShape)

        # Setup object pose
        self.pose = PybulletObjectPose(self)
        self.pose.get_base_from_tf()
        pos, ori = self.pose.get()

        # Setup multi body
        self.body_unique_id = self.pb.createMultiBody(
            baseMass=self.baseMass,
            baseVisualShapeIndex=self.base_visual_shape_index,
            baseCollisionShapeIndex=self.base_collision_shape_index,
            basePosition=pos,
            baseOrientation=ori,
        )

        # Reset initial velocity
        if self.reset_base_velocity is not None:
            self.pb.resetBaseVelocity(self.body_unique_id, **self.reset_base_velocity)

        # Set dynamics
        self.change_dynamics(self.changeDynamics)

        # Broadcast pose
        if self.pose.broadcast_tf:
            self.pose.start_pose_broadcaster()

    @property
    def baseMass(self):
        return self.config['baseMass']

    @property
    def createVisualShape(self):
        return self.config['createVisualShape']


    @property
    def createCollisionShape(self):
        return self.config['createCollisionShape']

    @property
    def changeDynamics(self):
        return self.config['changeDynamics']

    @property
    def reset_base_velocity(self):
        return self.config.get('resetBaseVelocity')
