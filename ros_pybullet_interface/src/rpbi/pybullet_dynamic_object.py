import numpy as np
import tf_conversions
from .pybullet_object import PybulletObject
from .pybullet_object_pose import PybulletObjectPose


class PybulletDynamicObject(PybulletObject):

    """Objects motion defined by Pybullet."""

    def init(self):

        # Get visual and collision shape indices
        self.base_visual_shape_index = self.create_visual_shape(self.config['createVisualShape'])
        self.base_collision_shape_index = self.create_collision_shape(self.config['createCollisionShape'])

        # Setup object pose
        self.pose = PybulletObjectPose(self)
        self.pose.get_base_from_tf()
        pos, ori = self.pose.get()

        # Setup multi body
        self.body_unique_id = self.pb.createMultiBody(
            baseMass=self.config['baseMass'],
            baseVisualShapeIndex=self.base_visual_shape_index,
            baseCollisionShapeIndex=self.base_collision_shape_index,
            basePosition=pos,
            baseOrientation=ori,
        )

        # Reset initial velocity
        reset_base_velocity_input = self.config.get('resetBaseVelocity')
        if reset_base_velocity_input is not None:
            self.pb.resetBaseVelocity(self.body_unique_id, **reset_base_velocity_input)

        # Set dynamics
        self.change_dynamics(self.config['changeDynamics'])

        # Broadcast pose
        if self.pose.broadcast_tf:
            self.pose.start_pose_broadcaster()
