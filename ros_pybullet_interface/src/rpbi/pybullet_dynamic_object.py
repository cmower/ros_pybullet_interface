import numpy as np
from .pybullet_object import PybulletObject
from .pybullet_object_pose import PybulletObjectPose


class PybulletDynamicObject(PybulletObject):

    """Objects motion defined by Pybullet."""

    def init(self):

        # Get visual and collision shape indices
        self.base_visual_shape_index = self.create_visual_shape(self.createVisualShape)
        self.base_collision_shape_index = self.create_collision_shape(self.createCollisionShape)

        # Setup multi body
        self.body_unique_id = self.pb.createMultiBody(
            baseMass=self.baseMass,
            baseVisualShapeIndex=self.base_visual_shape_index,
            baseCollisionShapeIndex=self.base_collision_shape_index,
            basePosition=self.basePosition,
            baseOrientation=self.baseOrientation,
        )

        # Reset initial velocity
        if self.reset_base_velocity is not None:
            self.pb.resetBaseVelocity(self.body_unique_id, **self.reset_base_velocity)

        # Set dynamics
        self.change_dynamics(self.changeDynamics)

        # Broadcast pose
        if self.broadcast_tf:
            dt = self.node.Duration(1.0/float(self.broadcast_hz))
            self.timers['broadcaster_dyn_obj_pose'] = self.node.Timer(dt, self.broadcast_pose)

    @property
    def broadcast_tf(self):
        return self.config.get('broadcast_tf', True)

    @property
    def broadcast_hz(self):
        return self.congig.get('broadcast_hz', 30)

    @property
    def baseMass(self):
        return self.config['baseMass']

    @property
    def basePosition(self):
        return self.config.get('basePosition', [0.0]*3)

    @property
    def baseOrientation(self):
        return self.config.get('baseOrientation', [0., 0., 0., 1.])

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

    def broadcast_pose(self, event):
        pos, quat = self.pb.getBasePositionAndOrientation(self.body_unique_id)
        self.node.tf.set_tf('rpbi/world', f'rpbi/{self.name}', pos, quat)
