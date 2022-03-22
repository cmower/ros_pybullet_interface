import numpy as np
import tf_conversions
from .pybullet_object import PybulletObject
from .utils import TimeoutExceeded


class PybulletDynamicObject(PybulletObject):

    """Objects motion defined by Pybullet."""

    def init(self):

        self.base_visual_shape_index = self.create_visual_shape(self.config['createVisualShape'])
        self.base_collision_shape_index = self.create_collision_shape(self.config['createCollisionShape'])

        object_tf_config = self.config.get('object_tf', {})
        self.offset = self.get_object_offset_in_base_tf(object_tf_config)
        self.base = self.get_static_object_base_tf_in_world(object_tf_config)
        pos, rot = self.get_base_position_and_orientation(self.offset, self.base)

        self.body_unique_id = self.pb.createMultiBody(
            baseMass=self.config['baseMass'],
            baseVisualShapeIndex=self.base_visual_shape_index,
            baseCollisionShapeIndex=self.base_collision_shape_index,
            basePosition=pos,
            baseOrientation=rot,
        )

        self.tf_frame_id = object_tf_config.get('tf_frame_id', f'rpbi/{self.name}')

        reset_base_velocity_input = self.config.get('resetBaseVelocity')
        if reset_base_velocity_input is not None:
            self.pb.resetBaseVelocity(self.body_unique_id, **reset_base_velocity_input)

        if object_tf_config.get('broadcast_tf', False):
            freq = object_tf_config.get('broadcast_frequency', 50)
            self.timers['broadcast_dynamic_object_tf'] = self.node.Timer(self.node.Duration(1.0/float(freq)), self.broadcast_dynamic_object_tf)
        # Set dynamics
        self.change_dynamics(self.config['changeDynamics'])

    def broadcast_dynamic_object_tf(self, event):
        """Grabs the current position and orientation of the dynamic object and broadcasts as a ROS tf using tf2 library."""
        pos, ori = self.pb.getBasePositionAndOrientation(self.body_unique_id)
        self.node.tf.set_tf('rpbi/world', self.tf_frame_id, pos, ori)
