from .pybullet_object import PybulletObject


class PybulletCollisionObject(PybulletObject):

    """Visualizes an object. Other objects will react to this, but it will remain unaffected."""

    def init(self):

        self.base_visual_shape_index = self.create_visual_shape(self.config['createVisualShape'])
        self.base_collision_shape_index = self.create_collision_shape(self.config['createCollisionShape'])

        object_tf_config = self.config.get('object_tf', {})
        self.offset = self.get_object_offset_in_base_tf(object_tf_config)

        if object_tf_config.get('is_static', True):
            self.base = self.get_static_object_base_tf_in_world(object_tf_config)
            pos, rot = self.get_base_position_and_orientation(self.offset, self.base)
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
            self.start_object_base_tf_listener(object_tf_config)
            self.start_applying_non_static_object_tf(object_tf_config)

        # Set dynamics
        self.change_dynamics(self.config['changeDynamics'])
