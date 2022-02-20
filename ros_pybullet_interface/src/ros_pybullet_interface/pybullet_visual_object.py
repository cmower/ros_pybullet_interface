from .pybullet_object import PybulletObject

class PybulletVisualObject(PybulletObject):

    def init(self):

        self.base_visual_shape_index = self.create_visual_shape(self.config['createVisualShape'])

        object_tf_config = self.config.get('object_tf', {})
        self.offset = self.get_object_offset_in_base_tf(object_tf_config)

        if object_tf_config.get('is_static', True):
            # object base is world or specified as static -> get base frame and apply
            self.base = get_static_object_base_tf_in_world(object_tf_config)
            pos, rot = self.get_base_position_and_orientation(self.offset, self.base)
            self.body_unique_id = self.pb.createMultiBody(baseVisualShapeIndex=self.base_visual_shape_index, basePosition=pos, baseOrientation=rot)
        else:
            # object base is non static
            self.body_unique_id = self.pb.createMultiBody(baseVisualShapeIndex=self.base_visual_shape_index)
            self.start_object_base_tf_listener(object_tf_config)
            self.start_applying_non_static_object_tf(object_tf_config)
