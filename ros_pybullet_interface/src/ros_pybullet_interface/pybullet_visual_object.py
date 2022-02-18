from .pybullet_object import PybulletObject

class PybulletVisualObject(PybulletObject):

    def init(self):

        # Initialize the visual object in Pybullet
        self.base_visual_shape_index = self.create_visual_shape(self.config['createVisualShape'])
        self.body_unique_id = self.pb.createMultiBody(baseVisualShapeIndex=self.base_visual_shape_index)

        # Setup object
        self.get_frame_offset()

        # Get object base tf frame id
        self.setup_object_base_tf_frame()
