from .pybullet_object import PybulletObject

class PybulletCollisionObject(PybulletObject):

    def init(self):

        # Initialize the collision object in Pybullet
        self.base_visual_shape_index = self.create_visual_shape(self.config['createVisualShape'])
        self.base_collision_shape_index = self.create_collision_shape(self.config['createCollisionShape'])
        self.body_unique_id = self.pb.createMultiBody(
            baseMass=0.0,
            baseVisualShapeIndex=self.base_visual_shape_index,
            baseCollisionShapeIndex=self.base_collision_shape_index,
        )

        # Set dynamics
        self.change_dynamics(self.config['changeDynamics'])

        # Setup object
        self.get_frame_offset()

        # Get object base tf frame id
        self.setup_object_base_tf_frame()
