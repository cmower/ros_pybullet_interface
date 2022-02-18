from .config import load_config


class PybulletObject:


    def __init__(self, pb, node, config_filename):

        # Set pybullet instance and ROS node
        self.pb = pb
        self.node = node

        # Load config
        self.config = load_config(config_filename)

        # Initialize object
        self.init()


    def init(self):
        raise NotImplementedError('a child class of PybulletObject needs to implement an init method')


    def create_visual_shape(self):
        create_visual_shape_input = self.config.get('createVisualShape', {})
        create_visual_shape_input['shapeType'] = getattr(self.pb, create_visual_shape_input['shapeType'])  # expect string
        return self.pb.createVisualShape(**create_visual_shape_input)


    def create_collision_shape(self):
        create_collision_shape_input = self.config.get('createCollisionShape', {})
        create_collision_shape_input['shapeType'] = getattr(self.pb, create_collision_shape_input['shapeType'])  # expect string
        return self.pb.createCollisionShape(**create_collision_shape_input)