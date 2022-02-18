class PybulletRobot:

    def __init__(self, pb, node, config):

        # Set pybullet instance and ROS node
        self.pb = pb
        self.node = node

        # Init config
        self.config = config
        self.name = self.config['name']
        del self.config['name']
