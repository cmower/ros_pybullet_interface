import rospy
from .config import load_config

class PybulletVisualizer:

    def __init__(self):

        # Load config
        config_filename = rospy.get_param('~visualizer_config_filename')
        self.config = load_config(config_filename)
