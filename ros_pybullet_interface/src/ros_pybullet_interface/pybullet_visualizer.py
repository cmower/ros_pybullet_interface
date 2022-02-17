from .config import load_config
from ros_pybullet_interface.msg import ResetDebugVisualizerCamera

class PybulletVisualizer:

    reset_debug_visualizer_camera_default = {
        'cameraDistance': 1.0,
        'cameraYaw': 0.0,
        'cameraPitch': 1.0,
        'cameraTargetPosition': [0.0]*3,
    }

    def __init__(self, pb, node):

        # Set pybullet instance and ROS node
        self.pb = pb
        self.node = node

        # Load config
        config_filename = self.node.get_param('~visualizer_config_filename', '')
        self.config = load_config(config_filename) if config_filename else {}

        # Initialize visualizer
        self.pb.configureDebugVisualizer(
            flag=self.pb.COV_ENABLE_GUI,
            enable=0,
        )
        self.camera_config = {
            key: self.config.get(key, self.reset_debug_visualizer_camera_default[key])
            for key in self.reset_debug_visualizer_camera.keys()
        }
        self.pb.resetDebugVisualizerCamera(**self.camera_config)

        # Setup subcscriber
        self.node.Subscriber('rpbi/reset_debug_visualizer_camera', ResetDebugVisualizerCamera, self.callback)
        self.node.logdebug('initialized Pybullet visualizer')

    def callback(self, msg):
        self.camera_config = {
            key: getattr(msg, key, self.reset_debug_visualizer_camera_default[key])
            for key in self.reset_debug_visualizer_camera.keys()
        }
        self.pb.resetDebugVisualizerCamera(**self.camera_config)
