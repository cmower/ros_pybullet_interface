from .config import load_config
from ros_pybullet_interface.msg import ResetDebugVisualizerCamera
from ros_pybullet_interface.srv import GetDebugVisualizerCamera, GetDebugVisualizerCameraResponse

class PybulletVisualizer:

    """Interface for the Pybullet visualizer."""

    reset_debug_visualizer_camera_default = {
        'cameraDistance': 1.0,
        'cameraYaw': 0.0,
        'cameraPitch': 1.0,
        'cameraTargetPosition': [0.0]*3,
    }

    configure_debug_visualizer_default = dict(
        flag='COV_ENABLE_GUI',
        enable=0,
    )

    def __init__(self, pb, node):

        # Set pybullet instance and ROS node
        self.pb = pb
        self.node = node

        # Initialize visualizer
        self.pb.configureDebugVisualizer(**self.configure_debug_visualizer)

        # Initialize camera pose
        self.pb.resetDebugVisualizerCamera(**self.reset_debug_visualizer_camera)

        # Setup subcscriber
        self.node.Subscriber('rpbi/reset_debug_visualizer_camera', ResetDebugVisualizerCamera, self.callback)

        # Setup service
        self.node.Service('rpbi/get_debug_visualizer_camera', GetDebugVisualizerCamera, self.service_get_debug_visualizer_camera)

        self.node.loginfo('initialized Pybullet visualizer')

    @property
    def configure_debug_visualizer(self):

        # Get from config
        configure_debug_visualizer = self.node.config.get('configureDebugVisualizer', {})
        for key, default_value in self.configure_debug_visualizer_default.items():
            if key not in configure_debug_visualizer:
                configure_debug_visualizer[key] = default_value

        # Handle flags
        if 'flag' in configure_debug_visualizer:

            flag_success = True

            if isinstance(configure_debug_visualizer['flag'], int):
                pass
            elif isinstance(configure_debug_visualizer['flag'], str):
                flag = ['self.pb.' + f for f in configure_debug_visualizer['flag'].split('|')]
                configure_debug_visualizer['flag'] = eval('|'.join(flag))
            elif isinstance(configure_debug_visualizer['flag'], list):
                if all(isinstance(f, (str, int)) for f in configure_debug_visualizer['flag']):

                    # Set initial flag
                    def to_int(f):
                        if isinstance(f, str):
                            return eval('self.pb.' + f)
                        else:
                            return f

                    flag = to_int(configure_debug_visualizer['flag'][0])

                    # Iterate over remaining flags
                    for f in configure_debug_visualizer['flag'][1:]:
                        flag |= to_int(f)

                    # Set flag
                    configure_debug_visualizer['flag'] = eval('|'.join(flag))

                else:
                    flag_success = False
            else:
                flag_success = False

            if not flag_success:
                raise TypeError("the type used for the flag argument in configureDebugVisualizer config is not recognized")

        return configure_debug_visualizer

    @property
    def reset_debug_visualizer_camera(self):
        return self.node.config.get('resetDebugVisualizerCamera', self.reset_debug_visualizer_camera_default)

    @reset_debug_visualizer_camera.setter
    def reset_debug_visualizer_camera(self, msg):
        assert isinstance(msg, ResetDebugVisualizerCamera), "reset_debug_visualizer_camera"
        if 'resetDebugVisualizerCamera' not in self.node.config: self.node.config['resetDebugVisualizerCamera'] = {}
        for key in self.reset_debug_visualizer_camera_default.keys():
            self.node.config['resetDebugVisualizerCamera'][key] = getattr(msg, key)

    def service_get_debug_visualizer_camera(self, req):
        resp = GetDebugVisualizerCameraResponse()
        for key, value in self.reset_debug_visualizer_camera.items(): setattr(resp, key, value)
        return resp

    def callback(self, msg):
        """Callback for subscriber listening for visualizer pose updates."""
        self.reset_debug_visualizer_camera = msg
        self.pb.resetDebugVisualizerCamera(**self.reset_debug_visualizer_camera)
