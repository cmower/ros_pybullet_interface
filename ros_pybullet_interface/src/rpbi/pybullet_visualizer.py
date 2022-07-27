from custom_ros_tools.config import load_config
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from std_msgs.msg import Int64MultiArray
import numpy as np
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

        self.pub_debug_vis = self.node.Publisher('rpbi/debug_visualizer_camera', ResetDebugVisualizerCamera, queue_size=10)
        self.node.Timer(self.node.Duration(1.0/30.0), self.debug_visualizer_camera_publish)


        # Setup publisher/timer for visualizer image
        self.cv_bridge = CvBridge()
        if self.publish_visualizer_image_hz > 0:
            self.visualizer_image_pub = self.node.Publisher(
                'rpbi/visualizer_image', Image, queue_size=10
            )
            self.visualizer_image_int_pub = self.node.Publisher(
                'rpbi/visualizer_image_int', Int64MultiArray, queue_size=10
            )
            dt = 1.0/float(self.publish_visualizer_image_hz)
            self.node.Timer(self.node.Duration(dt), self.publish_visualizer_image)

        # Setup service
        self.node.Service('rpbi/get_debug_visualizer_camera', GetDebugVisualizerCamera, self.service_get_debug_visualizer_camera)

        self.node.loginfo('initialized Pybullet visualizer')

    def debug_visualizer_camera_publish(self, event):
        msg = ResetDebugVisualizerCamera()
        for key, value in self.reset_debug_visualizer_camera.items():
            setattr(msg, key, value)
        self.pub_debug_vis.publish(msg)

    @property
    def publish_visualizer_image_hz(self):
        return self.node.config.get('publish_visualizer_image_hz', 0)

    @property
    def visualizer_image_height(self):
        return self.node.config.get('visualizer_image_height', 480)

    @property
    def visualizer_image_width(self):
        return self.node.config.get('visualizer_image_width', 640)

    def publish_visualizer_image(self, event):

        # Get image
        _, _, colour, _, _ = self.pb.getCameraImage(
            self.visualizer_image_width,
            self.visualizer_image_height,
            renderer=self.pb.ER_BULLET_HARDWARE_OPENGL,
        )

        self.visualizer_image_int_pub.publish(Int64MultiArray(data=colour.flatten().astype(int).tolist()))

        # Pack message
        msg = self.cv_bridge.cv2_to_imgmsg(colour[...,:3], encoding="rgb8")
        msg.header.stamp = self.node.time_now()

        # Publish image
        self.visualizer_image_pub.publish(msg)

    @property
    def configure_debug_visualizer(self):

        # Get from config
        configure_debug_visualizer = self.node.config.get('configureDebugVisualizer', {})
        for key, default_value in self.configure_debug_visualizer_default.items():
            if key not in configure_debug_visualizer:
                configure_debug_visualizer[key] = default_value

        # Handle flags
        if 'flag' in configure_debug_visualizer:
            configure_debug_visualizer['flag'] = self.node.parse_options(configure_debug_visualizer['flag'])

        return configure_debug_visualizer

    @property
    def reset_debug_visualizer_camera(self):
        reset_debug_visualizer_camera = self.node.config.get('resetDebugVisualizerCamera', {})
        for key, default_value in self.reset_debug_visualizer_camera_default.items():
            if key not in reset_debug_visualizer_camera:
                reset_debug_visualizer_camera[key] = default_value
        return reset_debug_visualizer_camera

    @reset_debug_visualizer_camera.setter
    def reset_debug_visualizer_camera(self, msg):
        assert isinstance(msg, ResetDebugVisualizerCamera), "reset_debug_visualizer_camera"
        if 'resetDebugVisualizerCamera' not in self.node.config: self.node.config['resetDebugVisualizerCamera'] = {}
        for key in self.reset_debug_visualizer_camera_default.keys():
            self.node.config['resetDebugVisualizerCamera'][key] = getattr(msg, key)

    def service_get_debug_visualizer_camera(self, req):
        resp = GetDebugVisualizerCameraResponse()
        for key, value in self.reset_debug_visualizer_camera.items(): setattr(resp.debug_visualizer_config, key, value)
        return resp

    def callback(self, msg):
        """Callback for subscriber listening for visualizer pose updates."""
        self.reset_debug_visualizer_camera = msg
        self.pb.resetDebugVisualizerCamera(**self.reset_debug_visualizer_camera)
