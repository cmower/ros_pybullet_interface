"""Instance of pybullet."""


class PybulletInstance:

    #
    # Config expects:
    #
    # camera_distance: float
    #      Distance from eye to camera target position, default is 1.
    #
    # camera_yaw: float
    #      Camera yaw angle (degrees) left/right, default is 0.
    #
    # camera_pitch: float
    #      Camera pitch angle (degrees) up/down, default is 1.
    #
    # camera_target_position: float[3]
    #      Camera focus point, default is [0,0,0].
    #
    # configure_camera_from_ros: bool
    #      If true, a subscriber will listen for
    #      std_msgs/Float64MultiArray messages on the rpbi/camera
    #      topic. The message data must contain the following
    #      information:
    #      msg.data = [
    #       camera_distance,
    #       camera_yaw,
    #       camera_pitch,
    #       camera_target_position[0],
    #       camera_target_position[1],
    #       camera_target_position[2],
    #      ]
    #      If false, the camera is static and based on mouse
    #      interactions as normal.
    #      Default is False.
    #
    # gravity: float[3]
    #      Gravity force along the [x, y, z] world axes, default is
    #      [0, 0, -9.807].
    #
    # pybullet_sampling_frequency: int
    #      Pybullet internal sampling frequency, default is 100.
    #
    # enable_real_time_simulation: bool
    #      If true, then the simulation is run in
    #      real-time. Otherwise, the simulator must be manually
    #      stepped. Default is true.
    #

    def __init__(self, config, pybullet):

        # Set class attributes
        self.pb = pybullet
        self.active = False

        # Set class attributes
        self.config = config

        # Get flags
        self.enable_real_time_simulation = self.config.get('enable_real_time_simulation', True)
        self.publish_visualizer_to_ros = self.config.get('publish_visualizer_to_ros', False)

        # Connect to pybullet
        self.client_id = self.pb.connect(self.pb.GUI)
        if self.client_id == -1:
            raise RuntimeError('Unable to connect to Pybullet!')

        # Reset simulation
        self.pb.resetSimulation(physicsClientId=self.client_id)

        # Init visualizer
        self.pb.configureDebugVisualizer(flag=self.pb.COV_ENABLE_GUI, enable=0)
        self.camera_config = None
        camera_config = [
            config.get('camera_distance', 1),
            config.get('camera_yaw', 0),
            config.get('camera_pitch', 1),
        ]
        camera_config += config.get('camera_target_position', [0, 0, 0])
        self.reset_debug_visualizer_camera(camera_config)
        self.configure_camera_from_ros = config.get('configure_camera_from_ros', False)

        # Initialize gravity
        self.pb.setGravity(*config.get('gravity', [0, 0, -9.807]))

        # Initialize time step
        self.hz = config.get('pybullet_sampling_frequency', 100)
        self.dt = 1.0/float(self.hz)
        self.pb.setTimeStep(self.dt)

    def start_real_time_simulation(self):
        self.pb.setRealTimeSimulation(1)
        self.active = True

    def stop_real_time_simulation(self):
        self.pb.setRealTimeSimulation(0)
        self.active = False

    def reset_debug_visualizer_camera_callback(self, ros_float64multiarray_msg):
        self.reset_debug_visualizer_camera(ros_float64multiarray_msg.data)

    def reset_debug_visualizer_camera(self, config):
        self.camera_config = config
        self.pb.resetDebugVisualizerCamera(
            cameraDistance=config[0],
            cameraYaw=config[1],
            cameraPitch=config[2],
            cameraTargetPosition=config[3:7],
        )

    def get_visualizer_image(self):
        output = self.pb.getCameraImage(
            width=self.config['camera_width'],
            height=self.config['camera_height'],
        )
        rgb_pixels = output[2]
        return rgb_pixels


    def step(self, nsteps, ros_rate):
        self.active = True
        for _ in range(nsteps):
            self.pb.stepSimulation()
            ros_rate.sleep()
        self.active = False

    def shutdown(self):
        self.pb.disconnect()
