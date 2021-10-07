import pybullet

"""Instance of pybullet."""


class Pybullet:

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

    def __init__(self, config):

        # Set class attributes
        self.active = False

        # Set class attributes
        self.config = config

        # Get real time indicator
        self.enable_real_time_simulation = self.config.get('enable_real_time_simulation', True)

        # Connect to pybullet
        self.client_id = pybullet.connect(pybullet.GUI)
        if self.client_id == -1:
            raise RuntimeError('Unable to connect to Pybullet!')

        # Reset simulation
        pybullet.resetSimulation(physicsClientId=self.client_id)

        # Init visualizer
        pybullet.configureDebugVisualizer(flag=pybullet.COV_ENABLE_GUI, enable=0)
        camera_config = [
            config.get('camera_distance', 1),
            config.get('camera_yaw', 0),
            config.get('camera_pitch', 1),
        ]
        camera_config += config.get('camera_target_position', [0, 0, 0])
        self.reset_debug_visualizer_camera(camera_config)
        self.configure_camera_from_ros = config.get('configure_camera_from_ros', False)

        # Initialize gravity
        pybullet.setGravity(*config.get('gravity', [0, 0, -9.807]))

        # Initialize time step
        self.hz = config.get('pybullet_sampling_frequency', 100)
        self.dt = 1.0/float(self.hz)
        pybullet.setTimeStep(self.dt)

    def start_real_time_simulation(self):
        pybullet.setRealTimeSimulation(1)
        self.active = True

    def stop_real_time_simulation(self):
        pybullet.setRealTimeSimulation(0)
        self.active = False

    def reset_debug_visualizer_camera_callback(self, ros_float64multiarray_msg):
        self.reset_debug_visualizer_camera(ros_float64multiarray_msg.data)

    def reset_debug_visualizer_camera(self, config):
        pybullet.resetDebugVisualizerCamera(
            cameraDistance=config[0],
            cameraYaw=config[1],
            cameraPitch=config[2],
            cameraTargetPosition=config[3:7],
        )

    def step(self, nsteps, ros_rate):
        self.active = True
        for _ in range(nsteps):
            pybullet.stepSimulation()
            ros_rate.sleep()
        self.active = False

    def shutdown(self):
        pybullet.disconnect()
