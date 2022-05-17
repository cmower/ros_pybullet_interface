import pybullet_data
from functools import partial
from std_msgs.msg import Int64
from std_srvs.srv import Trigger, TriggerResponse
from rosgraph_msgs.msg import Clock


class PybulletInstance:

    """Interface to the main Pybullet instance."""

    def __init__(self, pb, node):

        # Set pybullet instance and ROS node
        self.pb = pb
        self.node = node

        # Setup variables
        self.manual_step_timer = None
        self.is_active = False

        # Connect to pybullet
        self.client_id = self.pb.connect(self.connection_mode, **self.connect_kwargs)
        if self.client_id == -1:
            raise RuntimeError('Unable to connect to Pybullet!')
        self.node.loginfo(f'connected to Pybullet with client id {self.client_id}')

        # Set additional search path
        for path in self.set_additional_search_paths:
            if path == 'pybullet_data_path':
                p = pybullet_data.getDataPath()
            else:
                p = path
            self.pb.setAdditionalSearchPath(p)

        # Reset simulation
        self.pb.resetSimulation(**self.reset_simulation)

        # Set gravity
        self.pb.setGravity(**self.setGravity)

        # Setup time step
        self.dt = self.timeStep
        self.timeStepNSecs = int(1e9*self.timeStep)
        self.pb.setTimeStep(self.dt)

        # Set physics engine parameters
        self.pb.setPhysicsEngineParameter(**self.set_physics_engine_parameter)

        # Setup start/stop methods
        self.use_sim_time = False
        self.current_time_nsecs = 0
        self.sim_time_pub = None
        if self.step_pybullet_manually:
            self.start = self.start_manual
            self.stop = self.stop_manual
            self.use_sim_time = self.node.get_param('use_sim_time', False)
            if self.use_sim_time:
                self.sim_time_pub = self.node.Publisher('clock', Clock, queue_size=1)
        else:
            self.start = self.start_real_time_simulation
            self.stop = self.stop_real_time_simulation

        # Setup services
        self.node.Service('rpbi/start', Trigger, partial(self._service, handle=self.start))
        self.node.Service('rpbi/step', Trigger, partial(self._service, handle=self.step))
        self.node.Service('rpbi/stop', Trigger, partial(self._service, handle=self.stop))

        # Setup status publisher
        self.status_publisher = StatusPublisher(self)

        self.node.loginfo('initialized Pybullet instance')

    @property
    def set_additional_search_paths(self):
        set_additional_search_paths = self.node.config.get('setAdditionalSearchPath', [])
        if isinstance(set_additional_search_paths, list):
            return set_additional_search_paths
        elif isinstance(set_additional_search_paths, str):
            return [set_additional_search_paths]
        else:
            raise ValueError(f"did not recognize given paths {set_additional_search_paths}")

    @property
    def set_physics_engine_parameter(self):
        return self.node.config.get('setPhysicsEngineParameter', {})

    @property
    def reset_simulation(self):
        reset_simulation =  self.node.config.copy().get('resetSimulation', {})
        if 'flags' in reset_simulation:
            reset_simulation['flags'] = self.node.parse_options(reset_simulation['flags'])
        return reset_simulation

    @property
    def connection_mode(self):
        connect = self.node.config.copy().get('connect', {})
        connection_mode = connect.get('connection_mode', self.pb.GUI)
        if isinstance(connection_mode, int):
            return connection_mode
        elif isinstance(connection_mode, str):
            return getattr(self.pb, connection_mode)
        else:
            raise ValueError("did not recognize given connection mode")

    @property
    def connect_kwargs(self):
        connect = self.node.config.get('connect', {})  # use copy to prevent connection_mode from being removed
        connect_ = connect.copy()
        if 'connection_mode' in connect:
            del connect_['connection_mode']
        return connect_

    @property
    def start_pybullet_after_initialization(self):
        return self.node.config.get('start_pybullet_after_initialization', True)

    @property
    def setGravity(self):
        set_gravity_default = {'gravX': 0.0, 'gravY': 0.0, 'gravZ': 0.0}
        grav = self.node.config.get('setGravity', {})
        for key, default in set_gravity_default.items():
            if key not in grav:
                grav[key] = default
        return grav

    @property
    def gravity(self):
        return self.node.config.get('gravity', [0.0]*3)

    @property
    def timeStep(self):
        return self.node.config.get('timeStep', 0.02)

    @property
    def step_pybullet_manually(self):
        if self.connection_mode == self.pb.DIRECT:
            # DIRECT requires user to step manually, see:
            # https://pybullet.org/Bullet/phpBB3/viewtopic.php?t=13064
            return True
        else:
            return self.node.config.get('step_pybullet_manually', False)


    def start_manual(self):
        """Start Pybullet"""
        if not self.is_active:
            self.manual_step_timer = self.node.Timer(self.node.Duration(self.timeStep), self._step_manual)
            self.is_active = True
            success = True
            message = 'started Pybullet'
        else:
            success = False
            message = 'attempted to start Pybullet, but it already active!'
            self.node.logerr(message)
        return success, message


    def _step_manual(self, event):
        if self.use_sim_time:
            self.sim_time_pub.publish(Clock(clock=self.node.Time(nsecs=self.current_time_nsecs)))
            self.current_time_nsecs += self.timeStepNSecs
        self.pb.stepSimulation()


    def stop_manual(self):
        """Stop Pybullet"""
        if self.is_active:
            self.manual_step_timer.shutdown()
            self.manual_step_timer = None
            self.is_active = False
            success = True
            message = 'stopped Pybullet'
        else:
            success = False
            message = 'attempted to stop Pybullet, but it is not running!'
            self.node.logerr(messsage)
        return success, message


    def start_real_time_simulation(self):
        """Start Pybullet"""
        if not self.is_active:
            self.pb.setRealTimeSimulation(1)
            self.is_active = True
            success = True
            message = 'started Pybullet'
        else:
            success = False
            message = 'attempted to start Pybullet, but it already active!'
            self.node.logerr(message)
        return success, message


    def stop_real_time_simulation(self):
        """Stop Pybullet"""
        if self.is_active:
            self.pb.setRealTimeSimulation(0)
            self.is_active = False
            success = True
            message = 'stopped Pybullet'
        else:
            success = False
            message = 'attempted to stop Pybullet, but it is not running!'
            self.node.logerr(messsage)
        return success, message


    def step(self):
        """Step Pybullet by one time step."""
        if not self.is_active:
            self.is_active = True
            self.status_publisher.publish_status()
            self.pb.stepSimulation()
            self.node.sleep(self.dt)
            self.is_active = False
            self.status_publisher.publish_status()
            success = True
            message = 'stepped Pybullet'
        else:
            success = False
            message = 'attempted to step Pybullet, but it is already running!'
            self.node.logerr(message)
        return success, message


    def _service(self, req, handle):
        """Abstract method for start/stop/step services."""
        try:
            success, message = handle()
        except Exception as e:
            success = False
            message = 'failed to %s Pybullet, exception: %s' % (handle.__name__, str(e))
            self.node.logerr(message)
        if success:
            self.node.loginfo('%s Pybullet was successful', handle.__name__)
        return TriggerResponse(success=success, message=message)


    def close(self):
        """Close connection to Pybullet."""
        self.status_publisher.stop()
        self.pb.disconnect()


class StatusPublisher:

    def __init__(self, instance):
        self.instance = instance
        self.pub = self.instance.node.Publisher('rpbi/status', Int64, queue_size=10)
        dt = self.instance.node.Duration(1.0/float(self.status_hz))
        self.timer = self.instance.node.Timer(dt, self.publish_status)

    @property
    def status_hz(self):
        return self.instance.node.config.get('status_hz', 50)

    def publish_status(self, event=None):
        """Timer callback for publishing the status of Pybullet interface."""
        self.pub.publish(Int64(data=int(self.instance.is_active)))

    def stop(self):
        self.timer.shutdown()
