from functools import partial
from std_msgs.msg import Int64
from std_srvs.srv import Trigger, TriggerResponse


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

        # Reset simulation
        self.pb.resetSimulation()

        # Set gravity
        self.pb.setGravity(**self.setGravity)

        # Setup time step
        self.dt = self.timeStep
        self.pb.setTimeStep(self.dt)

        # Setup start/stop methods
        if self.step_pybullet_manually:
            self.start = self.start_manual
            self.stop = self.stop_manual
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
            self.pb.stepSimulation()
            self.node.sleep(self.dt)
            self.is_active = False
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

    def publish_status(self, event):
        """Timer callback for publishing the status of Pybullet interface."""
        self.pub.publish(Int64(data=int(self.instance.is_active)))

    def stop(self):
        self.timer.shutdown()
