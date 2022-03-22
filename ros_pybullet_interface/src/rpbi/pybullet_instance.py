from functools import partial
from .config import load_config
from std_msgs.msg import Int64
from std_srvs.srv import Trigger, TriggerResponse


class PybulletInstance:

    """Interface to the main Pybullet instance."""

    def __init__(self, pb, node):

        # Set pybullet instance and ROS node
        self.pb = pb
        self.node = node

        # Setup variables
        self.is_active = False

        # Connect to pybullet
        self.client_id = self.pb.connect(self.pb.GUI)
        if self.client_id == -1:
            raise RuntimeError('Unable to connect to Pybullet!')
        self.node.loginfo(f'connected to Pybullet with client id {self.client_id}')

        # Reset simulation
        self.pb.resetSimulation()

        # Set gravity
        if 'gravity' in self.node.config:
            g = self.node.config['gravity']
            self.pb.setGravity(gravX=g[0], gravY=g[1], gravZ=g[2])

        # Setup time step
        self.dt = self.node.config.get('timeStep', 0.02)
        self.pb.setTimeStep(self.dt)

        # Setup services
        self.node.Service('rpbi/start', Trigger, partial(self._service, handle=self.start))
        self.node.Service('rpbi/step', Trigger, partial(self._service, handle=self.step))
        self.node.Service('rpbi/stop', Trigger, partial(self._service, handle=self.stop))

        # Setup status publisher
        self.status_publisher = StatusPublisher(self)

        self.node.loginfo('initialized Pybullet instance')


    @property
    def start_pybullet_after_initialization(self):
        return self.node.config.get('start_pybullet_after_initialization', True)


    def start(self):
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


    def stop(self):
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
        hz = self.instance.node.config.get('status_frequency', 50)
        dt = self.node.Duration(1.0/float(hz))
        self.timer = self.instance.node.Timer(dt, self.publish_status)

    def publish_status(self, event):
        """Timer callback for publishing the status of Pybullet interface."""
        self.pub.publish(Int64(data=int(self.instance.is_active)))

    def stop(self):
        self.timer.shutdown()
