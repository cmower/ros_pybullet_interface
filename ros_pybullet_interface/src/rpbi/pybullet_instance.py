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

        # Load config
        config_filename = self.node.get_param('~pybullet_config_filename', '')
        self.config = load_config(config_filename) if config_filename else {}

        # Connect to pybullet
        self.client_id = self.pb.connect(self.pb.GUI)
        if self.client_id == -1:
            raise RuntimeError('Unable to connect to Pybullet!')
        self.node.loginfo(f'connected to Pybullet with client id {self.client_id}')

        # Reset simulation
        self.pb.resetSimulation()

        # Set gravity
        if 'gravity' in self.config.keys():
            gravity = self.config['gravity']
            self.pb.setGravity(
                gravX=gravity[0],
                gravY=gravity[1],
                gravZ=gravity[2],
            )

        # Setup time step
        hz = self.config.get('time_step_frequency', 50)
        dt = 1.0/float(hz)
        self.dt = dt
        self.pb.setTimeStep(dt)

        # Get user option to start pybullet from initialization
        self.start_pybullet_after_initialization = self.config.get('start_pybullet_after_initialization', True)

        # Setup services
        self.node.Service('rpbi/start', Trigger, self.service_start)
        self.node.Service('rpbi/step', Trigger, self.service_step)
        self.node.Service('rpbi/stop', Trigger, self.service_stop)

        # Setup publisher and start status timer
        self.status_pub = self.node.Publisher('rpbi/status', Int64, queue_size=10)
        status_hz = self.config.get('status_frequency', 50)
        status_dt = 1.0/float(status_hz)
        self.node.Timer(self.node.Duration(status_dt), self.publish_status)
        self.node.logdebug('initialized Pybullet instance')


    def publish_status(self, event):
        """Timer callback for publishing the status of Pybullet interface."""
        self.status_pub.publish(Int64(data=int(self.is_active)))


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


    def _service(self, handle):
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


    def service_start(self, req):
        """Service callback for starting Pybullet."""
        return self._service(self.start)


    def service_step(self, req):
        """Service callback for stepping Pybullet."""
        return self._service(self.step)


    def service_stop(self, req):
        """Service callback for stopping Pybullet."""
        return self._service(self.stop)


    def close(self):
        """Close connection to Pybullet."""
        self.pb.disconnect()
