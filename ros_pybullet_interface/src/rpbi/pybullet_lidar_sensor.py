import numpy as np
from sensor_msgs.msg import LaserScan
from .pybullet_sensor import PybulletSensor
from .pybullet_object_pose import PybulletObjectPose
from scipy.spatial.transform import Rotation as R

pi = np.pi

class PybulletLidarSensor(PybulletSensor):

    def init(self):

        # Setup pose
        self.pose = PybulletObjectPose(self)

        # Setup laser scan message
        self.msg = LaserScan(
            angle_min=self.angle_min,
            angle_max=self.angle_max,
            angle_increment=self.angle_increment,
            range_min=0.0,
            scan_time=self.dt,
            time_increment=0.0,
            range_max=self.range_max,
            ranges=[0.]*self.num_scans,
        )
        self.msg.header.frame_id = 'map' #'rpbi/world'

        # Start sensor timer
        self.pubs['laserscan'] = self.node.Publisher(f'rpbi/lidar/{self.name}', LaserScan, queue_size=10)
        self.timers['mainloop'] = self.node.Timer(self.dur, self.main_loop)

    @property
    def in_deg(self):
        return self.config.get('in_deg', False)

    @property
    def angle_min(self):
        if 'angle_min' in self.config:
            angle_min = self.config['angle_min']
            if self.in_deg:
                angle_min = np.deg2rad(angle_min)
        else:
            angle_min = np.deg2rad(-135.)
        return angle_min

    @property
    def angle_max(self):

        if 'angle_max' in self.config:
            angle_max = self.config['angle_max']
            if self.in_deg:
                angle_max = np.deg2rad(angle_max)
        else:
            angle_max = np.deg2rad(135.)
        return angle_max

    @property
    def num_scans(self):
        return int(np.clip(self.config.get('num_scans', 100), 2, np.inf))

    @property
    def angle_increment(self):
        return (self.angle_max - self.angle_min)/float(self.num_scans-1)

    @property
    def range_max(self):
        return self.config.get('range_max', 1.0)

    @property
    def dt(self):
        return 1.0/float(self.config.get('hz', 50))

    @property
    def dur(self):
        return self.node.Duration(self.dt)

    def main_loop(self, event):

        # Get pose of sensor
        pos, rot = self.pose.get()
        T = self.node.tf.pos_quat_to_matrix(pos, rot)
        rayFromPosition = T[:3, 3].flatten()

        # Get ranges
        for i in range(self.num_scans):
            angle = self.angle_min + float(i)*self.angle_increment
            Ta = R.from_euler('z', angle).as_matrix() @ T[:3, :3]
            rayToPosition = rayFromPosition + self.range_max*Ta[:3,0].flatten()
            ray = self.pb.rayTest(rayFromPosition, rayToPosition)[0]

            if ray[0] == -1:
                r = self.range_max
            else:
                hitPosition = np.array(ray[3])
                r = np.linalg.norm(rayFromPosition - hitPosition)

            self.msg.ranges[i] = r

        # Publish laser scan
        self.msg.header.stamp = self.node.time_now()
        self.pubs['laserscan'].publish(self.msg)
