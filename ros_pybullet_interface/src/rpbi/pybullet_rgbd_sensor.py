from std_msgs.msg import Header
from sensor_msgs.msg import Image, CameraInfo, PointCloud2, PointField
from sensor_msgs.point_cloud2 import create_cloud
from .pybullet_sensor import PybulletSensor
from cv_bridge import CvBridge
import numpy as np
import struct
from .pybullet_object_pose import PybulletObjectPose
from tf_conversions import transformations

class PybulletRGBDSensor(PybulletSensor):

    def init(self):
        self.cv_bridge = CvBridge()

        self.cfg_camera = self.config['intrinsics']

        self.pubs['colour'] = self.node.Publisher('rpbi/camera/colour/image', Image, queue_size=10)
        self.pubs['depth'] = self.node.Publisher('rpbi/camera/depth/image', Image, queue_size=10)
        self.pubs['ci/c'] = self.node.Publisher('rpbi/camera/colour/camera_info', CameraInfo, queue_size=10)
        self.pubs['ci/d'] = self.node.Publisher('rpbi/camera/depth/camera_info', CameraInfo, queue_size=10)

        self.pubs['segmentation'] = self.node.Publisher('rpbi/camera/segmentation', Image, queue_size=10)

        # publish point cloud?
        self.pub_pc = self.config.get("pointcloud", False)

        if self.pub_pc:
            self.pubs['pointcloud'] = self.node.Publisher('rpbi/camera/pointcloud', PointCloud2, queue_size=10)

        self.timers['mainloop'] = self.node.Timer(self.dt, self.main_loop)

        # TODO: expose other getCameraImage parameters so that user
        # can define view/projection matrix from a tf (if possible?)

        # intrinsics
        self.w = self.cfg_camera.get("width", 640)
        self.h = self.cfg_camera.get("height", 480)
        fov = self.cfg_camera.get("fov", 40)
        depth_range = self.cfg_camera.get("range", [0.01, 100])
        self.near = depth_range[0]
        self.far = depth_range[1]
        self.pm = self.pb.computeProjectionMatrixFOV(fov = fov, aspect = self.w / self.h, nearVal = self.near, farVal = self.far)

        # bullet only supports a scalar field-of-view
        # determine the horizontal field-of-view from aspect ratio
        fovs = fov * np.array([self.w / self.h, 1])
        # focal length in pixels
        self.fs = (0.5 * np.array([self.w, self.h])) / np.tan(np.deg2rad(fovs)/2)
        self.K = np.array([[self.fs[0], 0, self.w/2],
                           [0, self.fs[1], self.h/2],
                           [0, 0,          1       ]])
        self.P = np.concatenate((self.K, np.zeros((3,1))), axis=1)

        if self.pub_pc:
            # precompute coordinates
            uv = np.dstack(np.meshgrid(range(self.w), range(self.h), indexing='xy'))
            uv_list = uv.reshape((-1,2))
            # projection up to scale (scale with metric depth)
            xy = (uv_list - np.array([self.w/2, self.h/2])) / self.fs
            self.xy1 = np.concatenate((xy, np.ones((uv_list.shape[0],1))), axis=1)


        # Setup object pose
        self.pose = PybulletObjectPose(self)

        if self.pose.is_static:
            self.pose.get_base_from_tf()
            if self.pose.broadcast_tf:
                self.pose.start_pose_broadcaster()
        else:
            self.pose.start_resetter()
            if self.pose.broadcast_tf:
                self.node.logwarn('can not broadcast a non-static pose')

    @property
    def dt(self):
        return self.node.Duration(1.0/float(self.config.get('hz', 30)))

    def main_loop(self, event):

        # extrinsics
        p, q = self.pose.get()
        T = transformations.translation_matrix(p) @ transformations.quaternion_matrix(q)
        # convert from OpenCV coordinate frame to OpenGL coordinate frame
        # rotate 180 deg about x-axis (have y and z point in the opposite direction)
        T[:,1:3] *= -1
        T = np.linalg.inv(T)
        # serialise column-wise
        vm = T.T.ravel()

        (width, height, colour, depth_gl, segmentation) = \
            self.pb.getCameraImage(self.w, self.h, vm, self.pm, renderer=self.pb.ER_BULLET_HARDWARE_OPENGL)

        depth = self.far * self.near / (self.far - (self.far - self.near) * depth_gl)

        hdr = Header()
        hdr.stamp = self.node.time_now()
        # hdr.frame_id = 'rpbi/camera'
        hdr.frame_id = self.pose.base_tf_id

        # publish colour and depth image
        msg_colour = self.cv_bridge.cv2_to_imgmsg(colour[...,:3], encoding="rgb8")
        msg_colour.header = hdr
        msg_depth = self.cv_bridge.cv2_to_imgmsg(depth, encoding="passthrough")
        msg_depth.header = hdr

        self.pubs['colour'].publish(msg_colour)
        self.pubs['depth'].publish(msg_depth)

        # publish segmentation
        # NOTE: This will wrap around 16 bit unsigned integer (0 to 65,535)
        msg_segm = self.cv_bridge.cv2_to_imgmsg(segmentation.astype(np.uint16), encoding="mono16")
        msg_segm.header = hdr
        self.pubs['segmentation'].publish(msg_segm)

        # publish intrinsic parameters
        ci = CameraInfo()
        ci.header = hdr
        ci.width = width
        ci.height = height
        ci.K = self.K.ravel()
        ci.P = self.P.ravel()
        self.pubs['ci/c'].publish(ci)
        self.pubs['ci/d'].publish(ci)

        if self.pub_pc:
            # projection to 3D via precomputed coordinates
            points = self.xy1 * depth.reshape((-1,1))
            # pack 3 x uint8 into float32 in order (0,r,g,b)
            rgb_f = np.empty((points.shape[0],1))
            for i, rgb in enumerate(colour[...,:3].reshape((-1,3))):
                rgb_f[i] = struct.unpack('>f', struct.pack('4B', 0, *rgb))[0]

            fields = [PointField('x', 0, PointField.FLOAT32, 1),
                      PointField('y', 4, PointField.FLOAT32, 1),
                      PointField('z', 8, PointField.FLOAT32, 1),
                      PointField('rgb', 12, PointField.FLOAT32, 1)]

            msg_pc = create_cloud(hdr, fields, np.concatenate((points, rgb_f), axis=1))

            self.pubs['pointcloud'].publish(msg_pc)
