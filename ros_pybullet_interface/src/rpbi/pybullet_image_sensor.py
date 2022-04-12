from sensor_msgs.msg import Image
from .pybullet_sensor import PybulletSensor

class PybulletImageSensor(PybulletSensor):

    def init(self):
        self.pubs['image'] = self.node.Publisher('rpbi/image', Image, queue_size=10)
        self.timers['mainloop'] = self.node.Timer(self.dt, self.main_loop)

    @property
    def dt(self):
        return self.node.Duration(1.0/float(self.config.get('hz', 30)))

    @property
    def get_camera_image(self):
        get_camera_image = self.config['getCameraImage']
        if 'renderer' in get_camera_image:
            get_camera_image['renderer'] = self.node.parse_options(get_camera_image['renderer'])
        if 'flags' in get_camera_image:
            get_camera_image['flags'] = self.node.parse_options(get_camera_image['flags'])
        return get_camera_image

    def convert_camera_image_to_image(self, width, height, rgbPixels, depthPixels, segmentationMaskBuffer):
        msg = Image()
        msg.header.stamp = self.node.time_now()
        # >>>>>>>>>>>>>TODO<<<<<<<<<<<<
        return msg

    def main_loop(self, event):

        # Get camera image from Pybullet, see getCameraImage method here:
        # https://docs.google.com/document/d/10sXEhzFRSnvFcl3XxNGhnD4N2SedqwdAvK3dsihxVUA/edit#heading=h.u1jisfnt6984
        #
        # TODO: expose other getCameraImage parameters so that user
        # can define view/projection matrix from a tf (if possible?)
        img = self.pb.getCameraImage(**self.get_camera_image)
        width = img[0]
        height = img[1]
        rgbPixels = img[2]
        depthPixels = img[3]
        segmentationMaskBuffer = img[4]

        # Convert to point cloud and publish
        self.pubs['image'].publish(
            self.convert_camera_image_to_image(width, height, rgbPixels, depthPixels, segmentationMaskBuffer)
        )
