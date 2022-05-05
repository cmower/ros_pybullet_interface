from .pybullet_object import PybulletObject
from custom_ros_tools.config import replace_package

class PybulletSoftBodyObject(PybulletObject):

    def init(self):

        if self.load_method() == 'loadSoftBody':
            self.body_unique_id = self.pb.loadSoftBody(**self.load_soft_body)
        elif sself.load_method() == 'loadURDF':
            self.body_unique_id = self.pb.loadURDF(**self.load_urdf)

        for anchor in self.create_soft_body_anchor:
            self.pb.createSoftBodyAnchor(*anchor)

    def load_method(self):
        if 'loadSoftBody' in self.config:
            return 'loadSoftBody'
        elif 'loadURDF' in self.config:
            return 'loadURDF'

    @property
    def create_soft_body_anchor(self):
        """

        NOTE: the documentation for createSoftBodyAnchor is
        limited. It is not clear what is exactly the interface.

        In the config, you can create anchors by parsing a list to
        createSoftBodyAnchor, e.g.

        createSoftBodyAnchor:
        - [A1, A2, ...]
        - [A1, A2, ...]

        The soft body unique ID will be passed automatically, but any
        other parameters must be supplied. Some potential resources:

        - https://github.com/bulletphysics/bullet3/blob/master/examples/pybullet/examples/deformable_anchor.py
        - https://github.com/bulletphysics/bullet3/discussions/4088
        - https://github.com/bulletphysics/bullet3/blob/7dee3436e747958e7088dfdcea0e4ae031ce619e/examples/pybullet/pybullet.c#L2280-L2326

        """
        return self.config.get('createSoftBodyAnchor', [])        

    @property
    def load_urdf(self):
        load_urdf = self.config['loadURDF']
        load_urdf['fileName'] = replace_package(load_urdf['fileName'])
        if 'flags' in load_urdf:
            load_urdf['flags'] = self.node.parse_options(load_urdf['flags'])
        return load_urdf
        
        
    @property
    def load_soft_body(self):
        load_soft_body = self.config['loadSoftBody']
        if 'fileName' in load_soft_body:
            load_soft_body['fileName'] = replace_package(load_soft_body['fileName'])
        if 'simFileName' in load_soft_body:
            load_soft_body['simFileName'] = replace_package(load_soft_body['simFileName'])
        return load_soft_body
            
