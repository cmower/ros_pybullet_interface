
class Links:

    def __init__(self, pb_obj, joints, urdf):
        self.pb_obj = pb_obj
        self.urdf = urdf
        self.joints = joints
        self.root_link_name = self.get_root_link_name()  # HACK: see method below

        if self.broadcast_link_states:
            dt = self.pb_obj.node.Duration(1.0/float(self.broadcast_link_states_hz))
            self.pb_obj.timers['publish_link_states'] = self.pb_obj.node.Timer(dt, self._broadcast_link_states)

    def _broadcast_link_states(self, event):

        # Get base position/orientation for root link
        pos, rot = self.pb_obj.pb.getBasePositionAndOrientation(self.pb_obj.body_unique_id)
        self.pb_obj.node.tf.set_tf('rpbi/world', f'rpbi/{self.pb_obj.name}/{self.root_link_name}', pos, rot)

        # Iterate over joints
        link_states = self.pb_obj.pb.getLinkStates(self.pb_obj.body_unique_id, self.joints.indices, computeForwardKinematics=1)
        for joint, link_state in zip(self.joints, link_states):
            self.pb_obj.node.tf.set_tf('rpbi/world', f'rpbi/{self.pb_obj.name}/{joint.linkName}', link_state[4], link_state[5])

    @property
    def broadcast_link_states(self):
        return self.pb_obj.config.get('broadcast_link_states', False)

    @property
    def broadcast_link_states_hz(self):
        return self.pb_obj.config.get('broadcast_link_states_hz', 50)

    def get_root_link_name(self):
        """Return the root link name."""
        # HACK: since I haven't been able to find how to retrieve the root link name from pybullet, I had to use urdf_parser_py instead
        from urdf_parser_py import urdf
        with open(self.urdf.filename, 'r') as f:
            robot = urdf.Robot.from_xml_string(f.read())
        return robot.get_root()
