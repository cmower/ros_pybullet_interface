#!/usr/bin/env python3
import rospy
import numpy
from rpbi_work.srv import EffPoseFromObject, EffPoseFromObjectResponse
from ros_pybullet_interface.tf_interface import TfInterface
import tf_conversions

"""
Need from Christian
- box dimensions, Lx, Ly, Lz as ROS parameters (see below)
- TF frame ID for object linked to the necessary parent frame
"""

quaternion_matrix = tf_conversions.transformations.quaternion_matrix

class Node:


    def __init__(self):

        # Setup ros
        rospy.init_node('get_eff_pose_from_pushing_object_node')

        # Get offset
        self.offset = numpy.array([
            rospy.get_param('~offset_x'),
            rospy.get_param('~offset_y'),
            rospy.get_param('~offset_z'),
        ])

        # Setup tf interface
        self.tf = TfInterface()

        # Setup service
        rospy.Service('get_eff_pose_from_pushing_object', EffPoseFromObject, self.get_eff_pose_from_pushing_object)


    def get_eff_pose_from_pushing_object(self, req):

        offset = self.offset + numpy.array([-0.5*req.xlength, 0, 0])

        resp_input = dict(
            success=True,
            box_lx=req.xlength,
            box_ly=req.ylength,
            box_lz=0,
        )

        pos, rot = self.tf.get_tf(req.parent_frame_id, req.object_frame_id)
        if pos is not None:
            R = quaternion_matrix(rot)[:3,:3]
            resp_input['position'] = numpy.array(pos) + R@offset
            resp_input['rotation'] = rot
            frame = '%s_hand_goal' % req.arm
            self.tf.set_tf(req.parent_frame_id, frame, resp_input['position'], rot)
            rospy.loginfo('Successfully recieved TF frame %s in %s', req.object_frame_id, req.parent_frame_id)
        else:
            resp_input['success'] = False
            resp_input['position'] = [0.0]*3
            resp_input['rotation'] = [0.0]*3 + [1.0]
            rospy.logerr('Failed to get TF frame %s in %s', req.object_frame_id, req.parent_frame_id)

        return EffPoseFromObjectResponse(**resp_input)

    def spin(self):
        rospy.spin()


def main():
    Node().spin()


if __name__ == '__main__':
    main()
