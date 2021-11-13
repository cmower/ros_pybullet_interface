#!/usr/bin/env python3
import rospy
import tf2_ros
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker


WORLD_FRAME_ID = 'ros_pybullet_interface/world'


class Republish4Rviz(object):
    """docstring for ."""

    def __init__(self):

        self.name  = "RVIZ_marker_republisher"
        self.TF_listen_buff = tf2_ros.Buffer()
        listener = tf2_ros.TransformListener(self.TF_listen_buff)

        self.pub_plan_box = rospy.Publisher('/visualization_plan_box', Marker, queue_size=100)
        self.pub_real_box = rospy.Publisher('/visualization_real_box', Marker, queue_size=100)
        self.pub_yin_sphere = rospy.Publisher('/visualization_yin_sphere', Marker, queue_size=100)
        self.pub_yang_sphere = rospy.Publisher('/visualization_yang_sphere', Marker, queue_size=100)

        self.pub_predict_box = rospy.Publisher('/visualization_predict_box', Marker, queue_size=100)

        marker = Marker()
        self.plan_box_marker = self.makeMarker(marker, marker.CUBE, 0.55, 0.405, 0.415, 0.4, 0.5, 0.5, 0.5)
        marker = Marker()
        self.real_box_marker = self.makeMarker(marker, marker.CUBE, 0.55, 0.405, 0.415, 0.5, 0.5, 0.2, 0.3)
        marker = Marker()
        self.yin_sphere = self.makeMarker(marker, marker.SPHERE, 0.05, 0.05, 0.05, 1, 0.8, 0.8, 0.0)
        marker = Marker()
        self.yang_sphere = self.makeMarker(marker, marker.SPHERE, 0.05, 0.05, 0.05, 1, 0.3, 0.3, 0.8)

        markerTraj = Marker()
        self.predict_box_marker = self.makeMarker(markerTraj, marker.LINE_STRIP, 0.01, 0.01, 0.01, 1, 0.3, 0.3, 0.8)


    def makeMarker(self, marker, type, x,y,z, a, r,g,b):

        marker.header.frame_id = "ros_pybullet_interface/world"
        marker.type = type
        marker.action = marker.ADD
        marker.scale.x = x
        marker.scale.y = y
        marker.scale.z = z
        marker.color.a = a
        marker.color.r = r
        marker.color.g = g
        marker.color.b = b

        return marker


    def read_publ_all(self):

        self.read_publish("ros_pybullet_interface/target_visual", self.pub_plan_box, self.plan_box_marker)
        self.read_publish("vicon_offset/hanging_obj_plate/hanging_obj_plate", self.pub_real_box, self.real_box_marker)

        self.read_publish("ros_pybullet_interface/sphere", self.pub_yin_sphere, self.yin_sphere)
        self.read_publish("ros_pybullet_interface/sphere2", self.pub_yang_sphere, self.yang_sphere)

        self.read_publish_traj("ros_pybullet_interface/target_visual", self.pub_predict_box, self.predict_box_marker)



    def read_publish(self, TF_id, pub, marker):

        try:
            # Read the position and orientation of the robot from the /tf topic
            trans = self.TF_listen_buff.lookup_transform(WORLD_FRAME_ID, f"{TF_id}", rospy.Time())
        except:
            rospy.logwarn(f"{self.name}: /tf topic does NOT have {TF_id}")
            return

        marker.pose.position.x = trans.transform.translation.x
        marker.pose.position.y = trans.transform.translation.y
        marker.pose.position.z = trans.transform.translation.z
        marker.pose.orientation.x = trans.transform.rotation.x
        marker.pose.orientation.y = trans.transform.rotation.y
        marker.pose.orientation.z = trans.transform.rotation.z
        marker.pose.orientation.w = trans.transform.rotation.w

        pub.publish(marker)

    def read_publish_traj(self, TF_id, pub, marker):

        try:
            # Read the position and orientation of the robot from the /tf topic
            trans = self.TF_listen_buff.lookup_transform(WORLD_FRAME_ID, f"{TF_id}", rospy.Time())
        except:
            rospy.logwarn(f"{self.name}: /tf topic does NOT have {TF_id}")
            return

        # marker.pose.position.x = trans.transform.translation.x
        # marker.pose.position.y = trans.transform.translation.y
        # marker.pose.position.z = trans.transform.translation.z
        # marker.pose.orientation.x = trans.transform.rotation.x
        # marker.pose.orientation.y = trans.transform.rotation.y
        # marker.pose.orientation.z = trans.transform.rotation.z
        # marker.pose.orientation.w = trans.transform.rotation.w

        line_point = Point()
        line_point.x = trans.transform.translation.x
        line_point.y = trans.transform.translation.y
        line_point.z = trans.transform.translation.z

        marker.points.append(line_point)

        pub.publish(marker)



if __name__=='__main__':

    rospy.init_node("rviz_visualization")

    RepubRviz = Republish4Rviz()
    rate = rospy.Rate(500)


    while not rospy.is_shutdown():
        RepubRviz.read_publ_all()
        rate.sleep()
