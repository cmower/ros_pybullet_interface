#!/usr/bin/env python3
from rpbi.ros_node import RosNode
from cob_srvs.srv import SetString, SetStringRequest


class Node(RosNode):


    interval = 3.0


    def __init__(self):
        RosNode.__init__(self, 'pybullet_objects_example_node')
        self.it = 0
        self.add_pybullet_object()
        self.Timer(self.Duration(self.interval), self.main_loop)


    def add_pybullet_object(self):
        srv = 'rpbi/add_pybullet_dynamic_object'
        self.wait_for_service(srv)
        try:
            handle = self.ServiceProxy(srv, SetString)
            req = SetStringRequest(data='{rpbi_examples}/configs/pybullet_objects_example/dynamic_box.yaml')
            resp = handle(req)
            if resp.success:
                self.loginfo('successfully added object')
            else:
                self.logwarn('failed to add object: %s' % resp.message)
        except Exception as e:
            self.logerr('failed to add object: %s' % str(e))



    def remove_object(self):
        srv = 'rpbi/remove_pybullet_object'
        self.wait_for_service(srv)
        try:
            handle = self.ServiceProxy(srv, SetString)
            req = SetStringRequest(data="dynamic_box")
            resp = handle(req)
            if resp.success:
                self.loginfo('successfully removed object')
            else:
                self.logwarn('failed to remove object: %s' % resp.message)
        except Exception as e:
            self.logerr('failed to remove object: %s' % str(e))


    def main_loop(self, event):
        self.remove_object()
        self.add_pybullet_object()
        self.it += 1


def main():
    Node().spin()


if __name__ == '__main__':
    main()
