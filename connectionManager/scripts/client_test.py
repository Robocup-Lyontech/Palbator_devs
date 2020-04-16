#!/usr/bin/env python

import rospy
from std_msgs.msg import String


class TestClient(object):

    def __init__(self):
        rospy.init_node("Client_node")
        self.sub=rospy.Subscriber("connection_state",String,self.handle_connection_change)
        self.connection_ON=None
        self.rate=rospy.Rate(1)
        rospy.loginfo("client connected")

    def online_routine(self):
        # for i in range(0,200):
        rospy.loginfo("LALALALA JE SUIS ONLINE")
        self.rate.sleep()
            

    def offline_routine(self):
        # for i in range(0,200):
        rospy.loginfo("LALALALA JE SUIS OFFFFFFFFFFFF")
        self.rate.sleep()

    def handle_connection_change(self,req):
        if req.data=='Connected':
            self.connection_ON=True
            self.online_routine()

        elif req.data=='Disconnected':
            self.connection_ON=False
            self.offline_routine()

if __name__ == "__main__":
    client=TestClient()
    while not rospy.is_shutdown():
        rospy.spin()
    