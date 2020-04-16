#! /usr/bin/env python

import rospy

import actionlib

import ttsMimic.msg

from std_msgs.msg import String

class TtsClient(object):

    def __init__(self):
        rospy.init_node('TTS_client_node')
        self.sub=rospy.Subscriber("send_goal",String,self.handle_goal)

        self.client=actionlib.SimpleActionClient("action_TTS_MIMIC",ttsMimic.msg.TtsMimicAction)
        rospy.loginfo("waiting for server ....")
        self.client.wait_for_server()
        rospy.loginfo("connected to server")

        self.goal=None

        rospy.loginfo("waiting for goal ....")


    def handle_goal(self,req):
        rospy.loginfo("GOAL RECEIVED")
        self.goal=ttsMimic.msg.TtsMimicGoal(req.data)

        self.client.send_goal(self.goal)

        self.client.wait_for_result()

        rospy.loginfo(str(self.client.get_result()))
    
if __name__ == "__main__":
    instance_client=TtsClient()
    while not rospy.is_shutdown():
        rospy.spin()
