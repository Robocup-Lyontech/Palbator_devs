#!/usr/bin/env python

import rospy
import actionlib

import speechToTextPalbator.msg

import time
from std_msgs.msg import String
import json

class STTClient(object):

    def __init__(self):
        rospy.init_node("STT_clientOffline_node")

        self.action_offline_client = actionlib.SimpleActionClient("action_STT_offline",speechToTextPalbator.msg.SttOfflineAction)

        self.rate=rospy.Rate(0.1)
        self.enable_listening=None

        self.connection_ON=None


        rospy.loginfo("Waiting for offline server...")
        self.action_offline_client.wait_for_server()
        rospy.loginfo("Connected to offline server")

        

    def routine_both(self):
        self.goal_online = speechToTextPalbator.msg.SttOnlineGoal()
        order={
            'scenario': 'Receptionist',
            'order': 1,
            'action': "askAge",
            'speech': "How old are you ?"
        }
        json_in_str=json.dumps(order)
        self.goal_online.order=json_in_str

        self.goal_offline = speechToTextPalbator.msg.SttOfflineGoal()
        self.goal_offline.order=json_in_str

        # self.action_online_client.send_goal(self.goal_online)
        self.action_offline_client.send_goal(self.goal_offline)


        while self.action_offline_client.get_result() is None and not rospy.is_shutdown():
            rospy.logwarn("Waiting ....")
            self.rate.sleep()

              
        if self.action_offline_client.get_result() != None:
            rospy.loginfo("RESULT OFF "+str(self.action_offline_client.get_result()))
        

    def routine_offline(self):
        rospy.logwarn("---------------------OFFLINE ROUTINE----------------------")
        self.goal = speechToTextPalbator.msg.SttOfflineGoal()

        rospy.loginfo("Sending goal to offline...")

        order={
            'scenario': 'Receptionist',
            'order': 1,
            'action': "askAge",
            'speech': "How old are you ?"
        }
        json_in_str=json.dumps(order)
        self.goal.order=json_in_str

        self.action_offline_client.send_goal(self.goal)
        
        self.action_offline_client.wait_for_result()

        rospy.loginfo(str(self.action_offline_client.get_result()))
        rospy.logwarn("---------------------END OFFLINE ROUTINE----------------------")

            


        
if __name__ == "__main__":
    instance_client=STTClient()
    # while not rospy.is_shutdown():
    instance_client.routine_offline()
    instance_client.rate.sleep()