#!/usr/bin/env python

import rospy
import actionlib

import speechToTextPalbator.msg

import time
from std_msgs.msg import String
import json

class STTClient(object):

    def __init__(self):
        rospy.init_node("STT_clientOnline_node")

        self.action_online_client = actionlib.SimpleActionClient("action_STT_online",speechToTextPalbator.msg.SttOnlineAction)

        # self.action_offline_client = actionlib.SimpleActionClient("action_STT_offline",speechToTextPalbator.msg.SttOfflineAction)

        self.rate=rospy.Rate(1)
        self.enable_listening=None

        self.connection_ON=None
        rospy.loginfo("Waiting for online server...")
        self.action_online_client.wait_for_server()
        rospy.loginfo("Connected to server")

        # rospy.loginfo("Waiting for offline server...")
        # self.action_offline_client.wait_for_server()
        # rospy.loginfo("Connected to offline server")

        # self.sub = rospy.Subscriber("connection_state",String,self.handle_change_connection_state)
        

    def routine_both(self):
        self.goal_online = speechToTextPalbator.msg.SttOnlineGoal()
        order={
            'order': 1,
            'action': "askName",
            'speech': "What is your name ?"
        }
        json_in_str=json.dumps(order)
        self.goal_online.order=json_in_str

        self.goal_offline = speechToTextPalbator.msg.SttOfflineGoal()
        self.goal_offline.order=json_in_str

        self.action_online_client.send_goal(self.goal_online)
        # self.action_offline_client.send_goal(self.goal_offline)


        while self.action_online_client.get_result() is None and not rospy.is_shutdown():
            rospy.logwarn("Waiting ....")
            self.rate.sleep()

        if self.action_online_client.get_result() != None:
            rospy.loginfo("RESULT ONLINE "+str(self.action_online_client.get_result()))
        
        

    def routine_online(self):
        rospy.logwarn("---------------------ONLINE ROUTINE----------------------")
        self.goal = speechToTextPalbator.msg.SttOnlineGoal()
        # time.sleep(5)

        rospy.loginfo("Sending goal to online ...")

        order={
            'scenario': 'cleanup',
            'order': 1,
            'action': "askRoom",
            'speech': "What is your name ?"
        }
        json_in_str=json.dumps(order)
        self.goal.order=json_in_str

        self.action_online_client.send_goal(self.goal)
        
        self.action_online_client.wait_for_result()

        rospy.loginfo(str(self.action_online_client.get_result()))
        rospy.logwarn("---------------------END ONLINE ROUTINE----------------------")

    # def routine_offline(self):
    #     rospy.logwarn("---------------------OFFLINE ROUTINE----------------------")
    #     self.goal = speechToTextPalbator.msg.SttOfflineGoal()

    #     time.sleep(5)

    #     rospy.loginfo("Sending goal to offline...")

    #     order={
    #         'order': 1,
    #         'action': "askName"
    #     }
    #     json_in_str=json.dumps(order)
    #     self.goal.order=json_in_str

    #     self.action_offline_client.send_goal(self.goal)
        
    #     self.action_offline_client.wait_for_result()

    #     rospy.loginfo(str(self.action_offline_client.get_result()))
    #     rospy.logwarn("---------------------END OFFLINE ROUTINE----------------------")

    # def handle_change_connection_state(self,req):
    #     if req.data=='Connected':
    #         self.action_offline_client.cancel_all_goals()
    #         self.connection_ON=True
    #         rospy.loginfo("Connecteedddddddddddddddd")
            

    #     elif req.data=='Disconnected':
    #         self.action_online_client.cancel_all_goals()
    #         self.connection_ON=False
    #         rospy.loginfo("Disconnecteeeeeeeeeeeeeeeeeeeeeeeed")
            


        
if __name__ == "__main__":
    instance_client=STTClient()
    instance_client.routine_online()
    # while not rospy.is_shutdown():
    #     # if instance_client.connection_ON==True:
    #     #     instance_client.routine_online()
    #     # elif instance_client.connection_ON==False:
    #     #     instance_client.routine_offline()
    #     instance_client.routine_both()
    #     instance_client.rate.sleep()