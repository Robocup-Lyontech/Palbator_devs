#!/usr/bin/env python

import rospy

from speechToTextPalbator.msg import SpeechRecognitionAction, SpeechRecognitionGoal
import actionlib
import json


class ClientSTT():

    def __init__(self):
        rospy.init_node("test_client_STT_node")

        self.client = actionlib.SimpleActionClient('STT_action_server', SpeechRecognitionAction)

        rospy.loginfo("Connecting to STT server ...")
        self.client.wait_for_server()
        rospy.loginfo("Connected to STT")

        rospy.sleep(3)

        goal = SpeechRecognitionGoal()
        json_goal = {
            "order": 1,
            "action": "askAge",
            "scenario": "Receptionist",
            "Connection_state": "Online"
        }
        goal.order = json.dumps(json_goal)

        # while not rospy.is_shutdown():

        self.client.send_goal(goal)

        rospy.sleep(3)
        self.client.cancel_all_goals()
        self.client.wait_for_result()
        result = self.client.get_result()
        rospy.loginfo("RESULT %s",str(result))

if __name__ == "__main__":
    a = ClientSTT()