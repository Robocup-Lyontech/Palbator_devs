#!/usr/bin/env python

import rospy
from speechToTextPalbator.msg import SpeechRecognitionAction, SpeechRecognitionGoal
import actionlib
import json as js

class ClientSTT():

    def __init__(self):

        rospy.init_node('client_test_procedure_STT')
        _stt_server_name = rospy.get_param("~stt_server_name")
        self.action_stt_client = actionlib.SimpleActionClient(_stt_server_name,SpeechRecognitionAction)

        rospy.loginfo("{class_name} : Waiting for STT server...".format(class_name=self.__class__.__name__))
        self.action_stt_client.wait_for_server()
        rospy.loginfo("{class_name} : Connected to STT server".format(class_name=self.__class__.__name__))

    def vocal_detection(self,connectionstate):

        rospy.loginfo("{class_name} : TEST DETECTION %s".format(class_name=self.__class__.__name__),connectionstate)


        goal_stt = SpeechRecognitionGoal()
        data_STT = None
        json_goal = {
            "order": 1,
            "action": "askName",
            "scenario": "Receptionist",
            "connection_state": connectionstate
        }


        goal_stt.order = js.dumps(json_goal)

        self.action_stt_client.send_goal(goal_stt)

        self.action_stt_client.wait_for_result()
        result = self.action_stt_client.get_result()
        rospy.loginfo("{class_name} : STT result : %s".format(class_name=self.__class__.__name__),str(result))


if __name__ == "__main__":
    instance_client = ClientSTT()


    print("------------------ \n Test speech recognition online. Make sure your internet connection is available.\n --------------------")
    raw_input("To test the online speech recognition, please say one of the names in the list (which will appear later) when the mic will listen. To run the test, press Enter")
    instance_client.vocal_detection('Online')

    response = raw_input("Do you want to test one more time the online speech recognition ? (Y/N)")
    while response =="Y" or response == "y":
        response = None
        instance_client.vocal_detection('Online')
        response = raw_input("Do you want to test one more time the online speech recognition ? (Y/N)")

    print("------------------ \n Test speech recognition offline. \n --------------------")
    raw_input("To test the offline speech recognition, please say one of the names in the list (which will appear later) when the mic will listen. To run the test, press Enter")
    instance_client.vocal_detection("Offline")
    response = raw_input("Do you want to test one more time the offline speech recognition ? (Y/N)")
    while response =="Y" or response == "y":
        response = None
        instance_client.vocal_detection('Offline')
        response = raw_input("Do you want to test one more time the offline speech recognition ? (Y/N)")
