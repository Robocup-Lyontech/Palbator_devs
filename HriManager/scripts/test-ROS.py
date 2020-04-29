#!/usr/bin/python

import rospy
from std_msgs.msg import String
# from rospy_message_converter import json_message_converter
import json
from HriManager.msg import GmToHriAction, GmToHriGoal
import actionlib
import os
import json as js

class TestROS(object):
    def __init__(self):
        rospy.init_node("test",anonymous=True)
        # self.sub=rospy.Subscriber("test_JSON",String,self.handle_sub)
        # self.pub=rospy.Publisher("SendData",String,queue_size=10)
        # self.pub2=rospy.Publisher("SendData",String,queue_size=10)
        self.rate=rospy.Rate(1)
        # self.sub_test=rospy.Subscriber("test",String,self.handle_test)
        self.dir_path = os.path.dirname(os.path.realpath(__file__))


        self.sub_choice_scenario=rospy.Subscriber("choice_scenario",String,self.handle_choice_scenario)
        rospy.loginfo("wait for choice scenario")
        rospy.wait_for_message("choice_scenario",String)
        self.client_action_GmToHri=actionlib.SimpleActionClient("action_GmToHri",GmToHriAction)
        rospy.loginfo("wait for action server")
        self.client_action_GmToHri.wait_for_server()


        # goal=GmToHriGoal("je suis une patate")
        # self.client_action_GmToHri.send_goal(goal)
        # self.client_action_GmToHri.wait_for_result()
        # result_action=self.client_action_GmToHri.get_result()
        # rospy.loginfo("ACTION RESULT: "+str(result_action))#.Gm_To_Hri_output))


    def handle_choice_scenario(self,req):
        if req.data=='present_school':
            with open(self.dir_path+'/templates/public/json/present_school_test/scenario.json') as pres_school:
                self.presentSchool = js.load(pres_school)

        elif req.data=='creation_test':
            with open(self.dir_path+'/templates/public/json/creation_test/scenario.json') as creationTest:
                self.creation = js.load(creationTest)
        
        elif req.data=='inspection':
            with open(self.dir_path+'/templates/public/json/inspection/scenario.json') as inspec:
                self.inspection = js.load(inspec)

        elif req.data=='take_out_the_garbage':
            with open(self.dir_path+'/templates/public/json/take_out_the_garbage/scenario.json') as take_garbage:
                self.takeOutGarbage = js.load(take_garbage)
        elif req.data=='receptionist':
            with open(self.dir_path+'/templates/public/json/receptionist/scenario.json') as recep:
                self.receptionist = js.load(recep)

        rospy.loginfo("CHOICE SCENARIO: "+req.data)
    def handle_test(self,req):
        if req.data=="go":
            self.client_action_GmToHri.cancel_all_goals()
    # def publisher(self):
    #     x={u'dataToUse': u'ETI', u'index': 2}
    #     json_str=json.dumps(x)
    #     # y={u'dataToUse': u'false', u'index': 3}
    #     # json_str2=json.dumps(y)
        
    #     self.pub.publish(json_str)
    #     # self.pub2.publish(json_str2)
        


    # def handle_sub(self,req):
    #     json_str=json_message_converter.convert_ros_message_to_json(String(req.data))
    #     d=json.loads(json_str)
    #     print(type(d))
    #     print(d)
    #     print("try ", d['data'])

if __name__=="__main__":
    x=TestROS()
    while not rospy.is_shutdown():
        # x.publisher()
        x.rate.sleep()