#!/usr/bin/python
__authors__='Thomas Curé, Abdeljabar Ferdjani'

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
        self.rate=rospy.Rate(1)
        self.dir_path = os.path.dirname(os.path.realpath(__file__))
        self.Step_to_load=None

        self.JSON_scenario=None
        self.scenario_step_list=None

        self.scenario_index=None
        self.scenario_current_view=None

        # self.sub_request_hri_restart=rospy.Subscriber("HRI_restart_request",String,self.handle_restart_request)
        
        self.client_action_GmToHri=actionlib.SimpleActionClient("action_GmToHri",GmToHriAction)
        rospy.loginfo("wait for action server")
        self.client_action_GmToHri.wait_for_server()

        self.sub_choice_scenario=rospy.Subscriber("choice_scenario",String,self.handle_choice_scenario)
        rospy.loginfo("GM test initiated")

        
        rospy.loginfo("wait for choice scenario")
        # rospy.wait_for_message("choice_scenario",String)
    
    # def handle_restart_request(self,req):
    #     rospy.loginfo("HRI RESTART REQUEST")
    #     rospy.loginfo("CANCELLING CURRENT ROSACTION...")
    #     self.client_action_GmToHri.cancel_all_goals()
    #     rospy.loginfo("waiting for new scenario choice...")

    def handle_choice_scenario(self,req):

        rospy.loginfo("CHOICE SCENARIO: "+req.data)
        if req.data=='present_school':
            with open(self.dir_path+'/templates/public/json/present_school_test/scenario.json') as pres_school:
                self.JSON_scenario = js.load(pres_school)
                self.JSON_scenario['name']='present_school'


        elif req.data=='creation_test':
            with open(self.dir_path+'/templates/public/json/creation_test/scenario.json') as creationTest:
                self.JSON_scenario = js.load(creationTest)
                self.JSON_scenario['name']='creation_test'

        
        elif req.data=='inspection':
            with open(self.dir_path+'/templates/public/json/inspection/scenario.json') as inspec:
                self.JSON_scenario = js.load(inspec)
                self.JSON_scenario['name']='inspection'


        elif req.data=='take_out_the_garbage':
            with open(self.dir_path+'/templates/public/json/take_out_the_garbage/scenario.json') as take_garbage:
                self.JSON_scenario = js.load(take_garbage)
                self.JSON_scenario['name']='take_out_the_garbage'

        elif req.data=='receptionist':
            with open(self.dir_path+'/templates/public/json/receptionist/scenario.json') as recep:
                self.JSON_scenario = js.load(recep)
                self.JSON_scenario['name']='receptionist'

        self.scenario_step_list=self.JSON_scenario['steps']
        rospy.loginfo("SCENARIO "+req.data+" LOADED")
        

        self.load_scenario_on_Hri()

    def load_scenario_on_Hri(self):
        data_to_send={
            "whatToDo": "Load scenario",
            "scenario": self.JSON_scenario['name'],
            "stepsList": self.JSON_scenario['steps']
            } 
        json_in_str=js.dumps(data_to_send)
        goal=GmToHriGoal(json_in_str)
        self.client_action_GmToHri.send_goal(goal)
        self.client_action_GmToHri.wait_for_result()
        result_action=self.client_action_GmToHri.get_result()
        resultat=result_action.Gm_To_Hri_output
        json_resultat=js.loads(resultat)
        self.scenario_step_list=json_resultat['data']
        rospy.loginfo("SCENARIO LOADED ON HRI")

        
        self.execute_scenario()
        
    def execute_scenario(self):
        i=0
        while i<len(self.scenario_step_list) and not rospy.is_shutdown():
            rospy.loginfo("NEW STEP")
            data_to_send={
                "whatToDo": "Load step",
                "step": self.scenario_step_list[i],
            }
            json_in_str=js.dumps(data_to_send)
            goal=GmToHriGoal(json_in_str)
            self.client_action_GmToHri.send_goal(goal)
            self.client_action_GmToHri.wait_for_result()
            result_action=self.client_action_GmToHri.get_result()
            resultat=result_action.Gm_To_Hri_output
            json_resultat=js.loads(resultat)
            rospy.loginfo("JSON RESULT "+str(json_resultat))
            if json_resultat['actionName']=="":
                i=i+1
            elif json_resultat['actionName']=='confirm' and json_resultat['dataToUse']=='false':
                i=i-1
            
            elif json_resultat['actionName']=="RESTART HRI":
                rospy.loginfo("Waiting for new scenario")
                break
            else:
                i=i+1
            
            
        
if __name__=="__main__":
    x=TestROS()
    while not rospy.is_shutdown():
        rospy.spin()