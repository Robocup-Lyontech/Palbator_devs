#!/usr/bin/env python

import rospy
import yaml
from std_msgs.msg import String

# from scenario.CocktailPartyScenario import CocktailPartyScenario
# from scenario.TestNavigScenario import TestNavigScenario
# from scenario.DoorOpenAndNavigScenario import DoorOpenAndNavigScenario
# from scenario.TestDialogueScenario import TestDialogueScenario
# from scenario.TestCocktailPartyV1Scenario import TestCocktailPartyV1Scenario
# from scenario.TestCocktailPartyV2Scenario import TestCocktailPartyV2Scenario
# from scenario.TestCocktailPartyV3Scenario import TestCocktailPartyV3Scenario
# from scenario.SPRV1Scenario import SPRV1Scenario
# from scenario.HelpMeCarryV1Scenario import HelpMeCarryV1Scenario
# from scenario.TestHoomanoScenario import TestHoomanoScenario
# from scenario.InspectionScenario import InspectionScenario
# from scenario.SPRV2Scenario import SPRV2Scenario
# from scenario.GPRSV1Scenario import GPRSV1Scenario
# from scenario.GPRSV2CPE1Scenario import GPRSV2CPE1Scenario
# from scenario.TakeOutTheGarbage2019v1Scenario import TakeOutTheGarbage2019Scenario
# from scenario.TakeOutTheGarbage2019v2Scenario import TakeOutTheGarbage2019v2Scenario
# from scenario.Receptionist2019CPEScenario import Receptionist2019CPEScenario
# from scenario.TestFaceLearnAndFindPoint import TestFaceLearnAndFindPoint
# from scenario.Inspection2019Scenario import Inspection2019Scenario
from testThomas_Receptionist2020CPEScenario import Receptionist2020CPEScenario
import time
# Command Samples
# rostopic pub /gm_start std_msgs/String "data: 'START'"


class GeneralManager:
    CONFIG_FOLDER = ''
    CURRENT_SCENARIO = None
    _scenarioMap = {}
    _currentScenario = None
    START_STATUS = "START"
    PENDING_STATUS = "PENDING"
    # FIXME ADD ROS PARAM
    _current_status = PENDING_STATUS

    def __init__(self, config):
        rospy.init_node('general_manager')

        rospy.loginfo("CONFIG : "+str(config))

        rospy.loginfo("GM: INITIATING ...")

        # self.waitDoorOpened()
        self.config_GM=config

        self.configure(self.config_GM)

    # def handle_restart_request(self,req):
    #     if req.data=='RESTART':
    #         rospy.loginfo("----------------------------------------------------------------")
    #         rospy.logwarn("GM : SCENARIO RESTARTING ...")
    #         rospy.loginfo("---------------------------------------------------------------")
            
    #         # try:
    #         #     self._currentScenario._lm_wrapper.client_action_GmToHri.cancel_all_goals()
    #         # except Exception as ex:
    #         #     rospy.logwarn("LOG EXCEPTION : "+str(ex))
    #         # rospy.loginfo("GM : ACTION SERVER STATUS : "+str(self._currentScenario._lm_wrapper.client_action_GmToHri.get_status()))
    #         # del self._currentScenario
    #         # time.sleep(5)
    #         # self._currentScenario.startScenario()
    #         self.configure(self.config_GM)

    # def handle_choice_scenario(self,req):
    #     if self.enable_listening_scenario==True:
    #         self.enable_listening_scenario=False
    #         self._currentScenario._lm_wrapper.client_action_GmToHri.cancel_all_goals()
            # self.configure(self.config_GM)
    #     if self.enable_listening_scenario==True:
    #         self.enable_listening_scenario=False
    #         rospy.loginfo("CHOICE SCENARIO: "+req.data)
    #         # if req.data=='present_school':
    #         #     with open(self.dir_path+'/templates/public/json/present_school_test/scenario.json') as pres_school:
    #         #         self.JSON_scenario = js.load(pres_school)
    #         #         self.JSON_scenario['name']='present_school'


    #         # elif req.data=='creation_test':
    #         #     with open(self.dir_path+'/templates/public/json/creation_test/scenario.json') as creationTest:
    #         #         self.JSON_scenario = js.load(creationTest)
    #         #         self.JSON_scenario['name']='creation_test'

            
    #         # elif req.data=='inspection':
    #         #     with open(self.dir_path+'/templates/public/json/inspection/scenario.json') as inspec:
    #         #         self.JSON_scenario = js.load(inspec)
    #         #         self.JSON_scenario['name']='inspection'


    #         # elif req.data=='take_out_the_garbage':
    #         #     with open(self.dir_path+'/templates/public/json/take_out_the_garbage/scenario.json') as take_garbage:
    #         #         self.JSON_scenario = js.load(take_garbage)
    #         #         self.JSON_scenario['name']='take_out_the_garbage'

    #         if req.data=='receptionist':
    #             self.CURRENT_SCENARIO='RECEPTIONIST_2020_CPE'

    #         self.configure(self.config_GM)


    #######################################################################
    #######                Configure Current Node                    ######
    #######################################################################
    def configure(self, config):
        self._start_sub = rospy.Subscriber("gm_start", String, self.gmStartCallback)
        self._start_pub = rospy.Publisher("/gm_start", String, queue_size=1)

        # load face files form data directory
        self.CONFIG_FOLDER = config["GeneralManager"]["config_folder"]
        self.CURRENT_SCENARIO = config["GeneralManager"]["current_scenario"]


        rospy.loginfo("Param: config_folder:" + str(self.CONFIG_FOLDER))
        rospy.loginfo("Param: current_scenario:" + str(self.CURRENT_SCENARIO))
        rospy.loginfo('configure ok')
        scenario_config = None
        try:
            scenario_config = self.loadConfig()
        except Exception as e:
            rospy.logwarn("Unable to load config file: %s" % e)

        # self._scenarioMap = dict(COCKTAIL_PARTY=CocktailPartyScenario,
        #                          TEST_NAVIG=TestNavigScenario,
        #                          DOOR_OPENED_NAVIG=DoorOpenAndNavigScenario,
        #                          TEST_DIALOGUE=TestDialogueScenario,
        #                          TEST_COCKTAIL_PARTY_V1=TestCocktailPartyV1Scenario,
        #                          TEST_COCKTAIL_PARTY_V2=TestCocktailPartyV2Scenario,
        #                          TEST_COCKTAIL_PARTY_V3=TestCocktailPartyV3Scenario,
        #                          TEST_HOOMANO=TestHoomanoScenario,
        #                          SPRV1=SPRV1Scenario,
        #                          SPRV2=SPRV2Scenario,
        #                          HELP_ME_CARRY=HelpMeCarryV1Scenario,
        #                          INSPECTION=InspectionScenario,
        #                          GPRSV1=GPRSV1Scenario,
        #                          GPRSV2CPE=GPRSV2CPE1Scenario,
        #                          GARBAGE_2019_CPE=TakeOutTheGarbage2019Scenario,
        #                          GARBAGE_2019V2_CPE=TakeOutTheGarbage2019v2Scenario,
        #                          RECEPTIONIST_2019_CPE=Receptionist2019CPEScenario,
        #                          TEST_FACE_LEARN_AND_FIND_POINT=TestFaceLearnAndFindPoint,
        #                          INSPECTION_2019=Inspection2019Scenario)
        self._scenarioMap = dict(RECEPTIONIST_2020_CPE=Receptionist2020CPEScenario)

        try:
            self._currentScenario = self._scenarioMap[self.CURRENT_SCENARIO](scenario_config)
            self._currentScenario.initScenario()
        except Exception as e:
            rospy.logerr("Unable Load SCENARIO Object: %s" % e)

        # FOR DEBUG :
        self.execute()
            

    def execute(self):
        if self._currentScenario == None:
            rospy.logwarn("Current Scenario is not ready (maybe not currently loaded), unable to start scenario...")
            return
        else:
            self._currentScenario.startScenario()
            rospy.loginfo("GM : WAITING FOR NEW SCENARIO")
            self.enable_listening_scenario=True
            
            # if self._currentScenario._configurationReady:
            #     self._currentScenario.startScenario()
            # else:
            #     rospy.logwarn("Current Scenario is not ready, wait minutes and try again...")

    #######################################################################
    #######                LOAD Scenario config.                     ######
    #######################################################################
    def loadConfig(self):
        # scenarioFile = self.CONFIG_FOLDER + "/" + self.CURRENT_SCENARIO + "_SCENARIO.json"
        scenarioFile = self.CONFIG_FOLDER + "/scenario.json"
        rospy.loginfo("Opening scenario configuration file: %s" % scenarioFile)
        with open(scenarioFile) as json_data:
            jsonContent = yaml.safe_load(json_data)
            rospy.loginfo("Scenario configuration file content: %s" % jsonContent)
            # jsonString=json.load(jsonContent)
            # jsonObject = json.load(jsonString, object_hook=lambda d: namedtuple('X', d.keys())(*d.values()))
            ##rospy.loginfo("Scenario configuraiton Object content: %s" % str(jsonObject))
            rospy.loginfo('name: ' + jsonContent['name'])
            rospy.loginfo('description: ' + jsonContent['description'])
            return jsonContent

    def gmStartCallback(self, msg):
        if self._current_status == self.START_STATUS:
            rospy.loginfo('SCENARIO IS ALREADY STARTED...')
            return
        rospy.loginfo('START CMD: ' + msg.data)
        if msg.data == self.START_STATUS:
            self._current_status = self.START_STATUS
            self.execute()
            self._current_status = self.PENDING_STATUS
            rospy.loginfo('-----------------------------------END CURRENT SCENARIO-----------------------')


if __name__ == '__main__':
    import os

    dirname = os.path.dirname(__file__)
    filepath = os.path.join(dirname, '../config/common_gm.yaml')

    with open(filepath) as data:
        gm_config = yaml.safe_load(data)
    
    
    gm = GeneralManager(gm_config)
    while not rospy.is_shutdown():
        rospy.spin()

