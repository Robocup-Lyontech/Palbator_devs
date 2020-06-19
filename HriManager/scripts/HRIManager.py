#!/usr/bin/env python
from socketIO_client import SocketIO, LoggingNamespace
from python_depend.views import Views
import json as js

from std_msgs.msg import String, Bool
import rospy
import os 
import actionlib
from HriManager.msg import GmToHriAction, GmToHriFeedback, GmToHriResult
import time
from copy import deepcopy
from rospy.exceptions import ROSException, ROSInterruptException


from python_depend.views import Views
from speechToTextPalbator.msg import SpeechRecognitionAction, SpeechRecognitionGoal

import ttsMimic.msg

import random

class HRIManager:

  def __init__(self):
    """
        Initialize the HRI Manager
    """
    rospy.init_node("ROS_HRI_node",anonymous=True)

    self.setup_config_yaml()
    self.init_socket_listeners()

    self.view_launcher=Views(self.socketIO)

    rospy.on_shutdown(self.shutdown)


    self.init_variables()

    self.pub = rospy.Publisher("/gm_start",String,queue_size=1)

    self.init_connection()
    

    rospy.loginfo('{class_name} : HRI MANAGER LAUNCHED. WAITING FOR A TABLET REFRESH'.format(class_name=self.__class__.__name__))


    self.pub_test_restart = rospy.Publisher("/test_HRI_restart",Bool,queue_size=1)
    #### TEST FOR NEW MENU #####
    self.load_tablet_menu()

    ######

  def load_tablet_menu(self):
    
    step = {
      "name": "MainMenu",
      "order": 0,
      "eta": 0,
      "speech": {
        "said": "Please choose a scenario in the list",
        "title": "Please choose a scenario in the list"
      },
      "arguments": {
        "scenario_list": self.scenario_list
      },
      "action": "mainMenuPalbator",
      "id": "main-menu"
    }

    while self.dataToUse != 'TABLET_ON' and not rospy.is_shutdown():
      self.socketIO.wait(seconds=0.1)
    
    self.event_touch = False
    rospy.loginfo("TABLET CONNECTED AND READY")
    rospy.loginfo("LOADING MENU ON TABLET ...")

    # json_mic ={
    #   "hide": True
    # }
    # self.socketIO.emit("hideMic",json_mic,broadcast=True)

    if step['order'] == 0:
      self.loaded_steps = step['order']
    else:
      self.loaded_steps = self.loaded_steps + 1

    self.index=step['order']
    self.view_launcher.start(step['action'],step, step['order'], self.dataToUse)

    rospy.loginfo("MENU LOADED")
    while self.event_touch == False and not rospy.is_shutdown():
      self.socketIO.wait(seconds=0.1)

    self.event_touch = False

    rospy.loginfo("{class_name} : CHOOSEN SCENARIO : %s".format(class_name=self.__class__.__name__),self.dataToUse)
    if "CPE" in self.dataToUse:
      self.pub.publish(str(self.dataToUse))
    
    # json_mic ={
    #   "hide": False
    # }
    # self.socketIO.emit("hideMic",json_mic,broadcast=True)
   

  def init_connection(self):
    self.connection_ON=None
    self.action_GM_TO_HRI_server.start()
    self.enable_changing_connection=True
    self.sub_connection_state = rospy.Subscriber(rospy.get_param('~topic_connection_state'),String,self.handle_change_connection_state)
    rospy.loginfo("{class_name} : Waiting for connection state message ...".format(class_name=self.__class__.__name__))

    try:
      rospy.wait_for_message(rospy.get_param('~topic_connection_state'),String,3)
      rospy.loginfo("Got a connection state : %s",str(self.connection_ON))
    except (ROSException, ROSInterruptException) as e:
        rospy.logwarn("Unable to get connection state message. The system will assume a disconnected state")
        self.connection_ON = False

  def reset_for_restart(self):
    self.init_variables()
    self.load_tablet_menu()


  def init_variables(self):
    self.nameToUse=[]
    self.drinkToUse=[]
    self.ageToUse = []
    self.choosenName = None
    self.choosenDrink = None
    self.choosenAge = None
    self.choosenRoom = None
    self.nameAction = None
    self.currentStep = None
    self.currentAction = None
    self.index=0
    self.indexStepCompleted=0
    self.dataToUse=''
    self.data_received=False
    self.currentIndexDataReceivedJS=0
    self.end_guest_procedure=None
    self.json_for_GM=None
    self.event_detected_flag=False

    self.stepsList=None
    self.enable_choice_scenario=True
    self.event_touch = False

    self.loaded_steps = None
  
  def shutdown(self):
    rospy.logwarn("{class_name} : HRI Manager shutting down ...".format(class_name=self.__class__.__name__))
    self.socketIO.disconnect()


  def setup_config_yaml(self):
    """
        Load the configuration parameters from YAML.
    """
    self.enable_vocal_detection = rospy.get_param("~enable_STT")

    _socketIO_ip_param = "~socketIO_IP"
    _socketIO_port_param = "~socketIO_port"
    _topic_choice_scenario = rospy.get_param("~topic_choice_scenario")
    _stt_online_server_name = rospy.get_param("~stt_online")
    _stt_offline_server_name = rospy.get_param("~stt_offline")

    _stt_server_name = rospy.get_param("~stt_server")

    _tts_mimic_server_name = rospy.get_param("~tts_mimic")
    _action_server_hri_name = rospy.get_param("~action_server_hri")
    _switch_connection_timeout = rospy.get_param("~switch_connection_timeout")

    _list_scenario_available = rospy.get_param("~list_scenario_available")

    socketIP = None
    socketPort = None
    if rospy.has_param(_socketIO_ip_param):
      socketIP = rospy.get_param(_socketIO_ip_param)
    else:
      rospy.logerr("{class_name} : No IP specified for socketIO. Couldn't open the socket communication.".format(class_name=self.__class__.__name__))
      return

    if rospy.has_param(_socketIO_port_param):
      socketPort = rospy.get_param(_socketIO_port_param)
    else:
      rospy.logerr("{class_name} : No port specified for socketIO. Couldn't open the socket communication".format(class_name=self.__class__.__name__))
      return

    self.socketIO = SocketIO(socketIP, socketPort, LoggingNamespace)

    self.pub_choice_scenario=rospy.Publisher(_topic_choice_scenario,String,queue_size=1)

    # self.action_online_client = actionlib.SimpleActionClient(_stt_online_server_name,speechToTextPalbator.msg.SttOnlineAction)
    # self.action_offline_client = actionlib.SimpleActionClient(_stt_offline_server_name,speechToTextPalbator.msg.SttOfflineAction)

    self.action_stt_client = actionlib.SimpleActionClient(_stt_server_name,SpeechRecognitionAction)


    self.client_TTS=actionlib.SimpleActionClient(_tts_mimic_server_name,ttsMimic.msg.TtsMimicAction)
    self.action_GM_TO_HRI_server=actionlib.SimpleActionServer(_action_server_hri_name,GmToHriAction,self.action_GmToHri_callback,auto_start=False)
    self.action_GM_TO_HRI_feedback=GmToHriFeedback()
    self.action_GM_TO_HRI_result=GmToHriResult()
    if self.enable_vocal_detection == True:
      # rospy.loginfo("{class_name} : Waiting for STT online server...".format(class_name=self.__class__.__name__))
      # self.action_online_client.wait_for_server()
      # rospy.loginfo("{class_name} : Connected to STT online server".format(class_name=self.__class__.__name__))

      # rospy.loginfo("{class_name} : Waiting for STT offline server...".format(class_name=self.__class__.__name__))
      # self.action_offline_client.wait_for_server()
      # rospy.loginfo("{class_name} : Connected to STT offline server".format(class_name=self.__class__.__name__))
      rospy.loginfo("{class_name} : Waiting for STT server...".format(class_name=self.__class__.__name__))
      self.action_stt_client.wait_for_server()
      rospy.loginfo("{class_name} : Connected to STT server".format(class_name=self.__class__.__name__))

    rospy.loginfo("{class_name} : Waiting for TTS server...".format(class_name=self.__class__.__name__))
    self.client_TTS.wait_for_server()
    rospy.loginfo("{class_name} : TTS server connected".format(class_name=self.__class__.__name__))

    self.switch_timeout = _switch_connection_timeout

    self.scenario_list = _list_scenario_available

  def init_socket_listeners(self):
    """
        Initialize all the event listeners for the socketIO
    """
    #callback quand user choisit scenario sur tablette
    self.socketIO.on(rospy.get_param('~socket_choose_scenario'),self.chooseScenario)
    #callback reponse quand scenario choisit et charge
    self.socketIO.on(rospy.get_param('~socket_scenario_loaded'),self.scenarioCharged)
    #callback quand on recoit des donnees de la tablette
    self.socketIO.on(rospy.get_param('~socket_index_view_received'), self.indexDataJSstepDone)
    self.socketIO.on(rospy.get_param('~socket_data_received'), self.dataJSstepDone)
    #recuperation de l'order de la view chargee par la tablette
    self.socketIO.on(rospy.get_param('~socket_view_loaded'), self.send_gm_view_launched)
    # self.socketIO.on(rospy.get_param('~socket_hri_reset'), self.restart_hri)

  def handle_change_connection_state(self,req):
    """
        Callback function when a connection state change is detected
    """
    # if self.enable_changing_connection==True:
    if req.data=='Connected':
      # self.action_offline_client.cancel_all_goals()
      # self.action_stt_client.cancel_all_goals()
      self.connection_ON=True
        
    elif req.data=='Disconnected':
      # self.action_online_client.cancel_all_goals()
      self.connection_ON=False


  def tts_action(self,speech):
    """
        Send the speech to say to the TTS ActionServer

        :param speech: text to say
        :type speech: string
    """
    self.goal_TTS=ttsMimic.msg.TtsMimicGoal(speech)
    self.client_TTS.send_goal(self.goal_TTS)
    self.client_TTS.wait_for_result()


  def parser_scenario_step(self,scenario,goal):
    """
        Decide how to launch a step according to its action and its name.

        If new views or new scenarios are added, add them in the parser below.

        self.dynamic_view is for views which require additional data or an event

        self.static_view is for views which don't require any data or event

        self.load_multiple_views is to launch a sequence of views managed by HRI only

        :param scenario: scenario name
        :type scenario: string
        :param goal: Step informations
        :type goal: dict
    """
    which_step_action=deepcopy(self.stepsList[goal['stepIndex']]['action'])
    which_step_name=deepcopy(self.stepsList[goal['stepIndex']]['name'])

    if scenario == 'Receptionist':
        ##### rajouter des conditions si nouvelles actions importantes
      if which_step_action != '':
        if which_step_action=='askOpenDoor':
          self.dynamic_view(goal['stepIndex'],None,wait_for_event=True)

        elif which_step_action == 'mainMenuPalbator':
          self.dynamic_view(goal['stepIndex'],None,wait_for_event=True)


        elif which_step_action == 'lookForKnownGuest':
          self.dynamic_view(goal['stepIndex'],goal['data'])

        elif which_step_action=='presentPerson':
          self.dynamic_view(goal['stepIndex'],goal['data'])

        elif which_step_action=='seatGuest':
          self.dynamic_view(goal['stepIndex'],goal['data'])

        elif which_step_action=="pointTo":
          self.dynamic_view(goal['stepIndex'],goal['data'])

        elif which_step_action=='askToFollow':
          self.dynamic_view(goal['stepIndex'],goal['data'])

        elif which_step_action == 'goTo':
          self.dynamic_view(goal['stepIndex'],goal['data'])

        else:
          self.static_view(goal['stepIndex'])
      
      else:
        if "Ask infos" in which_step_name:
          self.load_multiple_views(goal['stepIndex'],procedure_type='guestInfos')

        else:
          self.static_view(goal['stepIndex'])

    elif scenario == 'Clean_up':
      if which_step_action != '':
        
        if which_step_action == 'goTo':
          self.dynamic_view(goal['stepIndex'],goal['data'])

        elif which_step_action == 'findObject':
          self.dynamic_view(goal['stepIndex'],goal['data'])

        elif which_step_action == 'objectAction':
          self.dynamic_view(goal['stepIndex'],goal['data'])


        else:
          self.static_view(goal['stepIndex'])

      else:
        if 'Ask room' in which_step_name:
          self.load_multiple_views(goal['stepIndex'],procedure_type='chooseRoom')
        else:
          self.static_view(goal['stepIndex'])

  def action_GmToHri_callback(self,goal):
    """
        Callback function for ActionServer HRI. Receive a goal and load the correct view according its step index.

        :param goal: A json containing parameters in order to load a specific view 
        :type goal: GmToHriGoal
    """
    rospy.loginfo("{class_name} : Action initiating ...".format(class_name=self.__class__.__name__))
    success=True
    json_output=None
    self.action_GM_TO_HRI_feedback.Gm_To_Hri_feedback=''
    json_goal=js.loads(goal.json_request)
    if json_goal['action'] == 'RESTART':
      # self.restart_hri()
      self.json_for_GM={
        "result": "Restart done"
      }

    if json_goal['action'] == "stepsList":
      self.scenario_loaded=False
      rospy.loginfo("{class_name} : Getting steps list...".format(class_name=self.__class__.__name__))
      self.stepsList=json_goal['stepsList']
      self.choosen_scenario=json_goal['scenario']
      self.json_for_GM={
        "result": "Steps received"
      }

    elif json_goal['action'] == "currentStep":
      if json_goal['stepIndex'] == 0:
        scenario_for_tablet = self.choosen_scenario.replace("_"," ")
        scenario_for_tablet = scenario_for_tablet.title()
        json_charge_scenario={
          'scenario': scenario_for_tablet
        }
        self.chargeScenario(json_charge_scenario)
        
      self.parser_scenario_step(self.choosen_scenario,json_goal)


    json_output=self.json_for_GM
    if not json_output is None:
      if 'result' in json_output and json_output['result']=='PREEMPTED':
        success=False

    self.action_GM_TO_HRI_server.publish_feedback(self.action_GM_TO_HRI_feedback)
    if success:
      self.action_GM_TO_HRI_result.Gm_To_Hri_output=js.dumps(json_output)
      rospy.loginfo("{class_name} : Action GM TO HRI succeeded".format(class_name=self.__class__.__name__))
      
      self.action_GM_TO_HRI_server.set_succeeded(self.action_GM_TO_HRI_result)


  def load_step_without_action(self):
    """
        Load a view which doesn't have any action name.
    """

    # rospy.logwarn("LOADED STEPS %s",str(self.loaded_steps))
    # rospy.logwarn("CURRENT INDEX %s",str(self.index))


    indexToSend = deepcopy(self.index)
    if indexToSend != self.loaded_steps:
      indexToSend = self.loaded_steps
      self.currentStep['order'] = self.loaded_steps

    if indexToSend != 1:
      stepCompletedJson = {"idSteps": self.indexStepCompleted}
      self.socketIO.emit(rospy.get_param("~socket_emit_step_complete"),stepCompletedJson,broadcast=True)
      rospy.loginfo("{class_name} : ETAPE TERMINEE: ".format(class_name=self.__class__.__name__)+str(self.currentStep['name']))
      self.indexStepCompleted = indexToSend
    else:
      self.indexStepCompleted = indexToSend

    rospy.loginfo("{class_name} : ETAPE COURANTE A DEMARRER: ".format(class_name=self.__class__.__name__)+self.currentStep['name'])
    dataJsonToSendCurrentStep = {
        "index": indexToSend,
        "step":self.currentStep
    }
    self.socketIO.emit(rospy.get_param("~socket_emit_current_step"),dataJsonToSendCurrentStep,broadcast=True)
    if self.currentStep['id'] != 'FinishScenario':
      dataJsonToSendTimer = {
          "state": 'TOGGLE_TIMER/ON'
      }
      self.socketIO.emit(rospy.get_param("~socket_emit_start_timer"),dataJsonToSendTimer, broadcast=True)
    else:
      dataJsonToSendTimer = {
          "state": 'TOGGLE_TIMER/OFF'
      }
      self.socketIO.emit(rospy.get_param("~socket_emit_start_timer"),dataJsonToSendTimer, broadcast=True)
    rospy.loginfo("{class_name} : ETAPE DEMARREE: ".format(class_name=self.__class__.__name__)+self.currentStep['name'])

      

  def load_view_with_action(self):
    """
        Load a view with an action name.
    """


    # rospy.logwarn("CURRENT LOADED STEPS %s",str(self.loaded_steps))
    # rospy.logwarn("CURRENT STEP ORDER %s",str(self.currentStep['order']))

    if self.loaded_steps != self.currentStep['order']:
      self.currentStep['order'] = self.loaded_steps

    self.view_launcher.start(self.currentStep['action'],self.currentStep, self.currentStep['order'], self.dataToUse)
    rospy.loginfo("{class_name} : CHARGEMENT VUE SOUS ETAPE:".format(class_name=self.__class__.__name__)+self.currentStep['name'] +" SUR TABLETTE")

    speech=deepcopy(self.currentStep['speech']['said'])
    self.tts_action(speech)
    

  def static_view(self,stepIndex):
    """
        Load a static view (a view which doesn't wait for an event)

        :param stepIndex: the index of the step to load
        :type stepIndex: int
    """
    rospy.logwarn("{class_name} : STATIC VIEW : ".format(class_name=self.__class__.__name__)+str(stepIndex))

    # if stepIndex == 0:
    #   self.loaded_steps = stepIndex
    # else:
    #   self.loaded_steps = self.loaded_steps + 1

    if self.loaded_steps is None:
      self.loaded_steps = stepIndex
    else:
      self.loaded_steps = self.loaded_steps + 1

    rospy.logwarn("LOADED STEPS "+str(self.loaded_steps))

    self.currentStep=deepcopy(self.stepsList[stepIndex])
    self.index=deepcopy(self.currentStep['order'])
    self.currentAction=deepcopy(self.currentStep['action'])
    

    if self.currentAction == '':
      self.load_step_without_action()
    else:
      self.load_view_with_action()

    if self.currentStep['name'] != 'Finish Scenario':
      self.json_for_GM={
          "indexStep": self.index-1,
          "actionName": '',
          "NextToDo": "next",
          "NextIndex": self.index
      }
    else:
      self.json_for_GM={
          "indexStep": self.index,
          "actionName": '',
          "NextToDo": "next",
          "NextIndex": ""
      }
    self.event_detected_flag=True
    self.socketIO.wait(seconds=0.1)

  def dynamic_view(self,stepIndex,data=None,wait_for_event=False,in_procedure=False):
    """
        Load a dynamic view (a view which will wait for an event or a view which needs additional data to be loaded)

        :param stepIndex: the index of the step to load
        :type stepIndex: int
        :param data: Additional data could be sent by GeneralManager to be used in the view
        :type data: dict
        :param wait_for_event: Flag to decide if the view will wait for an event or not
        :type wait_for_event: Boolean
        :param in_procedure: Flag to know if the view is part of a procedure (sequence of views managed by HRI) or not
        :type in_procedure: Boolean
    """
    rospy.logwarn("{class_name} : DYNAMIC VIEW : ".format(class_name=self.__class__.__name__)+str(stepIndex))

    # if stepIndex == 0:
    #   self.loaded_steps = stepIndex
    # else:
    #   self.loaded_steps = self.loaded_steps + 1
    if self.loaded_steps is None:
      self.loaded_steps = stepIndex
    else:
      self.loaded_steps = self.loaded_steps + 1
    
    rospy.logwarn("LOADED STEPS "+str(self.loaded_steps))

    self.currentStep=deepcopy(self.stepsList[stepIndex])
    self.index=deepcopy(self.currentStep['order'])
    self.currentAction=deepcopy(self.currentStep['action'])

    if wait_for_event == False:

      if self.currentAction == "askToFollow":
        key=self.currentStep['arguments']['key']
        self.currentStep['speech']['said']=self.currentStep['speech']['said'].replace(key+"_name",data[key]['name'])
        self.currentStep['speech']['said']=self.currentStep['speech']['said'].replace(key+"_drink",data[key]['drink'])
        self.currentStep['speech']['said']=self.currentStep['speech']['said'].replace(key+"_age",str(data[key]['age']))
        self.currentStep['arguments']['who']['name'] = data[key]['name']
        self.currentStep['arguments']['who']['drinkObj']['name'] = data[key]['drink']
        self.currentStep['arguments']['who']['drinkObj']['pathOnTablet'] = data[key]['pathOnTablet']
        self.currentStep['arguments']['who']['guestPhotoPath'] = data[key]['guestPhotoPath']
        self.currentStep['arguments']['who']['age'] = str(data[key]['age'])

      elif self.currentAction == "lookForKnownGuest":
        key=self.currentStep['arguments']['key']
        self.currentStep['speech']['said']=self.currentStep['speech']['said'].replace(key+"_name",data[key]['name'])
        self.currentStep['speech']['title']=self.currentStep['speech']['title'].replace(key+"_name",data[key]['name'])
        self.currentStep['arguments']['who']['name'] = data[key]['name']
        self.currentStep['arguments']['who']['guestPhotoPath'] = data[key]['guestPhotoPath']

      elif self.currentAction == "presentPerson":
        speech=self.currentStep['speech']['said']
        number_know_guests=len(self.currentStep['arguments']['to'])

        for key in data.keys():
          for i in range(0,number_know_guests):
            if key in self.currentStep['arguments']['to'][i]['name']:
              self.currentStep['arguments']['to'][i]['name']=data[key]['name']
              self.currentStep['arguments']['to'][i]['guestPhotoPath']=data[key]['guestPhotoPath']
              self.currentStep['arguments']['to'][i]['age']=str(data[key]['age'])
              self.currentStep['arguments']['to'][i]['drink']['name']=data[key]['drink']
              self.currentStep['arguments']['to'][i]['drink']['pathOnTablet']=data[key]['pathOnTablet']

              rospy.logwarn("{class_name} : KEY ".format(class_name=self.__class__.__name__)+str(key)+" DRINK "+str(data[key]['drink'])+" DRINKPATH "+str(data[key]['pathOnTablet']))

          if key in self.currentStep['arguments']['who']['name']:
            self.currentStep['arguments']['who']['name']=data[key]['name']
            self.currentStep['arguments']['who']['guestPhotoPath']=data[key]['guestPhotoPath']
            self.currentStep['arguments']['who']['age']=str(data[key]['age'])
            self.currentStep['arguments']['who']['drinkObj']['name']=data[key]['drink']
            self.currentStep['arguments']['who']['drinkObj']['pathOnTablet']=data[key]['pathOnTablet']
            rospy.logwarn("{class_name} : KEY ".format(class_name=self.__class__.__name__)+str(key)+" DRINK "+str(data[key]['drink'])+" DRINKPATH "+str(data[key]['pathOnTablet']))

          speech = speech.replace(key+'_name',data[key]['name'])
          speech = speech.replace(key+'_drink',data[key]['drink'])
          speech = speech.replace(key+'_age',str(data[key]['age']))
        self.currentStep['speech']['said']=speech
      
      elif self.currentAction == "seatGuest":
        for key in data.keys():
          if key in self.currentStep['arguments']['who']['name']:
            self.currentStep['arguments']['who']['name']=data[key]['name']
            self.currentStep['arguments']['who']['guestPhotoPath']=data[key]['guestPhotoPath']
            self.currentStep['speech']['said']=self.currentStep['speech']['said'].replace(key+"_name",data[key]['name'])
            self.currentStep['speech']['title']=self.currentStep['speech']['title'].replace(key+"_name",data[key]['name'])

      elif self.currentAction == "pointTo":
        object_to_point=deepcopy(self.currentStep['arguments']['what'])
        if object_to_point=='chair':
          pass
        elif object_to_point=='human':
          for key in data.keys():
            if key in self.currentStep['arguments']['who']['name']:
              self.currentStep['arguments']['who']['name']=data[key]['name']
              self.currentStep['arguments']['who']['guestPhotoPath']=data[key]['guestPhotoPath']
              self.currentStep['speech']['said']=self.currentStep['speech']['said'].replace(key+"_name",data[key]['name'])
              self.currentStep['speech']['title']=self.currentStep['speech']['title'].replace(key+"_name",data[key]['name'])

      elif self.currentAction == 'goTo':
        if self.choosen_scenario == "Receptionist":
          location_name = self.currentStep['arguments']['where']
          for item in data:
            if item['name'] == location_name:
              self.currentStep['speech']['said'] = self.currentStep['speech']['said'].replace("location_name",location_name)
              self.currentStep['speech']['title'] = self.currentStep['speech']['title'].replace("location_name",location_name)
              self.currentStep['arguments']['location']['pathOnTablet'] = item['pathOnTablet']
              self.currentStep['arguments']['location']['name'] = item['name']

        elif self.choosen_scenario == "Clean_up":
          key = self.currentStep['arguments']['where']
          self.currentStep['speech']['said'] = self.currentStep['speech']['said'].replace(key+"_name",data[key]['name'])
          self.currentStep['speech']['title'] = self.currentStep['speech']['title'].replace(key+"_name",data[key]['name'])
          self.currentStep['arguments']['location']['name'] = self.currentStep['arguments']['location']['name'].replace(key+"_name",data[key]['name'])
          self.currentStep['arguments']['location']['pathOnTablet'] = self.currentStep['arguments']['location']['pathOnTablet'].replace(key+"_pathOnTablet",data[key]['pathOnTablet'])

      elif self.currentAction == 'findObject':
        key = self.currentStep['arguments']['what']
        self.currentStep['speech']['said'] = self.currentStep['speech']['said'].replace(key+"_name",data[key]['name'])
        self.currentStep['speech']['title'] = self.currentStep['speech']['title'].replace(key+"_name",data[key]['name'])
        self.currentStep['arguments']['location']['name'] = self.currentStep['arguments']['location']['name'].replace(key+"_name",data[key]['name'])
        self.currentStep['arguments']['location']['pathOnTablet'] = self.currentStep['arguments']['location']['pathOnTablet'].replace(key+"_pathOnTablet",data[key]['pathOnTablet'])

      elif self.currentAction == 'objectAction':
        # rospy.logerr("DATA : %s",str(data))
        for key in data.keys():
          self.currentStep['speech']['said'] = self.currentStep['speech']['said'].replace(key+"_name",data[key]['name'])
          self.currentStep['speech']['title'] = self.currentStep['speech']['title'].replace(key+"_name",data[key]['name'])
          self.currentStep['arguments']['object']['name'] = self.currentStep['arguments']['object']['name'].replace(key+"_name",data[key]['name'])
          self.currentStep['arguments']['object']['pathOnTablet'] = self.currentStep['arguments']['object']['pathOnTablet'].replace(key+"_pathOnTablet",data[key]['pathOnTablet'])

        rospy.logwarn("------------------------")
        rospy.logwarn("{class_name} : ".format(class_name=self.__class__.__name__)+str(self.currentStep))
        rospy.logwarn("------------------------")



      self.load_view_with_action()

      if self.currentAction == 'findObject':
        self.json_for_GM={
          "indexStep": self.index-1,
          "actionName": self.currentAction,
          "scenario": self.choosen_scenario,
          "objectKey": self.currentStep['arguments']['objectKey'],
          "NextToDo": "next",
          "NextIndex": self.index
        }
      else:
        if self.currentAction == 'objectAction' and 'store' in self.currentStep['id']:
          self.json_for_GM={
            "indexStep": self.index-1,
            "actionName": self.currentAction,
            "scenario": self.choosen_scenario,
            "NextToDo": "next",
            "NextIndex": self.index+1
          }
        elif self.currentAction == 'goTo':
          self.json_for_GM={
            "indexStep": self.index-1,
            "actionName": self.currentAction,
            "scenario": self.choosen_scenario,
            "destination": self.currentStep['arguments']['location']['name'],
            "NextToDo": "next",
            "NextIndex": self.index
          }
        else:
          self.json_for_GM={
            "indexStep": self.index-1,
            "actionName": self.currentAction,
            "scenario": self.choosen_scenario,
            "NextToDo": "next",
            "NextIndex": self.index
          }
      self.event_detected_flag=True
      self.socketIO.wait(seconds=0.1)

    else:
      speech = deepcopy(self.currentStep['speech']['said'])
      title = deepcopy(self.currentStep['speech']['title'])

      if self.currentAction == 'confirm':
        if self.currentStep['name'] == 'Confirm room':
          speech=speech.replace("location_name",self.choosenRoom)
          title=title.replace("location_name",self.choosenRoom)

        elif "drink" in self.currentStep['name']:
          speech=speech.format(drink=str(self.choosenDrink))
          title=title.format(drink=str(self.choosenDrink))

        elif "name" in self.currentStep['name']:
          speech=speech.format(name=self.choosenName)
          title=title.format(name=self.choosenName)
      else:
        if "drink" in self.currentStep['name']:
          speech=speech.format(name=str(self.nameToUse[-1]))

      self.currentStep['speech']['said'] = speech
      self.currentStep['speech']['title'] = title
      
      self.load_view_with_action()
      self.event_touch = False

      if self.enable_vocal_detection == True:
        # if self.connection_ON==True:
        #   self.routine_online()

        # elif self.connection_ON==False:
        #   self.routine_offline()
        for i in range(0,5):
          if i == 1:
            self.tts_action("Could you repeat please ?")
          elif i == 2:
            self.tts_action("Say it again please?")
          elif i == 3:
            self.tts_action("Sorry I missed that. Could you repeat please ?")
          elif i == 4:
            self.tts_action("I couldn't understand. Please try again.")

          rospy.loginfo("{class_name} : intent %s".format(class_name=self.__class__.__name__),str(i))
          data_STT = self.vocal_detection()
          if not data_STT is None:
            if data_STT == 'EVENT_REQUEST':
              break
            else:
              self.dataToUse = data_STT
              break
        
        if data_STT is None:
          self.tts_action("I am currently not able to understand you. Please click on a button to continue the scenario")
          while self.event_touch == False:
            self.socketIO.wait(0.1)
          self.event_touch = False

      else:
        while self.event_touch == False:
          self.socketIO.wait(0.1)
        self.event_touch = False

      if in_procedure == False:
        self.json_for_GM={
                        "indexStep": self.index-1,
                        "actionName": self.currentAction,
                        "dataToUse": self.dataToUse,
                        "NextToDo": "next",
                        "NextIndex": self.index
        }


  def load_multiple_views(self,indexStep,procedure_type):
    """
        A sequence of views is loaded by HRI without GeneralManager's intervention.

        :param indexStep: the index of the step to load
        :type indexStep: int
        :param procedure_type: name of the loaded sequence 
        :type procedure_type: string 
    """
    
    rospy.logwarn("{class_name} : STARTING PROCEDURE VIEW : ".format(class_name=self.__class__.__name__)+procedure_type)

    end_procedure = False
    index_procedure=indexStep

    while end_procedure == False:
      self.currentStep=deepcopy(self.stepsList[index_procedure])
      self.index=deepcopy(self.currentStep['order'])
      self.currentAction=deepcopy(self.currentStep['action'])
      rospy.loginfo("{class_name} : CURRENT ACTION ".format(class_name=self.__class__.__name__)+str(self.currentAction))
      if self.currentAction == '':

        if index_procedure == 0:
          self.loaded_steps = index_procedure
        else:
          self.loaded_steps = self.loaded_steps + 1
        
        rospy.logwarn("STEPS LOADED "+str(self.loaded_steps))

        self.load_step_without_action()
        index_procedure=index_procedure+1
        rospy.logwarn("{class_name} : NEXT PROCEDURE INDEX ".format(class_name=self.__class__.__name__)+str(index_procedure))
      else:
        self.dynamic_view(index_procedure,data=None,wait_for_event=True,in_procedure=True)

        if self.currentAction == 'confirm':
          if self.dataToUse=='false' or self.dataToUse=='NO':
              rospy.loginfo("{class_name} : RECEIVED DATA -> FALSE".format(class_name=self.__class__.__name__))
              if "name" in self.currentAction:
                self.choosenName=''
              elif "drink" in self.currentAction:
                self.choosenDrink=''
              index_procedure=index_procedure-1
              self.loaded_steps = self.loaded_steps-2
          elif self.dataToUse=='true' or self.dataToUse=='YES':
            rospy.loginfo("{class_name} : RECEIVED DATA -> TRUE".format(class_name=self.__class__.__name__))
            if "name" in self.currentStep['name']:
              self.nameToUse.append(self.choosenName)
            elif "drink" in self.currentStep['name']:
              self.drinkToUse.append(self.choosenDrink)
            index_procedure=index_procedure+1
            if procedure_type == 'chooseRoom':
              end_procedure=True
        
        else:
          if self.currentAction == 'askRoom':
            self.choosenRoom = self.dataToUse

          elif self.currentAction == 'askName':
            self.choosenName = self.dataToUse
          
          elif self.currentAction == 'askDrink':
            self.choosenDrink = self.dataToUse
          
          elif self.currentAction == 'askAge':
            self.choosenAge = self.dataToUse
            end_procedure=True
          
          index_procedure=index_procedure+1

    if procedure_type == 'chooseRoom':
      self.json_for_GM={
        "indexStep": self.index-1,
        "actionName": self.currentAction,
        "dataToUse": self.dataToUse,
        "NextToDo": "next",
        "NextIndex": self.index,
        "saveData":{
          "action": "storeRoom",
          "what": self.currentStep['arguments']['what'],
          "where": self.choosenRoom
        }
      }
    elif procedure_type == 'guestInfos':
      self.json_for_GM={
        "indexStep": self.index-1,
        "actionName": self.currentAction,
        "dataToUse": self.dataToUse,
        "NextToDo": "next",
        "NextIndex": self.index,
        "saveData":{
          "who": self.currentStep['arguments']['who'],
          "name": self.nameToUse[-1],
          "drink": self.drinkToUse[-1],
          "age": self.choosenAge
        }
      }
    rospy.loginfo("{class_name} : END PROCEDURE ".format(class_name=self.__class__.__name__)+str(procedure_type))

  
  def vocal_detection(self):
    rospy.loginfo("{class_name} : ----------- DEBUT DETECTION VOCALE--------------------------------".format(class_name=self.__class__.__name__))

    goal_stt = SpeechRecognitionGoal()
    data_STT = None
    json_goal = {
      "order": self.index,
      "action": self.currentAction,
      "scenario": self.choosen_scenario,
      "connection_state": None
    }

    if self.connection_ON:
      json_goal['connection_state'] = "Online"
    else:
      json_goal['connection_state'] = "Offline"

    goal_stt.order = js.dumps(json_goal)

    self.action_stt_client.send_goal(goal_stt)

    while self.action_stt_client.get_result() is None and not rospy.is_shutdown():
      if self.event_touch:
        self.action_stt_client.cancel_all_goals()
        break
      self.socketIO.wait(seconds=0.1)

    rospy.loginfo("{class_name} : %s".format(class_name=self.__class__.__name__),str(self.action_stt_client.get_result()))

    if not self.action_stt_client.get_result() is None and self.action_stt_client.get_result().stt_result != '':

      json_data_in_str=str(self.action_stt_client.get_result().stt_result)
      json_data = js.loads(json_data_in_str)
      data_STT = json_data['stt_result']
    elif self.event_touch==True:
      data_STT = "EVENT_REQUEST"
      self.event_touch = False

    rospy.loginfo("{class_name} : ----------- FIN DETECTION VOCALE--------------------------------".format(class_name=self.__class__.__name__))

    return data_STT
  
  # def routine_online(self):
  #   """
  #       Function to use the online Speech Detection. If the counter reaches the timeout, the system will switch to offline Speech Detection. 
  #   """
  #   rospy.loginfo("{class_name} : ----------- DEBUT ROUTINE ONLINE--------------------------------".format(class_name=self.__class__.__name__))
  #   self.goal_online = speechToTextPalbator.msg.SttOnlineGoal()
  #   rospy.loginfo("{class_name} : Sending goal to online ...".format(class_name=self.__class__.__name__))
  #   order={
  #       'order': self.index,
  #       'action': self.currentAction,
  #       'scenario': self.choosen_scenario
  #   }
  #   json_in_str=js.dumps(order)
  #   self.goal_online.order=json_in_str
  #   self.action_online_client.send_goal(self.goal_online)
  #   cp=0
  #   while self.action_online_client.get_result() is None and not rospy.is_shutdown():
  #     if cp == self.switch_timeout or self.connection_ON == False:
  #       self.action_online_client.cancel_all_goals()
  #       self.tts_action('Switching to offline mode')
  #       break
  #     elif self.event_touch == True:
  #       self.action_online_client.cancel_all_goals()
  #       break

  #     rospy.loginfo("{class_name} : Waiting for online detect ....".format(class_name=self.__class__.__name__))
  #     cp=cp+1
  #     self.socketIO.wait(seconds=0.1)

  #   rospy.loginfo("{class_name} : "+str(self.action_online_client.get_result()))
  #   if not self.action_online_client.get_result() is None and self.action_online_client.get_result().stt_result != '':
  #     self.dataToUse=str(self.action_online_client.get_result().stt_result)
  #   elif self.event_touch==True:
  #     self.event_touch = False
  #     rospy.logwarn("{class_name} : EVENT TOUCH ".format(class_name=self.__class__.__name__)+str(self.event_touch))
  #   else:
  #     self.enable_changing_connection=False
  #     self.routine_offline()
  #     self.enable_changing_connection=True

  #   rospy.loginfo("{class_name} : ----------- FIN ROUTINE ONLINE--------------------------------".format(class_name=self.__class__.__name__))

  # def routine_offline(self):
  #   """
  #       Function to use the offline Speech Detection. 
  #   """
  #   rospy.loginfo("{class_name} : ----------- DEBUT ROUTINE OFFLINE--------------------------------".format(class_name=self.__class__.__name__))
  #   self.goal_offline = speechToTextPalbator.msg.SttOfflineGoal()
  #   rospy.loginfo("{class_name} : Sending goal to offline ...".format(class_name=self.__class__.__name__))
  #   order={
  #       'order': self.index,
  #       'action': self.currentAction,
  #       'scenario': self.choosen_scenario
  #   }
  #   json_in_str=js.dumps(order)
  #   self.goal_offline.order=json_in_str
  #   self.action_offline_client.send_goal(self.goal_offline)
  #   while self.action_offline_client.get_result() is None and not rospy.is_shutdown():
  #     if self.event_touch == True:
  #       self.action_offline_client.cancel_all_goals()
  #       self.event_touch = False
  #       rospy.logwarn("{class_name} : EVENT TOUCH ".format(class_name=self.__class__.__name__)+str(self.event_touch))
  #       break
  #     rospy.loginfo("{class_name} : Waiting for OFFLINE detect ....".format(class_name=self.__class__.__name__))
  #     self.socketIO.wait(seconds=0.1)
  #   rospy.loginfo("{class_name} : ".format(class_name=self.__class__.__name__)+str(self.action_offline_client.get_result()))
  #   if not self.action_offline_client.get_result() is None:
  #     if str(self.action_offline_client.get_result().stt_result) != '':
  #       self.dataToUse=str(self.action_offline_client.get_result().stt_result)
  #   rospy.loginfo("{class_name} : ----------- FIN ROUTINE OFFLINE--------------------------------".format(class_name=self.__class__.__name__))

  def indexDataJSstepDone(self,json):
    """
        When a view is loaded, the React send back a JSON containing informations about the view. This function gets the index of the last view successfully loaded.

        :param json: JSON with data of last loaded view
        :type json: dict
    """
    if('data' in json):
      self.currentIndexDataReceivedJS = json['data']

##################################### DATA RECEIVED FROM TOUCH MANAGER #################################################
  def dataJSstepDone(self,json):
    """
        This function will get the data sent by React after a view is loaded. If the view was associated to an event, the received data is collected.

        :param json: JSON with data of last loaded view
        :type json: dict
    """
    rospy.logwarn("DATA_RECEIVED")

    self.client_TTS.cancel_all_goals()
    self.event_detected_flag=True
    
    if (self.data_received is False and (self.index == self.currentIndexDataReceivedJS or self.loaded_steps == self.currentIndexDataReceivedJS)):
      self.event_touch = True
      rospy.logwarn("{class_name} : EVENT TOUCH ".format(class_name=self.__class__.__name__)+str(self.event_touch))
      rospy.loginfo("{class_name} : DONNEE RECUE DEPUIS TOUCH MANAGER".format(class_name=self.__class__.__name__))
      # self.action_online_client.cancel_all_goals()
      # self.action_offline_client.cancel_all_goals()
      self.action_stt_client.cancel_all_goals()
      self.data_received=True
      self.dataToUse = json['data']
      rospy.loginfo("{class_name} : DONNEE TOUCH MANAGER: ".format(class_name=self.__class__.__name__)+str(self.dataToUse))

    else:
      rospy.logwarn('{class_name} : Le touch Manager envoie une donnee a la mauvaise etape: '.format(class_name=self.__class__.__name__)+str(self.currentIndexDataReceivedJS)+' au lieu de '+str(self.index))

    self.data_received=False
    
  ###########################

  def scenarioCharged(self,json):
    """
        Callback function when a scenario is successfully loaded on the screen by the React. 

        :param json: JSON with data of the loaded scenario
        :type json: dict 
    """
    rospy.loginfo("{class_name} : HRI : SCENARIO CHARGED".format(class_name=self.__class__.__name__))
    self.json_for_GM=json
    self.scenario_loaded=True

  def chooseScenario(self,json):
    """
        Callback function when a scenario button is clicked on the screen. The scenario choice is sent to HRI by the React.

        :param json: JSON with data of the choosen scenario
        :type json: dict 
    """
    if self.enable_choice_scenario==True:
      self.enable_choice_scenario=False
      rospy.loginfo("{class_name} : choosing scenario...".format(class_name=self.__class__.__name__))
      self.choosen_scenario = json['scenario']
      self.pub_choice_scenario.publish(self.choosen_scenario)


  ########################## Chargement du scenario selectionne ################

  ######### On recoit depuis le REACT le nom du scenario selectionne et on charge le json approprie pour lui renvoyer ############
  def chargeScenario(self,json):
    """
        Callback function to send the data of the scenario to load to the React. 

        :param json: JSON with data of the scenario to load
        :type json: dict 
    """
    self.socketIO.emit(rospy.get_param('~socket_emit_scenario_to_charge'),json, broadcast=True)
  
  def send_gm_view_launched(self,json):
    """
        Callback function when a view is loaded on the screen. Gets the index of the loaded view.

        :param json: JSON with data of the loaded view
        :type json: dict 
    """
    self.json_confirm_View_launch=json
    rospy.loginfo("{class_name} : ".format(class_name=self.__class__.__name__)+str(json['data']))
    rospy.loginfo("{class_name} : Index de la vue lancee sur le Touch : ".format(class_name=self.__class__.__name__)+str(json['index']))

  def restart_hri(self, json):    
    """
        NOT WORKING FOR NOW.
        Callback function when the STOP button is clicked on the screen. Launches a reset of HRI and React.

        :param json: JSON with data of the last event
        :type json: dict 
    """
    
    self.socketIO.emit(rospy.get_param('~socket_emit_restart_hri'),broadcast=True)
    self.json_for_GM={
                        "indexStep": self.index,
                        "actionName": self.currentAction,
                        "dataToUse": self.dataToUse,
                        "NextToDo": "RESTART",
                        "NextIndex": self.index+1
    }

    self.pub_test_restart.publish(True)
    self.action_GM_TO_HRI_result.Gm_To_Hri_output=js.dumps(self.json_for_GM)
    rospy.loginfo("{class_name} : Action GM TO HRI succeeded".format(class_name=self.__class__.__name__))
    
    self.action_GM_TO_HRI_server.set_succeeded(self.action_GM_TO_HRI_result)
    self.reset_for_restart()

    # self.init_variables()

    rospy.logwarn("{class_name} : HRI RESTARTED".format(class_name=self.__class__.__name__))
    # self.pub_restart_request.publish("RESTART")


if __name__ == '__main__':

  hri = HRIManager()

  while not rospy.is_shutdown():
    hri.socketIO.wait(seconds=0.1)
    rospy.spin()

