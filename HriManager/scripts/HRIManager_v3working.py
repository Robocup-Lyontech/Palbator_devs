#!/usr/bin/env python


from socketIO_client import SocketIO, LoggingNamespace
from python_depend.views import Views
import json as js

from std_msgs.msg import String
import rospy
import os 
import actionlib
from HriManager.msg import GmToHriAction, GmToHriFeedback, GmToHriResult
import time
import weakref
from copy import deepcopy

from python_depend.views import Views
from std_msgs.msg import Empty
from actionlib_msgs.msg import GoalID
import speechToTextPalbator.msg
import ttsMimic.msg

class HRIManager:

  def __del__(self):
    rospy.loginfo("HRI ON SHUTDOWN")

  def __init__(self,socket):
    
    rospy.init_node("ROS_HRI_node",anonymous=True)

    self.pub_choice_scenario=rospy.Publisher("choice_scenario",String,queue_size=1)

    self.action_online_client = actionlib.SimpleActionClient("action_STT_online",speechToTextPalbator.msg.SttOnlineAction)

    self.action_offline_client = actionlib.SimpleActionClient("action_STT_offline",speechToTextPalbator.msg.SttOfflineAction)

    self.client_TTS=actionlib.SimpleActionClient("action_TTS_MIMIC",ttsMimic.msg.TtsMimicAction)

    self.enable_vocal_detection = False

    if self.enable_vocal_detection == True:
      rospy.loginfo("Waiting for online server...")
      self.action_online_client.wait_for_server()
      rospy.loginfo("Connected to server")

      rospy.loginfo("Waiting for offline server...")
      self.action_offline_client.wait_for_server()
      rospy.loginfo("Connected to offline server")

    rospy.loginfo("Waiting for TTS server...")
    self.client_TTS.wait_for_server()
    rospy.loginfo("TTS server connected")


    # self.connection_ON=None
    self.connection_ON=True

    self.enable_changing_connection=True
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
    self.indexFailure=None
    self.restart=0
    self.data_received=False
    self.currentIndexDataReceivedJS=0
    self.end_guest_procedure=None

    self.action_GM_TO_HRI_server=actionlib.SimpleActionServer("action_GmToHri",GmToHriAction,self.action_GmToHri_callback,auto_start=False)
    self.action_GM_TO_HRI_feedback=GmToHriFeedback()
    self.action_GM_TO_HRI_result=GmToHriResult()
    
    self.json_for_GM=None
    self.event_detected_flag=False

    self.stepsList=None
    self.enable_choice_scenario=True
    
    self.action_running=False

    self.event_touch = False

    self.socketIO=socket

    #callback quand user choisit scenario sur tablette
    self.socketIO.on('askToChangeScenarioHRIM',self.chooseScenario)

    #callback reponse quand scenario choisit et charge
    self.socketIO.on('scenarioCharged',self.scenarioCharged)

    #callback quand on recoit des donnees de la tablette
    self.socketIO.on('indexDataReceivedJS', self.indexDataJSstepDone)
    self.socketIO.on('dataReceivedJS', self.dataJSstepDone)

    #recuperation de l'order de la view chargee par la tablette
    self.socketIO.on('launchedView', self.send_gm_view_launched)

    # self.socketIO.on('resetHRI', self.restart_hri)

    self.view_launcher=Views(self.socketIO)
    self.action_GM_TO_HRI_server.start()
    self.sub = rospy.Subscriber("connection_state",String,self.handle_change_connection_state)
    rospy.loginfo('HRI MANAGER LAUNCHED')

  def handle_change_connection_state(self,req):
    if self.enable_changing_connection==True:
      if req.data=='Connected':
        self.action_offline_client.cancel_all_goals()
        self.connection_ON=True
          
      elif req.data=='Disconnected':
        self.action_online_client.cancel_all_goals()
        self.connection_ON=False


  def tts_action(self,speech):
    self.goal_TTS=ttsMimic.msg.TtsMimicGoal(speech)
    self.client_TTS.send_goal(self.goal_TTS)
    self.client_TTS.wait_for_result()


  def action_GmToHri_callback(self,goal):
    self.action_running=True
    rospy.loginfo("Action initiating ...")
    success=True
    json_output=None
    self.action_GM_TO_HRI_feedback.Gm_To_Hri_feedback=''
    json_goal=js.loads(goal.json_request)
    if json_goal['action']=="stepsList":
      self.scenario_loaded=False
      rospy.loginfo("Getting steps list...")
      self.stepsList=json_goal['stepsList']
      self.choosen_scenario=json_goal['scenario']
      self.json_for_GM={
        "result": "Steps received"
      }

    elif json_goal['action']=="currentStep":
      if json_goal['stepIndex']==0:
        json_charge_scenario={
          'scenario': self.choosen_scenario
        }
        self.chargeScenario(json_charge_scenario)

      which_step_action=deepcopy(self.stepsList[json_goal['stepIndex']]['action'])
      which_step_name=deepcopy(self.stepsList[json_goal['stepIndex']]['name'])
        
      if self.choosen_scenario=='receptionist':
        ##### rajouter des conditions si nouvelles actions importantes
        if which_step_action != '':
          if which_step_action=='askOpenDoor':
            self.procedure_action_askOpenDoor(json_goal['stepIndex'])

          elif which_step_action=='presentPerson':
            self.procedure_action_presentPerson(json_goal['stepIndex'],json_goal['data'])

          elif which_step_action=='seatGuest':
            self.procedure_action_seatGuest(json_goal['stepIndex'],json_goal['data'])

          elif which_step_action=="pointTo":
            self.procedure_action_pointTo(json_goal['stepIndex'],json_goal['data'])

          elif which_step_action=='askToFollow':
            self.procedure_action_askToFollow(json_goal['stepIndex'],json_goal['data'])

          else:
            #### procedure vue statique
            self.procedure_static_view(json_goal['stepIndex'])
        
        else:
          if "Ask infos" in which_step_name:
            ##### procedure action guest infos
            self.procedure_action_guest_infos(json_goal['stepIndex'])

          else:
            #### procedure vue statique
            self.procedure_static_view(json_goal['stepIndex'])
      else:
        if 'Ask room' in which_step_name:
          self.procedure_action_ask_room(json_goal['stepIndex'])
        else:
          self.procedure_static_view(json_goal['stepIndex'])

    json_output=self.json_for_GM
    if 'result' in json_output and json_output['result']=='PREEMPTED':
      success=False

    self.action_GM_TO_HRI_server.publish_feedback(self.action_GM_TO_HRI_feedback)
    if success:
      self.action_GM_TO_HRI_result.Gm_To_Hri_output=js.dumps(json_output)
      rospy.loginfo("Action GM TO HRI succeeded")
      
      self.action_GM_TO_HRI_server.set_succeeded(self.action_GM_TO_HRI_result)
    self.action_running=False


  def procedure_action_ask_room(self,indexStep):
    
    self.end_room_procedure=False
    index_procedure=indexStep
    while self.end_room_procedure==False: 
      self.currentStep=deepcopy(self.stepsList[index_procedure])
      self.index=deepcopy(self.currentStep['order'])
      self.currentAction=deepcopy(self.currentStep['action'])
      self.event_touch = False
      if self.currentAction == '':
        if self.index != 0:
          stepCompletedJson = {"idSteps": self.indexStepCompleted}
          self.socketIO.emit('CompleteStep',stepCompletedJson,broadcast=True)
          rospy.loginfo("ETAPE TERMINEE: "+str(self.currentStep['name']))
          self.indexStepCompleted = self.index
        else:
          self.indexStepCompleted = self.index
        
        rospy.loginfo("ETAPE COURANTE A DEMARRER: "+self.currentStep['name'])
        dataJsonToSendCurrentStep = {
            "index": self.index,
            "step":self.currentStep
        }
        self.socketIO.emit('stepCurrent',dataJsonToSendCurrentStep,broadcast=True)
        if self.currentStep['id'] != 'FinishScenario':
          dataJsonToSendTimer = {
              "state": 'TOGGLE_TIMER/ON'
          }
          self.socketIO.emit('startTimer',dataJsonToSendTimer, broadcast=True)
        rospy.loginfo("ETAPE DEMARREE: "+self.currentStep['name'])
      
        index_procedure=index_procedure+1
      else:
        name = deepcopy(self.currentStep['name'])
        speech=deepcopy(self.currentStep['speech']['said'])
        title=deepcopy(self.currentStep['speech']['title'])

        if "location_name" in speech:
          speech=speech.replace("location_name",self.choosenRoom)
          title=title.replace("location_name",self.choosenRoom)
          self.currentStep['speech']['said'] = speech
          self.currentStep['speech']['title'] = title

        self.view_launcher.start(self.currentStep['action'],self.currentStep, self.index, self.dataToUse)
        rospy.loginfo("CHARGEMENT VUE SOUS ETAPE:"+self.currentStep['name'] +" SUR TABLETTE")

        rospy.loginfo("CHOOSEN ROOM : "+str(self.choosenRoom))

        

        if self.enable_vocal_detection == False:
          while self.event_touch == False:
            socketIO.wait(0.1)
          self.event_touch = False
        else:
          if self.connection_ON==True:
            self.routine_online()

          elif self.connection_ON==False:
            self.routine_offline()

        rospy.logwarn("OLEEEEEEEEEEEEEEEEEe "+str(self.dataToUse))

        if self.currentAction == 'askRoomToClean':
          self.choosenRoom = self.dataToUse
          index_procedure = index_procedure + 1
        
        elif self.currentAction == 'confirm':
          if self.dataToUse=='false' or self.dataToUse=='NO':
              rospy.loginfo("RECEIVED DATA -> FALSE")
              index_procedure=index_procedure-1
          elif self.dataToUse=='true' or self.dataToUse=='YES':
            rospy.loginfo("RECEIVED DATA -> TRUE")
            index_procedure=index_procedure+1
            self.end_room_procedure=True

       
    self.json_for_GM={
      "indexStep": self.index,
      "actionName": self.currentAction,
      "dataToUse": self.dataToUse,
      "NextToDo": "next",
      "NextIndex": self.index+1,
      "saveData":{
        "where": self.choosenRoom
      }
    }
    rospy.loginfo("--------END PROCEDURE ASK ROOM-------------")


  def procedure_action_askToFollow(self,indexStep,people):
    self.currentStep=deepcopy(self.stepsList[indexStep])
    self.index=deepcopy(self.currentStep['order'])
    self.currentAction=deepcopy(self.currentStep['action'])

    key=self.currentStep['arguments']['who']
    self.currentStep['speech']['said']=self.currentStep['speech']['said'].replace(key+"_name",people[key]['name'])

    self.view_launcher.start(self.currentAction,self.currentStep, self.index, self.dataToUse)
    rospy.loginfo("CHARGEMENT VUE SOUS ETAPE:"+self.currentStep['name'] +" SUR TABLETTE")

    speech=deepcopy(self.currentStep['speech']['said'])
    self.tts_action(speech)

    self.json_for_GM={
        "indexStep": self.index,
        "actionName": '',
        "NextToDo": "next",
        "NextIndex": self.index+1
    }
    self.event_detected_flag=True
    self.socketIO.wait(seconds=0.1)

  def procedure_action_seatGuest(self,indexStep,people):
    self.currentStep=deepcopy(self.stepsList[indexStep])
    self.index=deepcopy(self.currentStep['order'])
    self.currentAction=deepcopy(self.currentStep['action'])


    for key in people.keys():
      if key in self.currentStep['arguments']['who']['name']:
        self.currentStep['arguments']['who']['name']=people[key]['name']
        self.currentStep['arguments']['who']['guestPhotoPath']=people[key]['guestPhotoPath']
        self.currentStep['speech']['said']=self.currentStep['speech']['said'].replace(key+"_name",people[key]['name'])
        self.currentStep['speech']['title']=self.currentStep['speech']['title'].replace(key+"_name",people[key]['name'])

    self.view_launcher.start(self.currentAction,self.currentStep, self.index, self.dataToUse)
    rospy.loginfo("CHARGEMENT VUE SOUS ETAPE:"+self.currentStep['name'] +" SUR TABLETTE")

    speech=deepcopy(self.currentStep['speech']['said'])
    self.tts_action(speech)

    self.json_for_GM={
        "indexStep": self.index,
        "actionName": '',
        "NextToDo": "next",
        "NextIndex": self.index+1
    }
    self.event_detected_flag=True
    self.socketIO.wait(seconds=0.1)


  def procedure_action_pointTo(self,indexStep,people):
    self.currentStep=deepcopy(self.stepsList[indexStep])
    self.index=deepcopy(self.currentStep['order'])
    self.currentAction=deepcopy(self.currentStep['action'])

    object_to_point=deepcopy(self.currentStep['arguments']['what'])

    if object_to_point=='chair':
      pass

    elif object_to_point=='human':
      for key in people.keys():
        if key in self.currentStep['arguments']['who']['name']:
          self.currentStep['arguments']['who']['name']=people[key]['name']
          self.currentStep['arguments']['who']['guestPhotoPath']=people[key]['guestPhotoPath']
          self.currentStep['speech']['said']=self.currentStep['speech']['said'].replace(key+"_name",people[key]['name'])
          self.currentStep['speech']['title']=self.currentStep['speech']['title'].replace(key+"_name",people[key]['name'])

    self.view_launcher.start(self.currentAction,self.currentStep, self.index, self.dataToUse)
    rospy.loginfo("CHARGEMENT VUE SOUS ETAPE:"+self.currentStep['name'] +" SUR TABLETTE")

    speech=deepcopy(self.currentStep['speech']['said'])
    self.tts_action(speech)

    self.json_for_GM={
        "indexStep": self.index,
        "actionName": '',
        "NextToDo": "next",
        "NextIndex": self.index+1
    }
    self.event_detected_flag=True
    self.socketIO.wait(seconds=0.1)


  def procedure_action_presentPerson(self,indexStep,people):
    self.currentStep=deepcopy(self.stepsList[indexStep])
    self.index=deepcopy(self.currentStep['order'])
    self.currentAction=deepcopy(self.currentStep['action'])

    speech=self.currentStep['speech']['said']
    number_know_guests=len(self.currentStep['arguments']['to'])

    for key in people.keys():
      for i in range(0,number_know_guests):
        if key in self.currentStep['arguments']['to'][i]['name']:
          self.currentStep['arguments']['to'][i]['name']=people[key]['name']
          self.currentStep['arguments']['to'][i]['guestPhotoPath']=people[key]['guestPhotoPath']
          self.currentStep['arguments']['to'][i]['drink']['name']=people[key]['drink']
          self.currentStep['arguments']['to'][i]['drink']['pathOnTablet']=people[key]['pathOnTablet']

          rospy.logwarn("KEY "+str(key)+" DRINK "+str(people[key]['drink'])+" DRINKPATH "+str(people[key]['pathOnTablet']))

      if key in self.currentStep['arguments']['who']['name']:
        self.currentStep['arguments']['who']['name']=people[key]['name']
        self.currentStep['arguments']['who']['guestPhotoPath']=people[key]['guestPhotoPath']
        self.currentStep['arguments']['who']['drinkObj']['name']=people[key]['drink']
        self.currentStep['arguments']['who']['drinkObj']['pathOnTablet']=people[key]['pathOnTablet']
        rospy.logwarn("KEY "+str(key)+" DRINK "+str(people[key]['drink'])+" DRINKPATH "+str(people[key]['pathOnTablet']))

      speech=speech.replace(key+'_name',people[key]['name'])
      speech=speech.replace(key+'_drink',people[key]['drink'])
  

    self.currentStep['speech']['said']=speech
    self.view_launcher.start(self.currentAction,self.currentStep, self.index, self.dataToUse)
    rospy.loginfo("CHARGEMENT VUE SOUS ETAPE:"+self.currentStep['name'] +" SUR TABLETTE")

    speech=deepcopy(self.currentStep['speech']['said'])
    self.tts_action(speech)
    self.json_for_GM={
        "indexStep": self.index,
        "actionName": '',
        "NextToDo": "next",
        "NextIndex": self.index+1
    }
    self.event_detected_flag=True
    self.socketIO.wait(seconds=0.1)



  def procedure_static_view(self,indexStep):
    self.currentStep=deepcopy(self.stepsList[indexStep])
    self.index=deepcopy(self.currentStep['order'])
    self.currentAction=deepcopy(self.currentStep['action'])
    if  self.currentAction == '':
      if self.index != 0:
        stepCompletedJson = {"idSteps": self.indexStepCompleted}
        self.socketIO.emit('CompleteStep',stepCompletedJson,broadcast=True)
        rospy.loginfo("ETAPE TERMINEE: "+str(self.currentStep['name']))
        self.indexStepCompleted = self.index
      else:
        self.indexStepCompleted = self.index

      rospy.loginfo("ETAPE COURANTE A DEMARRER: "+self.currentStep['name'])
      dataJsonToSendCurrentStep = {
          "index": self.index,
          "step":self.currentStep
      }
      self.socketIO.emit('stepCurrent',dataJsonToSendCurrentStep,broadcast=True)
      if self.currentStep['id'] != 'FinishScenario':
        dataJsonToSendTimer = {
            "state": 'TOGGLE_TIMER/ON'
        }
        self.socketIO.emit('startTimer',dataJsonToSendTimer, broadcast=True)
      rospy.loginfo("ETAPE DEMARREE: "+self.currentStep['name'])
      self.json_for_GM={
        "indexStep": self.index,
        "actionName": '',
        "NextToDo": "next",
        "NextIndex": self.index+1
      }
      self.event_detected_flag=True
      self.socketIO.wait(seconds=0.1)

    else:
      self.view_launcher.start(self.currentStep['action'],self.currentStep, self.currentStep['order'], self.dataToUse)
      rospy.loginfo("CHARGEMENT VUE SOUS ETAPE:"+self.currentStep['name'] +" SUR TABLETTE")

      speech=deepcopy(self.currentStep['speech']['said'])
      self.tts_action(speech)
      self.json_for_GM={
        "indexStep": self.index,
        "actionName": '',
        "NextToDo": "next",
        "NextIndex": self.index+1
      }
      self.event_detected_flag=True
      self.socketIO.wait(seconds=0.1)

  def routine_online(self):
    rospy.loginfo("----------- DEBUT ROUTINE ONLINE--------------------------------")
    self.goal_online = speechToTextPalbator.msg.SttOnlineGoal()
    rospy.loginfo("Sending goal to online ...")
    order={
        'order': self.index,
        'action': self.currentAction
    }
    json_in_str=js.dumps(order)
    self.goal_online.order=json_in_str
    self.action_online_client.send_goal(self.goal_online)
    cp=0
    while self.action_online_client.get_result() is None and not rospy.is_shutdown():
      if cp==20 or self.connection_ON==False:
        self.action_online_client.cancel_all_goals()
        self.tts_action('Switching to offline mode')
        break
      elif self.event_touch == True:
        self.action_online_client.cancel_all_goals()
        break

      rospy.loginfo("Waiting for online detect ....")
      cp=cp+1
      self.socketIO.wait(seconds=0.1)

    rospy.loginfo(str(self.action_online_client.get_result()))
    if not self.action_online_client.get_result() is None and self.action_online_client.get_result().stt_result != '':
      self.dataToUse=str(self.action_online_client.get_result().stt_result)
    elif self.event_touch==True:
      self.event_touch = False
      rospy.logwarn("EVENT TOUCH "+str(self.event_touch))
    else:
      self.enable_changing_connection=False
      self.routine_offline()
      self.enable_changing_connection=True

    rospy.loginfo("----------- FIN ROUTINE ONLINE--------------------------------")

  def routine_offline(self):
    rospy.loginfo("----------- DEBUT ROUTINE OFFLINE--------------------------------")
    self.goal_offline = speechToTextPalbator.msg.SttOfflineGoal()
    rospy.loginfo("Sending goal to offline ...")
    order={
        'order': self.index,
        'action': self.currentAction
    }
    json_in_str=js.dumps(order)
    self.goal_offline.order=json_in_str
    self.action_offline_client.send_goal(self.goal_offline)
    while self.action_offline_client.get_result() is None and not rospy.is_shutdown():
      if self.event_touch == True:
        self.action_offline_client.cancel_all_goals()
        self.event_touch = False
        rospy.logwarn("EVENT TOUCH "+str(self.event_touch))
        break
      rospy.loginfo("Waiting for OFFLINE detect ....")
      self.socketIO.wait(seconds=0.1)
    rospy.loginfo(str(self.action_offline_client.get_result()))
    if str(self.action_offline_client.get_result().stt_result) != '':
      self.dataToUse=str(self.action_offline_client.get_result().stt_result)
    rospy.loginfo("----------- FIN ROUTINE OFFLINE--------------------------------")


  def procedure_action_askOpenDoor(self,indexStep):
    self.currentStep=deepcopy(self.stepsList[indexStep])
    self.index=deepcopy(self.currentStep['order'])
    self.nameAction = deepcopy(self.currentStep['name'])
    self.currentAction=deepcopy(self.currentStep['action'])
    self.event_touch = False
    self.view_launcher.start(self.currentAction,self.currentStep, self.index, self.dataToUse)
    rospy.loginfo("CHARGEMENT VUE SOUS ETAPE:"+self.currentStep['name'] +" SUR TABLETTE")
    speech=deepcopy(self.currentStep['speech']['said'])
    self.tts_action(speech)

    if self.enable_vocal_detection == True:
      if self.connection_ON==True:
        self.routine_online()

      elif self.connection_ON==False:
        self.routine_offline()
    else:
      while self.event_touch == False:
        socketIO.wait(0.1)
      self.event_touch = False



    self.json_for_GM={
            "indexStep": self.index,
            "actionName": self.currentAction,
            "dataToUse": self.dataToUse,
            "NextToDo": "next",
            "NextIndex": self.index+1
          }
    rospy.loginfo("----------END PROCEDURE ASK OPEN DOOR--------")

  def procedure_action_guest_infos(self,indexStep):
    self.end_guest_procedure=False
    index_procedure=indexStep
    while self.end_guest_procedure==False: 
      self.currentStep=deepcopy(self.stepsList[index_procedure])
      self.index=deepcopy(self.currentStep['order'])

      self.currentAction=deepcopy(self.currentStep['action'])

      if self.currentAction == '':
        if self.index != 0:
          stepCompletedJson = {"idSteps": self.indexStepCompleted}
          self.socketIO.emit('CompleteStep',stepCompletedJson,broadcast=True)
          rospy.loginfo("ETAPE TERMINEE: "+str(self.currentStep['name']))
          self.indexStepCompleted = self.index
        else:
          self.indexStepCompleted = self.index
        
        rospy.loginfo("ETAPE COURANTE A DEMARRER: "+self.currentStep['name'])
        dataJsonToSendCurrentStep = {
            "index": self.index,
            "step":self.currentStep
        }
        self.socketIO.emit('stepCurrent',dataJsonToSendCurrentStep,broadcast=True)
        if self.currentStep['id'] != 'FinishScenario':
          dataJsonToSendTimer = {
              "state": 'TOGGLE_TIMER/ON'
          }
          self.socketIO.emit('startTimer',dataJsonToSendTimer, broadcast=True)
        rospy.loginfo("ETAPE DEMARREE: "+self.currentStep['name'])
      
        index_procedure=index_procedure+1
      
      else:
        self.event_touch = False
        name = deepcopy(self.currentStep['name'])
        self.view_launcher.start(self.currentStep['action'],self.currentStep, self.index, self.dataToUse)
        rospy.loginfo("CHARGEMENT VUE SOUS ETAPE:"+self.currentStep['name'] +" SUR TABLETTE")

        rospy.loginfo("CHOOSEN NAME :"+str(self.choosenName))
        rospy.loginfo("CONTENT NAMETOUSE "+str(self.nameToUse))
        rospy.loginfo("--------------")
        rospy.loginfo("CHOOSEN DRINK :"+str(self.choosenDrink))
        rospy.loginfo("CONTENT DRINKTOUSE "+str(self.drinkToUse))

        speech=deepcopy(self.currentStep['speech']['said'])
        title=deepcopy(self.currentStep['speech']['title'])
        if not self.dataToUse is None:
          if "Confirm" in name:
            if "drink" in name:
              speech=speech.format(drink=str(self.dataToUse))
              title=title.format(drink=str(self.dataToUse))
            elif "name" in name:
              speech=speech.format(name=self.dataToUse)
              title=title.format(name=self.dataToUse)
          else:
            if "drink" in name:
              speech=speech.format(name=str(self.nameToUse[-1]))


        
        self.tts_action(speech)

        if self.enable_vocal_detection == True:
          if self.connection_ON==True:
            self.routine_online()

          elif self.connection_ON==False:
            self.routine_offline()
        else:
          while self.event_touch == False:
            socketIO.wait(0.1)
          self.event_touch = False

        if self.dataToUse != '':
          if self.currentAction=='askName':
            self.choosenName=self.dataToUse

          elif self.currentAction=='askDrink':
            self.choosenDrink=self.dataToUse

          elif self.currentAction=='askAge':
            self.choosenAge=self.dataToUse

          elif self.currentAction=='confirm':
            if self.dataToUse == 'true' or self.dataToUse == 'YES':
              if "name" in self.currentStep['name']:
                self.nameToUse.append(self.choosenName)
              elif "drink" in self.currentStep['name']:
                self.drinkToUse.append(self.choosenDrink)
            elif self.dataToUse == 'false' or self.dataToUse == 'NO':
              if "name" in self.currentAction:
                self.choosenName=''
              elif "drink" in self.currentAction:
                self.choosenDrink=''
        
        if "Confirm" in name:
          if self.dataToUse=='false' or self.dataToUse=='NO':
            rospy.loginfo("RECEIVED DATA -> FALSE")
            index_procedure=index_procedure-1
          elif self.dataToUse=='true' or self.dataToUse=='YES':
            rospy.loginfo("RECEIVED DATA -> TRUE")
            index_procedure=index_procedure+1
        else:
            index_procedure=index_procedure+1

        if self.currentStep['action']=='askAge':
          self.end_guest_procedure=True
          break
      self.socketIO.wait(seconds=0.1)
    self.json_for_GM={
      "indexStep": self.index,
      "actionName": self.currentAction,
      "dataToUse": self.dataToUse,
      "NextToDo": "next",
      "NextIndex": self.index+1,
      "saveData":{
        "who": self.currentStep['arguments']['who'],
        "name": self.nameToUse[-1],
        "drink": self.drinkToUse[-1],
        "age": self.choosenAge
      }
    }
    rospy.loginfo("--------END PROCEDURE ASK INFOS ABOUT GUEST-------------")

  def indexDataJSstepDone(self,json):
    if('data' in json):
      self.currentIndexDataReceivedJS = json['data']

##################################### DATA RECEIVED FROM TOUCH MANAGER #################################################
  def dataJSstepDone(self,json):
    self.client_TTS.cancel_all_goals()
    self.event_detected_flag=True
    
    if (self.data_received is False and self.index == self.currentIndexDataReceivedJS):
      self.event_touch = True
      rospy.logwarn("EVENT TOUCH "+str(self.event_touch))
      rospy.loginfo("DONNEE RECUE DEPUIS TOUCH MANAGER")
      self.action_online_client.cancel_all_goals()
      self.action_offline_client.cancel_all_goals()
      self.data_received=True
      self.dataToUse = json['data']
      if(self.currentAction != 'confirm'):
        rospy.loginfo("ACTION SANS CONFIRM")
        
        rospy.loginfo("DONNEE TOUCH MANAGER: "+str(self.dataToUse))

        if self.currentAction != 'askName' and self.currentAction != 'askDrink' and self.currentAction != 'askAge':
          self.json_for_GM={
            "indexStep": self.index,
            "actionName": self.currentAction,
            "dataToUse": self.dataToUse,
            "NextToDo": "next",
            "NextIndex": self.index+1
          }
          self.event_detected_flag=True
        else:
          if self.currentAction=='askAge':
            rospy.loginfo("NAME LIST : "+str(self.nameToUse))
            rospy.loginfo("DRINK LIST : "+str(self.drinkToUse))
            self.choosenAge=self.dataToUse
            self.json_for_GM={
              "indexStep": self.index,
              "actionName": self.currentAction,
              "dataToUse": self.dataToUse,
              "NextToDo": "next",
              "NextIndex": self.index+1,
              "saveData":{
                "who": self.currentStep['arguments']['who'].replace(" ","_"),
                "name": self.nameToUse[-1],
                "drink": self.drinkToUse[-1],
                "age": self.dataToUse
              }
            }
            self.event_detected_flag=True
          else:
            if self.currentAction== 'askName':
              self.choosenName=self.dataToUse
            else:
              self.choosenDrink=self.dataToUse

      else:
        rospy.loginfo("ACTION AVEC CONFIRM")
        rospy.loginfo("DONNEE TOUCH MANAGER: "+str(json['data']))
        if(json['data'] != 'false'):
          if "name" in self.currentStep['name']:
            rospy.loginfo("STORING DATA NAME IN LIST")
            self.nameToUse.append(self.choosenName)
          elif "drink" in self.currentStep['name']:
            rospy.loginfo("STORING DATA DRINK IN LIST")
            self.drinkToUse.append(self.choosenDrink)
        else:
          if "name" in self.currentStep['name']:
            self.choosenName=''
          elif "drink" in self.currentStep['name']:
            self.choosenDrink=''

    else:
      rospy.logwarn('Le touch Manager envoie une donnee a la mauvaise etape: '+str(self.currentIndexDataReceivedJS)+' au lieu de '+str(self.index))

    self.data_received=False
    
  ###########################

  def scenarioCharged(self,json):
    rospy.loginfo("HRI : SCENARIO CHARGED")
    self.json_for_GM=json
    self.scenario_loaded=True

  def chooseScenario(self,json):
    if self.enable_choice_scenario==True:
      self.enable_choice_scenario=False
      rospy.loginfo("choosing scenario...")
      self.choosen_scenario = json['scenario']
      self.pub_choice_scenario.publish(self.choosen_scenario)


  ########################## Chargement du scenario selectionne ################

  ######### On recoit depuis le REACT le nom du scenario selectionne et on charge le json approprie pour lui renvoyer ############
  def chargeScenario(self,json):
      self.socketIO.emit('scenarioToCharged',json, broadcast=True)
  
  def send_gm_view_launched(self,json):
    self.json_confirm_View_launch=json
    rospy.loginfo(str(json['data']))
    rospy.loginfo("Index de la vue lancee sur le Touch : "+str(json['index']))

  def restart_hri(self,json):
    self.nameToUse=[]
    self.drinkToUse=[]
    self.ageToUse = []
    self.choosenName = None
    self.choosenDrink = None
    self.choosenAge = None
    self.nameAction = None
    self.currentStep = None
    self.currentAction = None
    self.index=0
    self.indexStepCompleted=0
    self.dataToUse=''
    self.indexFailure=None
    self.restart=0
    self.data_received=False
    self.currentIndexDataReceivedJS=0
    self.end_guest_procedure=None
    
    socketIO.emit('restartHRI',broadcast=True)
    self.pub_restart_request.publish("RESTART")


if __name__ == '__main__':
  
  
  socketIO = SocketIO('http://127.0.0.1', 5000, LoggingNamespace)
  hri=HRIManager(socketIO)


  while not rospy.is_shutdown():
    rospy.spin()

