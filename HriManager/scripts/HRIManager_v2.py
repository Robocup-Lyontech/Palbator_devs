#!/usr/bin/env python


from socketIO_client import SocketIO, LoggingNamespace
# from __main__ import socketIO
from python_depend.views import Views
import json as js

from std_msgs.msg import String
from HriManager.msg import DataToSay
import rospy
import os 
import actionlib
from HriManager.msg import GmToHriAction, GmToHriFeedback, GmToHriResult
import time
import weakref
# from testThomas_Receptionist2020CPEScenario import InterruptExecution
from copy import deepcopy

from python_depend.views import Views
from std_msgs.msg import Empty
from actionlib_msgs.msg import GoalID
import rapp_platform_ros_communications.msg

class HRIManager:

  def __del__(self):
    rospy.loginfo("HRI ON SHUTDOWN")

  def __init__(self,socket):
    
    rospy.init_node("ROS_HRI_node",anonymous=True)
    
    self.pub_current_view=rospy.Publisher("CurrentView",DataToSay,queue_size=10)
    self.subscriber=rospy.Subscriber("SendData",String,self.handle_data)

    self.pub_choice_scenario=rospy.Publisher("choice_scenario",String,queue_size=1)
    self.pub_event_TM=rospy.Publisher("event_on_TM",String,queue_size=1)
    self.pub_event_VM=rospy.Publisher("event_on_VM",String,queue_size=1)
    self.pub_restart_request=rospy.Publisher("HRI_restart_request",String,queue_size=1)


    # self.pub_restart_mode=rospy.Publisher("restarting_mode",String,queue_size=1)
    # self.sub_restart_mode=rospy.Subscriber("restarting_mode",String,self.handle_restart_mode)
    # self.sub_server=rospy.Subscriber("test",Empty,self.handle_sub)
    # self.sub_cancel_request=rospy.Subscriber("/action_GmToHri/cancel",GoalID,self.handle_cancel_action)
    
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

    self.action_GM_TO_HRI_server=actionlib.SimpleActionServer("action_GmToHri",GmToHriAction,self.action_GmToHri_callback,auto_start=False)
    self.action_GM_TO_HRI_feedback=GmToHriFeedback()
    self.action_GM_TO_HRI_result=GmToHriResult()
    self.action_GM_TO_HRI_server.start()
    self.json_for_GM=None

    self.client_action_tts=actionlib.SimpleActionClient("action_TTS",rapp_platform_ros_communications.msg.SayVocalSpeechAction)
    rospy.loginfo("waiting for tts server...")
    self.client_action_tts.wait_for_server()

    self.event_detected_flag=False

    self.stepsList=None
    self.restart_order=False
    self.enable_choice_scenario=True
    
    self.restart_mode="OFF"
    self.action_running=False

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
    rospy.loginfo('HRI MANAGER LAUNCHED')

  def action_GmToHri_callback(self,goal):
    if self.restart_mode=='OFF':
      self.action_running=True
      rospy.loginfo("Action initiating ...")
      success=True
      json_output=None
      self.action_GM_TO_HRI_feedback.Gm_To_Hri_feedback=''
      json_goal=js.loads(goal.json_request)
      if json_goal['whatToDo']=="Load scenario":
        self.scenario_loaded=False
        rospy.loginfo("Getting steps list...")
        self.stepsList=json_goal['stepsList']
        self.choosen_scenario=json_goal['scenario']
        self.json_for_GM={
          "result": "Steps received"
        }

      elif json_goal['whatToDo']=="Load step":
        if json_goal['stepIndex']==0:
          json_charge_scenario={
            'scenario': self.choosen_scenario
          }
          self.chargeScenario(json_charge_scenario)

        which_step_action=deepcopy(self.stepsList[json_goal['stepIndex']]['action'])
        which_step_name=deepcopy(self.stepsList[json_goal['stepIndex']]['name'])
        
        
        ##### rajouter des conditions si nouvelles actions importantes
        if which_step_action != '':
          if which_step_action=='askOpenDoor':
            #####procedure action askOpenDoor
            self.procedure_action_askOpenDoor(json_goal['stepIndex'])

          elif which_step_action=='presentPerson':
            self.procedure_action_presentPerson(json_goal['stepIndex'],json_goal['people'])

          elif which_step_action=='seatGuest':
            self.procedure_action_seatGuest(json_goal['stepIndex'],json_goal['people'])

          elif which_step_action=="pointTo":
            self.procedure_action_pointTo(json_goal['stepIndex'],json_goal['people'])

          elif which_step_action=='askToFollow':
            self.procedure_action_askToFollow(json_goal['stepIndex'],json_goal['people'])

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

      json_output=self.json_for_GM
      if 'result' in json_output and json_output['result']=='PREEMPTED':
        success=False

      self.action_GM_TO_HRI_server.publish_feedback(self.action_GM_TO_HRI_feedback)
      if success:
        self.action_GM_TO_HRI_result.Gm_To_Hri_output=js.dumps(json_output)
        rospy.loginfo("Action GM TO HRI succeeded")
        
        self.action_GM_TO_HRI_server.set_succeeded(self.action_GM_TO_HRI_result)
      self.action_running=False

  def tts_action(self,speech):
    self.enable_vocal_data=False
    # self.client_action_tts.cancel_all_goals()
    goal=rapp_platform_ros_communications.msg.SayVocalSpeechGoal(speech)
    self.client_action_tts.send_goal(goal)
    self.client_action_tts.wait_for_result()
    result_action=self.client_action_tts.get_result()
    rospy.loginfo("Action result "+str(result_action))
    self.enable_vocal_data=True

  def procedure_action_askToFollow(self,indexStep,people):
    self.currentStep=deepcopy(self.stepsList[indexStep])
    self.index=deepcopy(self.currentStep['order'])
    self.currentAction=deepcopy(self.currentStep['action'])

    key=self.currentStep['arguments']['who']
    self.currentStep['speech']['said']=self.currentStep['speech']['said'].replace(key+"_name",people[key]['name'])

    self.view_launcher.start(self.currentAction,self.currentStep, self.index, self.dataToUse)
    rospy.loginfo("CHARGEMENT VUE SOUS ETAPE:"+self.currentStep['name'] +" SUR TABLETTE")
    messageDataToSay=DataToSay()
    messageDataToSay.json_in_string=js.dumps(self.currentStep)
    messageDataToSay.data_to_use_in_string=''
    self.pub_current_view.publish(messageDataToSay)

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
    messageDataToSay=DataToSay()
    messageDataToSay.json_in_string=js.dumps(self.currentStep)
    messageDataToSay.data_to_use_in_string=''
    self.pub_current_view.publish(messageDataToSay)

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
    messageDataToSay=DataToSay()
    messageDataToSay.json_in_string=js.dumps(self.currentStep)
    messageDataToSay.data_to_use_in_string=''
    self.pub_current_view.publish(messageDataToSay)

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
    messageDataToSay=DataToSay()
    messageDataToSay.json_in_string=js.dumps(self.currentStep)
    messageDataToSay.data_to_use_in_string=''
    self.pub_current_view.publish(messageDataToSay)
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
      messageDataToSay=DataToSay()
      messageDataToSay.json_in_string=js.dumps(self.currentStep)
      messageDataToSay.data_to_use_in_string=''
      self.pub_current_view.publish(messageDataToSay)
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

  def procedure_action_askOpenDoor(self,indexStep):
    self.restart_order=False
    self.currentStep=deepcopy(self.stepsList[indexStep])
    self.index=deepcopy(self.currentStep['order'])
    self.nameAction = deepcopy(self.currentStep['name'])
    self.currentAction=deepcopy(self.currentStep['action'])

    self.view_launcher.start(self.currentAction,self.currentStep, self.index, self.dataToUse)
    rospy.loginfo("CHARGEMENT VUE SOUS ETAPE:"+self.currentStep['name'] +" SUR TABLETTE")
    messageDataToSay=DataToSay()
    messageDataToSay.json_in_string=js.dumps(self.currentStep)
    messageDataToSay.data_to_use_in_string=''
    self.pub_current_view.publish(messageDataToSay)
    speech=deepcopy(self.currentStep['speech']['said'])
    self.tts_action(speech)

    ###wait for vocal data or touch data
    self.event_detected_flag=False
    while self.event_detected_flag==False:# and self.restart_order==False:
      self.action_GM_TO_HRI_feedback.Gm_To_Hri_feedback='Waiting for action TM or VM'
      self.action_GM_TO_HRI_server.publish_feedback(self.action_GM_TO_HRI_feedback)
      if self.action_GM_TO_HRI_server.is_preempt_requested():
        self.json_for_GM={
          "result": "PREEMPTED"
        }
        break
      self.socketIO.wait(seconds=0.1)


  def procedure_action_guest_infos(self,indexStep):
    self.restart_order=False
    self.end_guest_procedure=False
    index_procedure=indexStep
    while self.end_guest_procedure==False: #and self.restart_order==False:
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
        name = deepcopy(self.currentStep['name'])
        self.view_launcher.start(self.currentStep['action'],self.currentStep, self.index, self.dataToUse)
        rospy.loginfo("CHARGEMENT VUE SOUS ETAPE:"+self.currentStep['name'] +" SUR TABLETTE")
        messageDataToSay=DataToSay()
        messageDataToSay.json_in_string=js.dumps(self.currentStep)

        rospy.loginfo("CHOOSEN NAME :"+str(self.choosenName))
        rospy.loginfo("CONTENT NAMETOUSE "+str(self.nameToUse))
        rospy.loginfo("--------------")
        rospy.loginfo("CHOOSEN DRINK :"+str(self.choosenDrink))
        rospy.loginfo("CONTENT DRINKTOUSE "+str(self.drinkToUse))

        speech=deepcopy(self.currentStep['speech']['said'])

        if not self.dataToUse is None:
          if "Confirm" in name:
            if "drink" in name:
              messageDataToSay.data_to_use_in_string=str(self.dataToUse)
              speech=speech.format(drink=str(self.dataToUse))
            elif "name" in name:
              messageDataToSay.data_to_use_in_string=str(self.dataToUse)
              speech=speech.format(name=self.dataToUse)
          else:
            if "drink" in name:
              messageDataToSay.data_to_use_in_string=str(self.nameToUse[-1])
              speech=speech.format(name=str(self.nameToUse[-1]))
        else:
          messageDataToSay.data_to_use_in_string=''
        self.pub_current_view.publish(messageDataToSay)

        
        self.tts_action(speech)

        ###wait for vocal data or touch data
        self.event_detected_flag=False
        while self.event_detected_flag==False: # and self.restart_order==False:
          rospy.loginfo('Waiting for action TM or VM')
          self.action_GM_TO_HRI_feedback.Gm_To_Hri_feedback='Waiting for action TM or VM'
          self.action_GM_TO_HRI_server.publish_feedback(self.action_GM_TO_HRI_feedback)
          if self.action_GM_TO_HRI_server.is_preempt_requested():
            self.json_for_GM={
              "result": "PREEMPTED"
            }
            break
          self.socketIO.wait(seconds=0.1)

        if "Confirm" in name:
          if self.dataToUse=='false':
            rospy.loginfo("RECEIVED DATA -> FALSE")
            index_procedure=index_procedure-1
          else:
            rospy.loginfo("RECEIVED DATA -> TRUE")
            index_procedure=index_procedure+1
        else:
            index_procedure=index_procedure+1

        if self.currentStep['action']=='askAge':
          self.end_guest_procedure=True
          break
      self.socketIO.wait(seconds=0.1)


##################################### DATA RECEIVED FROM VOICE MANAGER #################################################

  def handle_data(self,req):
    data=req.data
    json_received=js.loads(data)
    if (self.data_received is False and self.index == json_received['index'] and self.enable_vocal_data is True):
      self.dataToUse = json_received['dataToUse']
      rospy.loginfo('CHARGEMENT DONNEE DE VOICE MANAGER')
      rospy.loginfo('DONNEE RECUE DEPUIS VOICE MANAGER %s',json_received['dataToUse'])
      self.data_received=True
      if(self.currentAction != 'confirm'):
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
          if self.currentAction== 'askAge':
            rospy.loginfo("NAME LIST : "+str(self.nameToUse))
            rospy.loginfo("DRINK LIST : "+str(self.drinkToUse))
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
                "age": self.dataToUse
              }
            }
            self.event_detected_flag=True
          else:
            if self.currentAction== 'askName':
              rospy.loginfo("CHOOSEN NAME BY VM : "+self.dataToUse)
              self.choosenName=self.dataToUse
            else:
              rospy.loginfo("CHOOSEN DRINK BY VM : "+self.dataToUse)
              self.choosenDrink=self.dataToUse
            self.event_detected_flag=True

      else:
        if(json_received['dataToUse'] != 'false'):
          if "name" in self.currentStep['name']:
            self.nameToUse.append(self.choosenName)
          elif "drink" in self.currentStep['name']:
            self.drinkToUse.append(self.choosenDrink)
          

        else:
          if "name" in self.currentAction:
            self.choosenName=''
          elif "drink" in self.currentAction:
            self.choosenDrink=''
        
        self.event_detected_flag=True

      self.data_received=False
    else:
      rospy.logwarn('Le voice Manager envoie une donnee a la mauvaise etape: '+str(json_received['index'])+' au lieu de '+str(self.index))


  def indexDataJSstepDone(self,json):
    if('data' in json):
      self.currentIndexDataReceivedJS = json['data']

##################################### DATA RECEIVED FROM TOUCH MANAGER #################################################
  def dataJSstepDone(self,json):
    self.event_detected_flag=True
    if (self.data_received is False and self.index == self.currentIndexDataReceivedJS):
      rospy.loginfo("DONNEE RECUE DEPUIS TOUCH MANAGER")
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
      # rospy.loginfo("SCENARIO CHARGE: "+str(json['scenario']))
  
  def send_gm_view_launched(self,json):
    self.json_confirm_View_launch=json
    rospy.loginfo(str(json['data']))
    rospy.loginfo("Index de la vue lancee sur le Touch : "+str(json['index']))

  
    # raise ValueError
    # self.pub_restart_request.publish("restart")
    

    # self.json_for_GM_restart={
    #   "actionName": "RESTART HRI",
    #   "NextToDo": "",
    #   "NextIndex": ""
    # }
    # self.restart_order=True
    # # self.action_GM_TO_HRI_result.Gm_To_Hri_output=js.dumps(self.json_for_GM)
    
    # self.HRI_reconfigure()

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

    self.restart_mode='ON'

    
    socketIO.emit('restartHRI',broadcast=True)
    self.pub_restart_request.publish("RESTART")




  
# class HRILauncher(object):

#   def __init__(self):
#     rospy.init_node("hri_manager_node",anonymous=True)

#     self.pub_restart=rospy.Publisher("HRI_restart_request", String,queue_size=1)
    
#     self.hri=HRIManager()

#     #callback pour revenir au menu principal sur la tablette
#     socketIO.on('resetHRI', self.restart_hri)


#   def restart_hri(self,json):

#     message_to_VoiceManager=DataToSay()
#     dataJsonView={
#       "order": 0,
#       "action": None
#     }
#     message_to_VoiceManager.json_in_string=js.dumps(dataJsonView)
#     message_to_VoiceManager.data_to_use_in_string=''
#     self.hri.pub_current_view.publish(message_to_VoiceManager)


    

#     self.hri.action_GM_TO_HRI_server.set_preempted()
#     del self.hri
    
#     self.pub_restart.publish("RESTART")
#     time.sleep(1)
#     socketIO.emit('restartHRI',broadcast=True)
#     rospy.loginfo("RESTART HRI")
    
  
#     # self.hri=HRIManager()
    

    

# class Test:
#   def __init__(self):
    

#   def restart(self,json):
#     socketIO.emit('restartHRI',broadcast=True)
#     rospy.logwarn("RESTART HRI")
#     time.sleep(3)
#     self.hri.pub_restart_request.publish("RESTART")
#     # rospy.signal_shutdown("blabla")

if __name__ == '__main__':
  
  
  socketIO = SocketIO('http://127.0.0.1', 5000, LoggingNamespace)
  # from python_depend.views import Views
  hri=HRIManager(socketIO)
  # hri_launcher=HRILauncher()
  # hri = Test()

  # #callback quand user choisit scenario sur tablette
  # socketIO.on('askToChangeScenarioHRIM',hri.chooseScenario)

  # #callback reponse quand scenario choisit et charge
  # socketIO.on('scenarioCharged',hri.scenarioCharged)

  # #callback quand on recoit des donnees de la tablette
  # socketIO.on('indexDataReceivedJS', hri.indexDataJSstepDone)
  # socketIO.on('dataReceivedJS', hri.dataJSstepDone)

  #callback pour revenir au menu principal sur la tablette
  # socketIO.on('resetHRI', hri.restart_hri)

  # #recuperation de l'order de la view chargee par la tablette
  # socketIO.on('launchedView', hri.send_gm_view_launched)

  while not rospy.is_shutdown():
    # hri.pub_restart_mode.publish(hri.restart_mode)
    # rospy.loginfo("RESTART MODE : "+hri.restart_mode)
    rospy.spin()
    # socketIO.wait(seconds=1)

