#!/usr/bin/env python


from socketIO_client import SocketIO, LoggingNamespace
import json as js

from std_msgs.msg import String
from HriManager.msg import DataToSay
import rospy
import os 
import actionlib
from HriManager.msg import GmToHriAction, GmToHriFeedback, GmToHriResult
import time


class HRIManager:
  # def load_scenario_json(self,json):
  #   name_scenario_unicode = json['scenario']
  #   name_scenario = str(name_scenario_unicode)
  #   print(name_scenario,type(name_scenario))
  #   path_scenario_json = '/Users/abdeljabarferdjani/Documents/Robocup/robocup_palbator-hri_python/templates/public/json/' +name_scenario+'/scenario.json'
  #   print(type(path_scenario_json))
  #   print('nom du path',path_scenario_json)
  #   with open(path_scenario_json) as scenar:
  #     print(scenar)
      # chargedScenario = json.load(scenar) ## essayer avec loads
      # print(chargedScenario)
    # return chargedScenario

  def __init__(self):
    rospy.init_node("hri_manager_node",anonymous=True)
    self.pub_current_view=rospy.Publisher("CurrentView",DataToSay,queue_size=10)
    self.subscriber=rospy.Subscriber("SendData",String,self.handle_data)

    self.pub_choice_scenario=rospy.Publisher("choice_scenario",String,queue_size=10)
    self.pub_event_TM=rospy.Publisher("event_on_TM",String,queue_size=1)
    self.pub_event_VM=rospy.Publisher("event_on_VM",String,queue_size=1)
    
    dir_path = os.path.dirname(os.path.realpath(__file__))
    ######### SCENARIO JSON LOAD #############

    with open(dir_path+'/templates/public/json/present_school_test/scenario.json') as pres_school:
      self.presentSchool = js.load(pres_school)
      self.presentSchool['name']='present_school'

    with open(dir_path+'/templates/public/json/creation_test/scenario.json') as creationTest:
        self.creation = js.load(creationTest)
        self.creation['name']='creation_test'

    with open(dir_path+'/templates/public/json/inspection/scenario.json') as inspec:
        self.inspection = js.load(inspec)
        self.inspection['name']='inspection'

    with open(dir_path+'/templates/public/json/take_out_the_garbage/scenario.json') as take_garbage:
        self.takeOutGarbage = js.load(take_garbage)
        self.takeOutGarbage['name']='take_out_the_garbage'

    with open(dir_path+'/templates/public/json/receptionist/scenario.json') as recep:
        self.receptionist = js.load(recep)
        self.receptionist['name']='receptionist'


    self.finalStep = None
    self.nameToUse = []
    self.drinkToUse = []
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

    self.action_GM_TO_HRI_server=actionlib.SimpleActionServer("action_GmToHri",GmToHriAction,self.action_GmToHri_callback,auto_start=False)
    self.action_GM_TO_HRI_feedback=GmToHriFeedback()
    self.action_GM_TO_HRI_result=GmToHriResult()
    self.action_GM_TO_HRI_server.start()

    rospy.loginfo('HRI MANAGER LAUNCHED')


  def action_GmToHri_callback(self,goal):
    rospy.loginfo("Action initiating ...")

    success=True
    self.action_GM_TO_HRI_feedback.Gm_To_Hri_feedback='All clear'
    rospy.loginfo("GOAL RECEIVED: "+goal.json_request)

    time.sleep(2)


    ##################"
    for i in range(0,50):
      if self.action_GM_TO_HRI_server.is_preempt_requested():
        rospy.loginfo("Preempted GM TO HRI Action")
        self.action_GM_TO_HRI_feedback.Gm_To_Hri_feedback='Action preempted by client'
        self.action_GM_TO_HRI_server.set_preempted()
        success=False
        break
      rospy.loginfo("HELLOOOOOOOOOOOOOOOOOO")
      time.sleep(1)

    # ##################"

    self.action_GM_TO_HRI_server.publish_feedback(self.action_GM_TO_HRI_feedback)
    if success:
      self.action_GM_TO_HRI_result.Gm_To_Hri_output=self.action_GM_TO_HRI_feedback.Gm_To_Hri_feedback
      rospy.loginfo("Action GM TO HRI succeeded")
      self.action_GM_TO_HRI_server.set_succeeded(self.action_GM_TO_HRI_result)


  ##########
  # On a le current step en json et son index afin de start la vue approprie
  # Si le step n a pas d action, c est que c est le titre d une etape.
  # Dans ce cas la, on update l etat de currentStep et on lance le timer
  # Si le step a une action, on lance la vue
  # A l interieur de la vue on envoie au FLASK la vue a lance et les attributs
  def stepToStart(self,json,index,dataToUse):
    step = json
    # rospy.loginfo("ETAPE A DEMARRER :"+step['name'])
    # print('---------- STEP DANS STEPTOSTART ------',step)
    # global indexFailure
    # global currentAction
    # rospy.loginfo(step)
    self.currentAction = step['action']
    if(step['action'] == 'confirm' and 'indexFailure' in step): 
      self.indexFailure = step['indexFailure']
    if step['action'] != '':
      self.nameAction = step['name']
      Views.start(self,step['action'],step, index, dataToUse)
      rospy.loginfo("CHARGEMENT VUE SOUS ETAPE:"+step['name'] +" SUR TABLETTE")
      messageDataToSay=DataToSay()
      messageDataToSay.json_in_string=js.dumps(step)
      if not dataToUse is None:
        if "drink" in self.nameAction and not "Confirm" in self.nameAction:
          messageDataToSay.data_to_use_in_string=str(self.nameToUse[-1])
        else:
          messageDataToSay.data_to_use_in_string=str(dataToUse)
      else:
        messageDataToSay.data_to_use_in_string=''
      self.pub_current_view.publish(messageDataToSay)
      # rospy.loginfo("PUBLICATION DONNEES CURRENT VIEW A VOICE MANAGER : "+str(messageDataToSay))
    else:
      rospy.loginfo("ETAPE COURANTE A DEMARRER: "+step['name'])
      dataJsonToSendCurrentStep = {
          "index": index,
          "step":step
      }
      socketIO.emit('stepCurrent',dataJsonToSendCurrentStep,broadcast=True)
      if step['id'] != 'FinishScenario':
        dataJsonToSendTimer = {
            "state": 'TOGGLE_TIMER/ON'
        }
        socketIO.emit('startTimer',dataJsonToSendTimer, broadcast=True)
      rospy.loginfo("ETAPE DEMARREE: "+step['name'])
      self.updateNextStep(index)

##################################### DATA RECEIVED #################################################

  def handle_data(self,req):
    data=req.data
    json_received=js.loads(data)
    if (self.data_received is False and self.index == json_received['index']):
      rospy.loginfo('CHARGEMENT DONNEE DE VOICE MANAGER')
      # rospy.loginfo('INDEX HRI MANAGER %s',self.index)
      # rospy.loginfo('INDEX VOICE MANAGER %s',json_received['index'])
      rospy.loginfo('DONNEE RECUE DEPUIS VOICE MANAGER %s',json_received['dataToUse'])
      self.data_received=True
      if(self.currentAction != 'confirm'):
        self.dataToUse = json_received['dataToUse']
      if(json_received['dataToUse'] != 'false'):
        self.updateNextStep(self.index)
      else:
        self.updatePreviousStep(self.index)
      self.data_received=False
    else:
      rospy.logwarn('Le voice Manager envoie une donnee a la mauvaise etape: '+str(json_received['index'])+' au lieu de '+str(self.index))

  ######### On recoit les donnees que l utilisateur a entre et on appelle la fonction updateNextStep
  ######### La fonction updateNextStep change l index sur lequel currentStep est et met donc a jour le currentStep

  def indexDataJSstepDone(self,json):
    if('data' in json):
      self.currentIndexDataReceivedJS = json['data']


  def dataJSstepDone(self,json):
    
    if (self.data_received is False and self.index == self.currentIndexDataReceivedJS):
      rospy.loginfo("DONNEE RECUE DEPUIS TOUCH MANAGER")
      self.data_received=True
      if(self.currentAction != 'confirm'):
        self.dataToUse = json['data']
        rospy.loginfo("DONNEE TOUCH MANAGER: "+str(self.dataToUse))
        if('name' in self.nameAction):
          self.choosenName = self.dataToUse
        if('drink' in self.nameAction):
          self.choosenDrink = self.dataToUse
        if('age' in self.nameAction):
          self.choosenAge = self.dataToUse
          self.ageToUse.append(self.choosenAge)
        self.updateNextStep(self.index)
      else:
        rospy.loginfo("DONNEE TOUCH MANAGER: "+str(json['data']))
        if(json['data'] != 'false'):
          if('name' in self.nameAction):
            self.nameToUse.append(self.choosenName)
          if('drink' in self.nameAction):
            self.drinkToUse.append(self.choosenDrink)
          # if('age' in self.nameAction):
          #   self.ageToUse.append(self.choosenAge)
          self.updateNextStep(self.index)
        else:
          self.updatePreviousStep(self.index)
    else:
      rospy.logwarn('Le touch Manager envoie une donnee a la mauvaise etape: '+str(self.currentIndexDataReceivedJS)+' au lieu de '+str(self.index))

    self.data_received=False
      # print('la liste des noms',self.nameToUse)
      # print('la liste des boissons',self.drinkToUse)
      # print('la liste des ages',self.ageToUse)
  ###########################


  ####### On a en parametre, un json provenant du REACT qui contient la liste des steps ordonnees ainsi que le nom du scenario #########
  ####### On initialise le currentStep afin qu il ai le premier step de la liste
  ####### Le currentStep pointe sur une case du tableau de step, cette case varie en fonction de index grace a la fonction updateNextStep
  ####### On fait une boucle. Elle ne s arrete pas tant que le currentStep ne vaut pas le finalStep
  ####### On start le currentStep avec la fonction stepToStart
  def updateCurrentStep(self,json):
    # socketIO.wait(seconds=1)
    lastStep = None
    # global index
    # global currentStep
    # global dataToUse
    # global indexStepCompleted
    # global restart
    self.restart = 0
    if json['data'] != []:
      self.finalStep = json['data'][len(json['data']) - 1]
      self.currentStep = json['data'][self.index]
      while self.currentStep != self.finalStep and self.restart == 0:
        ############# STEP COMPLETED PUSH FOR RECEPTIONIST, on check si on arrive a la fin de l etape et on envoie
        ############# au REACT l index de l etape finie
        if self.currentStep['action'] == '' and self.currentStep['order'] != 0:
          stepCompletedJson = {"idSteps": self.indexStepCompleted}
          socketIO.emit('CompleteStep',stepCompletedJson,broadcast=True)
          rospy.loginfo("ETAPE TERMINEE: "+str(self.currentStep['name']))
          self.indexStepCompleted = self.currentStep['order']

        if (self.currentStep != None and self.currentStep != lastStep):
          self.stepToStart(self.currentStep,self.index,self.dataToUse)
          lastStep=self.currentStep
        socketIO.wait(seconds=0.1)
        self.currentStep = json['data'][self.index]
    self.restart_hri({""})
  
  def updateNextStep(self,indexForNextStep):
    # global index
    self.index =indexForNextStep+1

  def updatePreviousStep(self,indexForPreviousStep):
    # global index
    # global indexFailure
    if(self.indexFailure != None):
      self.index =self.indexFailure
    else:
      self.index=indexForPreviousStep-1


  ########################## Chargement du scenario selectionne ################


  ######### On recoit depuis le REACT le nom du scenario selectionne et on charge le json approprie pour lui renvoyer ############
  def chargeScenario(self,json):
      # global dataJson
      ####
      # dataJson = self.load_scenario_json(json)
      # print(dataJson)
      ####
      # global socketIO
      choices = {self.receptionist['name']: self.receptionist, self.takeOutGarbage['name']: self.takeOutGarbage, self.inspection['name']: self.inspection, self.presentSchool['name']: self.presentSchool, self.creation['name']: self.creation}
      dataJson = choices.get(json['scenario'], 'default')
      dataJsonToSendScenario = {
        "scenario": dataJson['name'],
        "stepsList": dataJson['steps']
      }
      socketIO.emit('scenarioToCharged',dataJsonToSendScenario, broadcast=True)
      rospy.loginfo("SCENARIO CHARGE: "+str(dataJson['name']))
      self.pub_choice_scenario.publish(str(dataJson['name']))



  def restart_hri(self,json):

    # rospy.logwarn("JSON RESTART :j"+str(json))
    # global finalStep
    self.finalStep = None
    # global currentStep
    self.currentStep = None
    # global currentAction
    self.currentAction = None
    # global index
    self.index=0
    # global indexStepCompleted
    self.indexStepCompleted=0
    # global dataToUse
    self.dataToUse=''
    # global indexFailure
    self.indexFailure=None
    # global restart
    self.restart = 1

    message_to_VoiceManager=DataToSay()
    dataJsonView={
      "order": self.index,
      "action": self.currentAction
    }
    message_to_VoiceManager.json_in_string=js.dumps(dataJsonView)
    message_to_VoiceManager.data_to_use_in_string=''
    self.pub_current_view.publish(message_to_VoiceManager)
    socketIO.emit('restartHRI',broadcast=True)
    rospy.loginfo("RESTART HRI")

  #############


  

if __name__ == '__main__':
  
  socketIO = SocketIO('http://127.0.0.1', 5000, LoggingNamespace)
  from python_depend.views import Views

  hri = HRIManager()
  socketIO.on('askToChangeScenarioHRIM', hri.chargeScenario)
  socketIO.on('scenarioCharged', hri.updateCurrentStep)
  # socketIO.on('NextStep', hri.updateNextStep)
  socketIO.on('indexDataReceivedJS', hri.indexDataJSstepDone)
  socketIO.on('dataReceivedJS', hri.dataJSstepDone)
  socketIO.on('resetHRI', hri.restart_hri)

  while not rospy.is_shutdown():
    socketIO.wait(seconds=1)

