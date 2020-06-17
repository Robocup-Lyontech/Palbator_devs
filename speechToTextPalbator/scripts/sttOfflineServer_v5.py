#!/usr/bin/env python

import rospy
import actionlib
import speechToTextPalbator.msg
import os
import shutil
from std_msgs.msg import String

from pocketsphinx.pocketsphinx import *
from sphinxbase.sphinxbase import *
from statistics import mean 

import json
import pyaudio
import datetime
import time
from recorder_v2 import RecorderForSpeechRecog
import wave
import ttsMimic.msg


# import spacy
# from spacy.lang.en import English
# from spacy.lang.en.stop_words import STOP_WORDS

class SpeechToTextOffline(object):

    def __init__(self):

        rospy.init_node("STT_offline_node")
        if rospy.has_param("~stt_offline_server_name"):
            self.action_server = actionlib.SimpleActionServer(rospy.get_param("~stt_offline_server_name"),speechToTextPalbator.msg.SttOfflineAction,self.handle_stt_offline,auto_start=False)
            self.action_server_feedback = speechToTextPalbator.msg.SttOfflineFeedback()
            self.action_server_result = speechToTextPalbator.msg.SttOfflineResult()
        else:
            rospy.logerr("{class_name} : No name specified for offline STT ActionServer".format(class_name=self.__class__.__name__))

        self.old_scenario = None
        self.current_scenario = None

        self.dictionary_choose=None
        self.current_view_action=None
        self.enable_publish_detection_output=None

        self.first_launch=True
        
        self.current_directory=os.path.dirname(os.path.realpath(__file__))      

        
        
        if rospy.has_param('~config_recorder'):
            self.config_recorder = rospy.get_param("~config_recorder")
        else:
            rospy.logerr("{class_name} : No configuration specified for audio recorder".format(class_name=self.__class__.__name__))

        ##### TEST RECORDER AUDIO ######
        self.rec = RecorderForSpeechRecog(self.config_recorder)

        self.client_TTS=actionlib.SimpleActionClient(self.config_recorder['tts_service'],ttsMimic.msg.TtsMimicAction)
        rospy.loginfo("{class_name} : Waiting for TTS server ...".format(class_name=self.__class__.__name__))
        self.client_TTS.wait_for_server()
        rospy.loginfo("{class_name} : TTS server connected".format(class_name=self.__class__.__name__))

        
        
        self.audio_dir = self.config_recorder['audio_record_dir']   

        self.reset_record_folder()

        if rospy.has_param('~config'):
            self.config = rospy.get_param('~config')
        else:
            rospy.logerr("{class_name} : No configuration specified for Speech Recognition Server".format(class_name=self.__class__.__name__)) 


        if rospy.has_param("~parser_action_database"):
            self.parser_action_database = rospy.get_param("~parser_action_database")     
        else:
            rospy.logerr("{class_name} : No configuration specified for parser action database".format(class_name=self.__class__.__name__)) 
       

        self.action_server.start()
        rospy.on_shutdown(self.shutdown)
        rospy.loginfo("{class_name} : server for STT offline initiated".format(class_name=self.__class__.__name__))

    def tts_action(self,speech):
        """
        This function will use the TTS ROSAction to say the speech.

        :param speech: Speech to say
        :type speech: string
        """
        self.goal_TTS=ttsMimic.msg.TtsMimicGoal(speech)
        self.client_TTS.send_goal(self.goal_TTS)
        self.client_TTS.wait_for_result()
    
    def reset_record_folder(self):
        file_path = ''
        try:
            folder = os.path.join(self.current_directory,self.audio_dir)
            for filename in os.listdir(folder):
                file_path = os.path.join(folder, filename)
                if os.path.isfile(file_path) or os.path.islink(file_path):
                    os.unlink(file_path)
                elif os.path.isdir(file_path):
                    shutil.rmtree(file_path)

        except Exception as e:
            rospy.logwarn('{class_name} : Failed to delete %s. Reason: %s'.format(class_name=self.__class__.__name__),file_path, e)
            if not os.path.exists(folder):
                os.makedirs(folder)
        
    def shutdown(self):
        """This function is executed on node shutdown."""
        # command executed after Ctrl+C is pressed
        rospy.loginfo("{class_name} : Stop ASRControl".format(class_name=self.__class__.__name__))
        self.action_server.set_aborted()

    def record_audio(self):
        """
        This function will record a audio file and save it to a location specified in configuration file.
        """

        self.rec.record_audio()
    

    def setup_params(self):
        """
        This function will setup the necessary parameters (dictionary, keywords list, grammar etc..) according to the current scenario.
        """
        self.config_scenario = self.config[self.current_scenario]

        _dict_param = "dict"
        _hmm_param = "~hmm"
        _gram = "gram"
        _rule = "rule"
        _grammar = "grammar"
        _kwlist = "kwlist"

        if rospy.has_param(_hmm_param):
            self.hmm = rospy.get_param(_hmm_param)
        else:
            rospy.logerr("{class_name} : No language model specified. Couldn't find default model.".format(class_name=self.__class__.__name__))
            return

        if _dict_param in self.config_scenario:
            dict_path = self.config_scenario[_dict_param]
            self.dict=os.path.join(self.current_directory,dict_path)
            rospy.loginfo("{class_name} : Dict path : ".format(class_name=self.__class__.__name__)+str(dict_path))
        else:
            rospy.logerr("{class_name} : No dictionary found. Please add an appropriate dictionary argument.".format(class_name=self.__class__.__name__))
            return


        if _kwlist in self.config_scenario:
            kwlist_path=self.config_scenario[_kwlist]
            self.kwlist=os.path.join(self.current_directory,kwlist_path)
        else:
            rospy.logerr("{class_name} : No keywords list found. Please add an appropriate keywords list argument.".format(class_name=self.__class__.__name__))
            return

        if _grammar in self.config_scenario:
            self.grammar = self.config_scenario[_grammar]
        else:
            rospy.logerr("{class_name} : No grammar found. Please add an appropriate grammar along with gram file.".format(class_name=self.__class__.__name__))
            return

        if _gram in self.config_scenario and _rule in self.config_scenario:
            gram_path = self.config_scenario[_gram]
            self.gram = os.path.join(self.current_directory,gram_path)
            self.rule = self.config_scenario[_rule]
        else:
            rospy.logerr("{class_name} : Couldn't find suitable parameters for gram and rule. Please take a look at the documentation".format(class_name=self.__class__.__name__))
            return

        self.in_speech_bf = False
        rospy.loginfo("{class_name} : Parameters initialized for scenario ".format(class_name=self.__class__.__name__)+self.current_scenario)
        self.FAKE_POSITIVE=None
    
    def parser_view_action_to_dic_mode(self,view_action):
        """
        This function will get the database name associated to the view action name.

        :param view_action: Action name of the current view
        :type view_action: string
        """
        dic_mode=""

        if view_action in self.parser_action_database[self.current_scenario]:
            dic_mode = self.parser_action_database[self.current_scenario][view_action]
            rospy.logwarn("{class_name} : DIC MODE ".format(class_name=self.__class__.__name__)+str(dic_mode))
            self.enable_publish_detection_output=True
        else:
            rospy.logwarn("{class_name} : Detection is not allowed for this action".format(class_name=self.__class__.__name__))
            self.enable_publish_detection_output=False
        
        return dic_mode

    def start_recognizer(self):
        """
        This function will setup the decoder with all the parameters defined in the function setup_params.
        """
        self.setup_params()
        self.first_launch=False
        
        config = Decoder.default_config()
        rospy.loginfo("{class_name} : Done initializing pocketsphinx".format(class_name=self.__class__.__name__))

        # Setting configuration of decoder using provided params
        config.set_string('-hmm', self.hmm)
        config.set_string('-dict', self.dict)
        config.set_string('-kws',self.kwlist)

        # Check if language model to be used or grammar mode
        self.decoder = Decoder(config)

        # Switch to JSGF grammar
        jsgf = Jsgf(self.gram + '.gram')
        rule = jsgf.get_rule(self.grammar + '.' + self.rule)
        # Using finite state grammar as mentioned in the rule
        rospy.loginfo("{class_name} : ".format(class_name=self.__class__.__name__) + self.grammar + '.' + self.rule)
        fsg = jsgf.build_fsg(rule, self.decoder.get_logmath(), 7.5)
        rospy.loginfo("{class_name} : Writing fsg to ".format(class_name=self.__class__.__name__) + self.gram + '.fsg')
        fsg.writefile(self.gram + '.fsg')

        self.decoder.set_fsg(self.gram, fsg)
        self.decoder.set_search(self.gram)

        rospy.loginfo("{class_name} : Decoder started successfully".format(class_name=self.__class__.__name__))


    def load_database(self):
        """
        This function will load the verification database associated to the current scenario and view.
        """
        _database = "database"
        if _database in self.config_scenario:
            if self.dictionary_choose in self.config_scenario[_database]:
                database_path = self.config_scenario[_database][self.dictionary_choose] 
                file_database=open(os.path.join(self.current_directory,database_path),"r")
                self.database_words=[]
                
                for line in file_database:
                    data=line.split("\n")[0]
                    data2=data.split("\t")
                    if len(data2)>1:
                        self.database_words.append([data2[0],float(data2[1])])
                    else:
                        self.database_words.append(data2[0])
                file_database.close()
                print(self.database_words)
            else:
                rospy.logwarn("{class_name} : Missing verification database for this action.".format(class_name=self.__class__.__name__))
                self.enable_publish_detection_output = False
        else:
            rospy.logerr("{class_name} : No verification database specified.".format(class_name=self.__class__.__name__))

    def process_audio(self):
        """
        This function will initiate the record of an audio file and process it to detect the speech content.
        """
        
        while not self.success and not rospy.is_shutdown():
            if self.action_server.is_preempt_requested():
                rospy.loginfo("{class_name} : Preempted request server offline".format(class_name=self.__class__.__name__))
                self.action_server.set_preempted()
                break

            self.record_audio()

            if self.rec.audio_correctly_recorded:
                rospy.loginfo("{class_name} : Audio Reading in process ---------".format(class_name=self.__class__.__name__))
                self.decoder.start_utt()
                cp=0
                for file in os.listdir(os.path.join(self.current_directory,self.audio_dir)):
                    if file.endswith(".wav"):
                        cp = cp+1

                chunk = 1024
                filename = os.path.join(self.current_directory,self.audio_dir)+'/intent'+str(cp)+'.wav'
                wf = wave.open(filename, 'rb')
                print("{class_name} : Loading file : ".format(class_name=self.__class__.__name__)+str(filename))
                p = pyaudio.PyAudio()
                stream = p.open(format = p.get_format_from_width(wf.getsampwidth()),
                            channels = wf.getnchannels(),
                            rate = wf.getframerate(),
                            output = True)
                data = wf.readframes(chunk)

                list_detection=[]
                
                while data != '':
                    buf=data
                    if buf:
                        self.decoder.process_raw(buf, False, False)
                    else:
                        break

                    if self.decoder.get_in_speech() != self.in_speech_bf:
                        self.in_speech_bf = self.decoder.get_in_speech()
                        if not self.in_speech_bf:
                            self.decoder.end_utt()
                            if self.decoder.hyp() is not None:
                                print([(seg.word, float(seg.prob), seg.start_frame, seg.end_frame) for seg in self.decoder.seg()])
                                print("--------------------------------------")
                                print("{class_name} : ".format(class_name=self.__class__.__name__) + "Detected %s at %s" % (self.decoder.hyp().hypstr, str(datetime.datetime.now().time())))
                                print("--------------------------------------")
                                print('{class_name} : Score: '.format(class_name=self.__class__.__name__) + str(self.decoder.hyp().best_score))
                                logmath=self.decoder.get_logmath()
                                print('{class_name} : Score LOG: '.format(class_name=self.__class__.__name__) + str(logmath.exp(self.decoder.hyp().best_score)))
                                list_detection.append(self.decoder.hyp().hypstr)

                            self.decoder.start_utt()
                    data = wf.readframes(chunk)
                stream.close()
                self.decoder.end_utt()


                
                for item in list_detection:
                    if self.dictionary_choose == "age":
                        if " " in item:
                            list_in_item = item.split(" ")
                            for cut in list_in_item:
                                try:
                                    test = int(cut)
                                    self.success=True
                                    self.output=cut
                                    rospy.loginfo("{class_name} : GOOD DETECTION!!!!!".format(class_name=self.__class__.__name__))
                                    break
                                except ValueError:
                                    pass
                        else:
                            try:
                                test = int(item)
                                self.success=True
                                self.output=item
                                rospy.loginfo("{class_name} : GOOD DETECTION!!!!!".format(class_name=self.__class__.__name__))
                                break
                            except ValueError:
                                pass
                    else:
                        for element in self.database_words:
                            if element in item:
                                self.success=True
                                self.output=element
                                rospy.loginfo("{class_name} : GOOD DETECTION!!!!!".format(class_name=self.__class__.__name__))
                                break

                if not self.success:
                    rospy.logwarn("{class_name} : Uncorrect detection. Please try again".format(class_name=self.__class__.__name__))
                    self.tts_action("I didn't understand what you meant. Please try again.")


    
    def handle_stt_offline(self,goal):
        """
        This function will start when an ActionGoal is received from the ActionClient.

        :param goal: goal received from ActionClient.
        :type goal: string
        """

        rospy.loginfo("{class_name} : Action Offline initiating ...".format(class_name=self.__class__.__name__))
        self.success=False

        d=json.loads(goal.order)
        view_id=d['order']
        view_action=d['action']
        self.current_scenario = d['scenario']

        rospy.logwarn("{class_name} : CURRENT SCENARIO : ".format(class_name=self.__class__.__name__)+str(self.current_scenario))

        if self.old_scenario is None:
            self.old_scenario = d['scenario']


        if self.old_scenario is None:
            rospy.logwarn("{class_name} : No scenario specified. Preempting action ...".format(class_name=self.__class__.__name__))
            self.action_server.set_preempted()


        if self.first_launch==True or self.old_scenario != self.current_scenario:
            rospy.logwarn("{class_name} : NEED NEW SETUP : CHANGE SCENARIO".format(class_name=self.__class__.__name__))
            self.old_scenario=self.current_scenario
            self.start_recognizer()

        self.current_view_action=view_action
        self.dictionary_choose=self.parser_view_action_to_dic_mode(view_action)
        if self.dictionary_choose != "":
            self.load_database()
            rospy.loginfo("{class_name} : Load new database config ...".format(class_name=self.__class__.__name__))


        self.action_server_feedback.stt_feedback = 'None'
        if self.enable_publish_detection_output == True:
            self.process_audio()
        else:
            rospy.logwarn("{class_name} : Detection not allowed. Preempting action ...".format(class_name=self.__class__.__name__))
            self.action_server.set_preempted()
                
        if self.success:
            self.action_server.publish_feedback(self.action_server_feedback)
            
            self.action_server_result.stt_result = self.output
            rospy.loginfo('{class_name} : Action STT offline succeeded'.format(class_name=self.__class__.__name__))
            self.action_server.set_succeeded(self.action_server_result)

if __name__ == "__main__":
    instance_server_offline=SpeechToTextOffline()
    while not rospy.is_shutdown():
        rospy.spin()