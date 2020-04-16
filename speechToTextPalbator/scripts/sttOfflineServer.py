#!/usr/bin/env python

import rospy
import actionlib
import speechToTextPalbator.msg
import os
from std_msgs.msg import String

from pocketsphinx.pocketsphinx import *
from sphinxbase.sphinxbase import *
from statistics import mean 

import json

class SpeechToTextOffline(object):

    def __init__(self):
        rospy.init_node("STT_offline_node")

        self.action_server = actionlib.SimpleActionServer("action_STT_offline",speechToTextPalbator.msg.SttOfflineAction,self.handle_stt_offline,auto_start=False)
        self.action_server_feedback = speechToTextPalbator.msg.SttOfflineFeedback()
        self.action_server_result = speechToTextPalbator.msg.SttOfflineResult()


        self.dictionary_choose=None
        self.current_view_id=None
        self.current_view_action=None
        self.enable_publish_detection_output=None
        
        self.current_directory=os.path.dirname(os.path.realpath(__file__))        
        self.setup_params()

        self.action_server.start()

        rospy.loginfo("server for STT offline initiated")

    
    
    def setup_params(self):
        # Params

        # File containing language model
        _lm_param = "~lm"
        # Dictionary
        _dict_param = "~dict"
        # Hidden markov model. Default has been provided below
        _hmm_param = "~hmm"
        # Gram file contains the rules and grammar
        _gram = "~gram"
        # Name of rule within the grammar
        _rule = "~rule"

        _grammar = "~grammar"

        # check if lm or grammar mode. Default = grammar
        self._use_lm = 0

        self.in_speech_bf = False

        # Setting param values
        if rospy.has_param(_hmm_param):
            self.hmm = rospy.get_param(_hmm_param)
            if rospy.get_param(_hmm_param) == ":default":
                if os.path.isdir("/usr/local/share/pocketsphinx/model"):
                    rospy.loginfo("Loading the default acoustic model")
                    self.hmm = "/usr/local/share/pocketsphinx/model/en-us/en-us"
                    rospy.loginfo("Done loading the default acoustic model")
                else:
                    rospy.logerr(
                        "No language model specified. Couldn't find default model.")
                    return
        else:
            rospy.logerr(
                "No language model specified. Couldn't find default model.")
            return

        if rospy.has_param(_dict_param) and rospy.get_param(_dict_param) != ":default":
            dict_path=rospy.get_param(_dict_param)
            self.dict=self.current_directory+"/"+dict_path
            rospy.loginfo("Dict path : "+str(dict_path))
        else:
            rospy.logerr(
                "No dictionary found. Please add an appropriate dictionary argument.")
            return

        if rospy.has_param(_grammar) and rospy.get_param(_grammar) != ":default":
            pass
        else:
            rospy.logerr(
                "No grammar found. Please add an appropriate grammar along with gram file.")
            return

        if rospy.has_param(_lm_param) and rospy.get_param(_lm_param) != ':default':
            self._use_lm = 1
            lm_path=rospy.get_param(_lm_param)
            self.class_lm=self.current_directory+"/"+lm_path
            rospy.loginfo("LM path : "+str(lm_path))
        elif rospy.has_param(_gram) and rospy.has_param(_rule):
            self._use_lm = 0
            self.gram = rospy.get_param(_gram)
            self.rule = rospy.get_param(_rule)
        else:
            rospy.logerr(
                "Couldn't find suitable parameters. Please take a look at the documentation")
            return

        rospy.loginfo("ASR Module initialized ...")
        # rospy.loginfo("Configuration Set : "+self.dictionary_choose)
        self.FAKE_POSITIVE=None
        # All params satisfied. Starting recognizer
    

    def parser_view_action_to_dic_mode(self,view_action):
        dic_mode=""
        if view_action=="askAge":
            dic_mode="age"
            self.enable_publish_detection_output=False
        
        elif view_action=="askDrink":
            dic_mode="drinks"
            self.enable_publish_detection_output=True
        
        elif view_action=="askSomething":
            self.enable_publish_detection_output=False

        elif view_action=="askName":
            dic_mode="names"
            self.enable_publish_detection_output=True

        elif view_action=="confirm":
            dic_mode="confirmation"
            self.enable_publish_detection_output=True

        elif view_action==None:
            self.enable_publish_detection_output=False
        
        elif view_action=='askOpenDoor':
            dic_mode="next"
            self.enable_publish_detection_output=True

        else:
            self.enable_publish_detection_output=False
        return dic_mode


    def start_recognizer(self):
        """Function to handle lm or grammar processing of audio."""
        config = Decoder.default_config()
        rospy.loginfo("Done initializing pocketsphinx")

        # Setting configuration of decoder using provided params
        config.set_string('-hmm', self.hmm)
        config.set_string('-dict', self.dict)

        # Check if language model to be used or grammar mode
        if self._use_lm:
            rospy.loginfo('Language Model Found.')
            config.set_string('-lm', self.class_lm)
            self.decoder = Decoder(config)
        else:
            rospy.loginfo(
                'language model not found. Using JSGF grammar instead.')
            self.decoder = Decoder(config)

            # Switch to JSGF grammar
            jsgf = Jsgf(self.gram + '.gram')
            rule = jsgf.get_rule(rospy.get_param('~grammar') + '.' + self.rule)
            # Using finite state grammar as mentioned in the rule
            rospy.loginfo(rospy.get_param('~grammar') + '.' + self.rule)
            fsg = jsgf.build_fsg(rule, self.decoder.get_logmath(), 7.5)
            rospy.loginfo("Writing fsg to " +
                          self.gram + '.fsg')
            fsg.writefile(self.gram + '.fsg')

            self.decoder.set_fsg(self.gram, fsg)
            self.decoder.set_search(self.gram)

        # Start processing input audio
        self.decoder.start_utt()
        rospy.loginfo("Decoder started successfully")

        # Subscribe to audio topic
        rospy.Subscriber("jsgf_audio", String, self.process_audio)
        # rospy.spin()

    def load_database(self):
        _database = "~database"
        if rospy.has_param(_database):
            for database_mode,database_path in rospy.get_param(_database).iteritems():
                if database_mode==self.dictionary_choose:
                    file_database=open(self.current_directory+"/"+database_path,"r")
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


    def process_audio(self, data):
        """Audio processing based on decoder config."""
        
        # Check if input audio has ended
        self.FAKE_POSITIVE=None
        if self.enable_publish_detection_output==True:
            self.decoder.process_raw(data.data, False, False)
            if self.decoder.get_in_speech() != self.in_speech_bf:
                self.in_speech_bf = self.decoder.get_in_speech()
                if not self.in_speech_bf:
                    self.decoder.end_utt()
                    if self.decoder.hyp() != None:
                        logmath=self.decoder.get_logmath()
                        rospy.loginfo([(seg.word, logmath.exp(seg.prob))for seg in self.decoder.seg()])
                        rospy.loginfo('OUTPUT: \"' + self.decoder.hyp().hypstr + '\"')
                        rospy.loginfo("DICT PATH  : "+str(self.dict))
                        rospy.loginfo('Score: ' + str(logmath.exp(self.decoder.hyp().best_score)))
                        self.score_detection=logmath.exp(self.decoder.hyp().best_score)
                        rospy.loginfo('Proba : '+str(logmath.exp(self.decoder.hyp().prob)))
                        self.output=self.decoder.hyp().hypstr
                        
                        self.FAKE_POSITIVE=True
                        for object in self.database_words:
                            # if type(object) is list:
                            if object[0]==self.output:
                                self.FAKE_POSITIVE=False
                                detection_seuil=object[1]
                                rospy.loginfo("------------------GOOD DETECTION----------------")
                                break
                                    
                            # else:
                            #     if object==self.output:
                            #         self.FAKE_POSITIVE=False
                            #         rospy.loginfo("------------------GOOD DETECTION----------------")
                            #         break
                        
                        if self.FAKE_POSITIVE==False and self.score_detection>=detection_seuil:
                            rospy.logwarn("YOUHOUUUUUUU: DETECTED WORD "+self.output)
                            self.enable_publish_detection_output=False
                            self.success=True
                        elif self.FAKE_POSITIVE==True:
                            rospy.logwarn("WORD NOT IN DICT: "+self.output)
                        
                        elif self.score_detection<detection_seuil:
                            rospy.logwarn("SEUIL TROP FAIBLE: "+str(round(self.score_detection,4))+" AU LIEU DE "+str(round(detection_seuil,4)))

                    self.decoder.start_utt()
        else:
            rospy.logwarn("Not allowed to publish detection")
    
    def handle_stt_offline(self,goal):
        rospy.loginfo("Action initiating ...")
        self.success=False

        d=json.loads(goal.order)
        view_id=d['order']

        view_action=d['action']
        self.current_view_action=view_action
        self.dictionary_choose=self.parser_view_action_to_dic_mode(view_action)
        if self.dictionary_choose != "":
            self.load_database()
            rospy.loginfo("Load new database config ...")

        self.enable_publish_detection_output=True
        self.start_recognizer()


        self.action_server_feedback.stt_feedback = 'None'
        while not self.success and not rospy.is_shutdown():
            if self.action_server.is_preempt_requested():
                rospy.loginfo("Preempted request server offline")
                self.action_server.set_preempted()
                break

        if self.success:
            self.action_server.publish_feedback(self.action_server_feedback)
            
            self.action_server_result.stt_result = self.output
            rospy.loginfo('Action STT offline succeeded')
            self.action_server.set_succeeded(self.action_server_result)

if __name__ == "__main__":
    instance_server_offline=SpeechToTextOffline()
    while not rospy.is_shutdown():
        rospy.spin()