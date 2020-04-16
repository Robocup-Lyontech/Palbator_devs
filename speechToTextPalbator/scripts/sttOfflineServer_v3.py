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
import pyaudio
import datetime
import time
from recorder import Recorder, RecordingFile
import wave


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

        self.first_launch=True
        
        self.current_directory=os.path.dirname(os.path.realpath(__file__))        
        self.setup_params()

        self.action_server.start()

        rospy.loginfo("server for STT offline initiated")

    def record_audio(self):
        rec = Recorder(channels=1,rate=16000)
        with rec.open(self.current_directory+'/intent.wav', 'wb') as recfile2:
            recfile2.start_recording()
            rospy.loginfo("Speak")
            time.sleep(5.0)
            recfile2.stop_recording()
            rospy.loginfo("End of file")
    
    def setup_params(self):
        # Params

        # File containing language model
        # _lm_param = "~lm"
        # Dictionary
        _dict_param = "~dict"
        # Hidden markov model. Default has been provided below
        _hmm_param = "~hmm"
        # Gram file contains the rules and grammar
        _gram = "~gram"
        # Name of rule within the grammar
        _rule = "~rule"

        _grammar = "~grammar"

        _kwlist = "~kwlist"

        if rospy.has_param(_hmm_param):
            self.hmm = rospy.get_param(_hmm_param)
        else:
            rospy.logerr("No language model specified. Couldn't find default model.")
            return

        if rospy.has_param(_dict_param):
            dict_path=rospy.get_param(_dict_param)
            self.dict=self.current_directory+"/"+dict_path
            rospy.loginfo("Dict path : "+str(dict_path))
        else:
            rospy.logerr(
                "No dictionary found. Please add an appropriate dictionary argument.")
            return

        if rospy.has_param(_kwlist):
            kwlist_path=rospy.get_param(_kwlist)
            self.kwlist=self.current_directory+"/"+kwlist_path
        else:
            rospy.logerr(
                "No keywords list found. Please add an appropriate keywords list argument.")
            return

        if rospy.has_param(_grammar):
            self.grammar=rospy.get_param(_grammar)
        else:
            rospy.logerr(
                "No grammar found. Please add an appropriate grammar along with gram file.")
            return

        if rospy.has_param(_gram) and rospy.get_param(_rule):
            gram_path = rospy.get_param(_gram)
            self.gram = self.current_directory+"/"+gram_path
            self.rule = rospy.get_param(_rule)
        else:
            rospy.logerr(
                "Couldn't find suitable parameters. Please take a look at the documentation")
            return


        self.in_speech_bf = False

        # Setting param values
        

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
        self.first_launch=False
        """Function to handle lm or grammar processing of audio."""
        config = Decoder.default_config()
        rospy.loginfo("Done initializing pocketsphinx")

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
        rospy.loginfo(self.grammar + '.' + self.rule)
        fsg = jsgf.build_fsg(rule, self.decoder.get_logmath(), 7.5)
        rospy.loginfo("Writing fsg to " +
                        self.gram + '.fsg')
        fsg.writefile(self.gram + '.fsg')

        self.decoder.set_fsg(self.gram, fsg)
        self.decoder.set_search(self.gram)

        # Start processing input audio
        self.decoder.start_utt()
        rospy.loginfo("Decoder started successfully")



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


    def process_audio(self):
        """Audio processing based on decoder config."""
        
        while not self.success and not rospy.is_shutdown():
            if self.action_server.is_preempt_requested():
                rospy.loginfo("Preempted request server offline")
                self.action_server.set_preempted()
                break
            rospy.loginfo("Speak Now Please")
            time.sleep(1)
            self.record_audio()
            

            # x=raw_input("Wait for action")
            rospy.loginfo("Audio in process ---------")
            chunk = 1024
            wf = wave.open(self.current_directory+'/intent.wav', 'rb')
            p = pyaudio.PyAudio()
            stream = p.open(format = p.get_format_from_width(wf.getsampwidth()),
                        channels = wf.getnchannels(),
                        rate = wf.getframerate(),
                        output = True)
            data = wf.readframes(chunk)
            while data != '' :
                buf = data
                if buf:
                    self.decoder.process_raw(buf, False, False)
                else:
                    break

                if self.decoder.get_in_speech() != self.in_speech_bf:
                    self.in_speech_bf = self.decoder.get_in_speech()
                    if not self.in_speech_bf:
                        self.decoder.end_utt()
                        if self.decoder.hyp() is not None:
                            rospy.loginfo([(seg.word, seg.prob, seg.start_frame, seg.end_frame) for seg in self.decoder.seg()])
                            rospy.loginfo("--------------------------------------")
                            logmath=self.decoder.get_logmath()
                            rospy.loginfo("Detected %s at %s" % (self.decoder.hyp().hypstr, str(datetime.datetime.now().time())))
                            rospy.loginfo('Score: ' + str(self.decoder.hyp().best_score))
                            rospy.loginfo('Score LOG: ' + str(logmath.exp(self.decoder.hyp().best_score)))
                            rospy.loginfo("--------------------------------------")
                            rospy.loginfo ('Best 10 hypothesis: ')
                            for best, i in zip(self.decoder.nbest(), range(10)):
                                rospy.loginfo ([best.hypstr, best.score])
                        self.decoder.start_utt()
                data = wf.readframes(chunk)
            stream.close()
            x=raw_input("fzefezfzf")
            self.success=True
            self.output=''

    
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
        if self.first_launch==True:
            self.start_recognizer()


        self.action_server_feedback.stt_feedback = 'None'
        self.process_audio()


        if self.success:
            self.action_server.publish_feedback(self.action_server_feedback)
            
            self.action_server_result.stt_result = self.output
            rospy.loginfo('Action STT offline succeeded')
            self.action_server.set_succeeded(self.action_server_result)

if __name__ == "__main__":
    instance_server_offline=SpeechToTextOffline()
    while not rospy.is_shutdown():
        rospy.spin()