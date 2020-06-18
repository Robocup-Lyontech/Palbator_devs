#!/usr/bin/env python

import rospy
import depend.speech_recognition as sr
from threading import Thread
from socketIO_client import SocketIO, LoggingNamespace
import ttsMimic.msg
import json
import os
import actionlib

from speechToTextPalbator.msg import SpeechRecognitionAction, SpeechRecognitionResult
from pocketsphinx.pocketsphinx import *
from sphinxbase.sphinxbase import *
import pyaudio
import wave
import datetime



class AudioThread(Thread):
    def __init__(self,config_recorder, config_decoder, parser_action_database, goal, socketIO):
        Thread.__init__(self)

        self.recognizer = sr.Recognizer()
        self.micro = sr.Microphone()

        self.config_recorder = config_recorder
        self.config_decoder = config_decoder
        self.parser_action_database = parser_action_database
        self.goal = goal

        self.flag_stop = False
        self.json_response = None

        self.current_scenario = self.goal['scenario']
        self.view_action = self.goal['action']

        self.current_directory = os.path.dirname(os.path.realpath(__file__))

        self.audio_dir = self.config_recorder['audio_record_dir']

        self.success = False

        self.socketIO = socketIO

    def setup_params_for_offline(self):
        """
        This function will setup the necessary parameters (dictionary, keywords list, grammar etc..) according to the current scenario.
        """
        self.config_scenario = self.config_decoder[self.current_scenario]

        _dict_param = "dict"
        _hmm_param = "hmm"
        _gram = "gram"
        _rule = "rule"
        _grammar = "grammar"
        _kwlist = "kwlist"

        if _hmm_param in self.config_scenario:
            language_model_path = self.config_scenario[_hmm_param]
            self.hmm = os.path.join(self.current_directory,language_model_path)
            rospy.loginfo("{class_name} : Language model path : %s".format(class_name=self.__class__.__name__),self.hmm)
        else:
            rospy.logerr("{class_name} : No language model specified. Couldn't find default model.".format(class_name=self.__class__.__name__))
            return

        if _dict_param in self.config_scenario:
            dict_path = self.config_scenario[_dict_param]
            self.dict=os.path.join(self.current_directory,dict_path)
            rospy.loginfo("{class_name} : Dict path : %s".format(class_name=self.__class__.__name__),str(dict_path))
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
        rospy.loginfo("{class_name} : PARSING VIEW ACTION TO GET DICT MODE".format(class_name=self.__class__.__name__))

        dic_mode=""

        if view_action in self.parser_action_database[self.current_scenario]:
            dic_mode = self.parser_action_database[self.current_scenario][view_action]
            rospy.logwarn("{class_name} : DIC MODE ".format(class_name=self.__class__.__name__)+str(dic_mode))
            self.enable_detection=True
        else:
            rospy.logwarn("{class_name} : Detection is not allowed for this action".format(class_name=self.__class__.__name__))
            self.enable_detection=False
        
        return dic_mode


    def load_database(self):
        """
        This function will load the verification database associated to the current scenario and the current view.
        """
        rospy.loginfo("{class_name} : LOADING VOCAB DATABASE".format(class_name=self.__class__.__name__))

        self.config_scenario = self.config_decoder[self.current_scenario]
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
                self.enable_detection = False
        else:
            rospy.logerr("{class_name} : No verification database specified.".format(class_name=self.__class__.__name__))

    
    
    def process_detection_online(self,transcription):
        """
        This function will process the detection received from the recognizer.

        :param transcription: detection result of the recognizer
        :type transcription: string
        """
        transcription=transcription.upper()
        data = ''

        if self.dictionary_choose == "age":
            if not " " in transcription:
                try:
                    new_transcription = int(transcription)
                    for element in self.database_words:
                        if transcription == element:
                            data=element
                            rospy.loginfo("{class_name} : GOOD DETECTION".format(class_name=self.__class__.__name__))
                            break

                except ValueError:
                    pass

            else:
                new_transcription_cut = transcription.split(" ")
                for item in new_transcription_cut:
                    try:
                        test = int(item)
                        for element in self.database_words:
                            if item == element:
                                data=element
                                rospy.loginfo("{class_name} : GOOD DETECTION".format(class_name=self.__class__.__name__))
                                break

                    except ValueError:
                        pass

        else:
            for element in self.database_words:
                if element in transcription:
                    data=element
                    rospy.loginfo("{class_name} : GOOD DETECTION".format(class_name=self.__class__.__name__))
                    break

        if data == '':
            rospy.loginfo("{class_name} : BAD DETECTION. Please try again".format(class_name=self.__class__.__name__))
        
        return data

    def process_detection_offline(self,list_detection):
        output = ''
        for item in list_detection:
            if self.dictionary_choose == "age":
                if " " in item:
                    list_in_item = item.split(" ")
                    for cut in list_in_item:
                        try:
                            test = int(cut)
                            output=cut
                            rospy.loginfo("{class_name} : GOOD DETECTION!!!!!".format(class_name=self.__class__.__name__))
                            break
                        except ValueError:
                            pass
                else:
                    try:
                        test = int(item)
                        output=item
                        rospy.loginfo("{class_name} : GOOD DETECTION!!!!!".format(class_name=self.__class__.__name__))
                        break
                    except ValueError:
                        pass
            else:
                for element in self.database_words:
                    if element in item:
                        output=element
                        rospy.loginfo("{class_name} : GOOD DETECTION!!!!!".format(class_name=self.__class__.__name__))
                        break
        return output

    def run(self):

        if self.goal['connection_state'] == 'Offline':

            self.setup_params_for_offline()
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


        self.dictionary_choose=self.parser_view_action_to_dic_mode(self.view_action)
        self.load_database()

        if self.enable_detection:

            ##### RECORDING #######
            with self.micro as source:
                json_mic ={
                    "hide": False
                }
                self.socketIO.emit("hideMic",json_mic,broadcast=True)
                self.recognizer.adjust_for_ambient_noise(source,duration=self.config_recorder['ajusting_ambient_noise_duration'])
                rospy.loginfo("{class_name} : Starts listening".format(class_name=self.__class__.__name__))
                audio = self.recognizer.listen(source,timeout=self.config_recorder['listen_timeout'],phrase_time_limit=self.config_recorder['listen_phrase_time_limit'])
                response = {
                        "success": True,
                        "error": None,
                        "transcription": None
                }

            json_mic ={
                "hide": True
            }
            self.socketIO.emit("hideMic",json_mic,broadcast=True)

            ###### AUDIO PROCESSED FOR ONLINE #####
            if self.goal['connection_state'] == 'Online':
                self.success = False
                rospy.loginfo("{class_name} : ONLINE PROCESSING".format(class_name=self.__class__.__name__))
                if not audio is None:
                    try:
                        rospy.loginfo("{class_name} : start decoding".format(class_name=self.__class__.__name__))
                        response["transcription"] = self.recognizer.recognize_google(audio)

                    except sr.RequestError:
                        # API was unreachable or unresponsive
                        response["success"] = False
                        response["error"] = "API unavailable"
                    except sr.UnknownValueError:
                        # speech was unintelligible
                        response["error"] = "Unable to recognize speech"

                    rospy.loginfo("{class_name} : %s".format(class_name=self.__class__.__name__),str(response))

                    if response["transcription"]:
                        data=self.process_detection_online(response["transcription"])
                        if data != '':
                            self.success=True
                            self.json_response = {
                                "order": self.goal['order'],
                                "action": self.view_action,
                                "scenario": self.current_scenario,
                                "stt_result": data
                            }
                        
                    else:
                        print("I didn't catch that. What did you say?\n")
                        if response["error"]:
                            print("ERROR: {}".format(response["error"]))
                            
            ###### AUDIO PROCESSED FOR OFFLINE #####
            else:
                self.success = False
                rospy.loginfo("{class_name} : OFFLINE PROCESSING".format(class_name=self.__class__.__name__))
                if not audio is None:

                    audio_file_dir = os.path.join(self.current_directory,self.audio_dir)
                    if not os.path.exists(audio_file_dir):
                        os.makedirs(audio_file_dir)
                    try:
                        wav_data=audio.get_wav_data(convert_rate=self.config_recorder['convert_rate'],convert_width=self.config_recorder['convert_width'])
                        with open(audio_file_dir + '/intent_STT.wav', "wb") as f:
                            f.write(wav_data)
                        rospy.loginfo('{class_name} : Audio written successfully'.format(class_name=self.__class__.__name__))
                        audio_correct = True
                    except Exception as e:
                        rospy.logwarn("{class_name} : Couldn't write correctly audio in file : %s".format(class_name=self.__class__.__name__),str(e))
                        audio_correct = False


                    if audio_correct:
                        self.decoder.start_utt()

                        chunk = 1024
                        filename = audio_file_dir + '/intent_STT.wav'
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

                        if list_detection != []:
                            data = self.process_detection_offline(list_detection)
                            if data != '':
                                self.success = True
                                self.json_response = {
                                        "order": self.goal['order'],
                                        "action": self.view_action,
                                        "scenario": self.current_scenario,
                                        "stt_result": data
                                }
                

class SpeechRecognitionManager():

    def __init__(self):
        rospy.init_node("speech_recognition_manager",anonymous=True)

        self.socketIO = SocketIO('http://127.0.0.1', 5000, LoggingNamespace)

        if rospy.has_param("~stt_server_name"):
            self.STT_server = actionlib.SimpleActionServer(rospy.get_param("~stt_server_name"),SpeechRecognitionAction, self.handle_STT_action,False)
        else:
            rospy.logerr("{class_name} : No name specified for STT ActionServer".format(class_name=self.__class__.__name__))

        
        if rospy.has_param("~config"):
            self._config_decoder = rospy.get_param("~config")
        else:
            rospy.logerr("{class_name} : No config specified for STT ActionServer".format(class_name=self.__class__.__name__))

        if rospy.has_param("~config_recorder"):
            self._config_recorder = rospy.get_param("~config_recorder")
        else:
            rospy.logerr("{class_name} : No config specified for audio recorder".format(class_name=self.__class__.__name__))

        if rospy.has_param("~parser_action_database"):
            self._parser_action_database = rospy.get_param("~parser_action_database")
        else:
            rospy.logerr("{class_name} : No action parser specified for STT processing".format(class_name=self.__class__.__name__))

        rospy.loginfo("{class_name} : STTManager initialized".format(class_name=self.__class__.__name__))
        
        self.STT_server.start()


        
    def handle_STT_action(self,goal):

        rospy.loginfo("{class_name} : STT REQUEST RECEIVED".format(class_name=self.__class__.__name__))

        action_result = SpeechRecognitionResult()

        goal_data = json.loads(goal.order)

        thread_STT = AudioThread(config_recorder=self._config_recorder, config_decoder=self._config_decoder, parser_action_database=self._parser_action_database, goal=goal_data, socketIO=self.socketIO)

        thread_STT.start()
        while thread_STT.isAlive():
            if self.STT_server.is_preempt_requested():
                if not thread_STT.flag_stop:
                    thread_STT.recognizer.flag_stop = True
                    thread_STT.flag_stop = True

        if thread_STT.success and thread_STT.json_response:
            stt_result = json.dumps(thread_STT.json_response)
            action_result.stt_result = stt_result

            rospy.loginfo("{class_name} : ACTION SUCCEEDED".format(class_name=self.__class__.__name__))
            self.STT_server.set_succeeded(action_result)

        else:
            rospy.logwarn("{class_name} : ACTION ABORTED".format(class_name=self.__class__.__name__))
            self.STT_server.set_aborted(None)



if __name__ == "__main__":
    instance_server = SpeechRecognitionManager()
    while not rospy.is_shutdown():
        rospy.spin()