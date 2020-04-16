#!/usr/bin/env python

import rospy
import actionlib

import speechToTextPalbator.msg

import speech_recognition as sr
import json
import os
import ttsMimic.msg

class SpeechToTextOnline(object):

    def __init__(self):
        rospy.init_node("STT_online_node")

        if rospy.has_param("~stt_online_server_name"):
            self.action_server=actionlib.SimpleActionServer(rospy.get_param("~stt_online_server_name"), speechToTextPalbator.msg.SttOnlineAction,self.handle_server_callback,auto_start=False)
            self.action_server_feedback=speechToTextPalbator.msg.SttOnlineFeedback()
            self.action_server_result=speechToTextPalbator.msg.SttOnlineResult()
        else:
            rospy.logerr("{class_name} : No name specified for STT online server.".format(class_name=self.__class__.__name__))

        self.current_directory=os.path.dirname(os.path.realpath(__file__))    

        if rospy.has_param('~config_recorder'):
            self.config_recorder = rospy.get_param("~config_recorder")
        else:
            rospy.logerr("{class_name} : No configuration specified for audio recorder".format(class_name=self.__class__.__name__))

        self.audio_dir = self.config_recorder['audio_record_dir']   

        if rospy.has_param('~config'):
            self.config = rospy.get_param('~config')
        else:
            rospy.logerr("{class_name} : No configuration specified for Speech Recognition Server".format(class_name=self.__class__.__name__)) 


        if rospy.has_param("~parser_action_database"):
            self.parser_action_database = rospy.get_param("~parser_action_database")     
        else:
            rospy.logerr("{class_name} : No configuration specified for parser action database".format(class_name=self.__class__.__name__))   

        if rospy.has_param("~tts_server_name"):
            self.client_TTS=actionlib.SimpleActionClient(rospy.get_param("~tts_server_name"),ttsMimic.msg.TtsMimicAction)
            rospy.loginfo("{class_name} : Waiting for TTS server ...".format(class_name=self.__class__.__name__))
            self.client_TTS.wait_for_server()
            rospy.loginfo("{class_name} : TTS connected".format(class_name=self.__class__.__name__))
        else:
            rospy.logerr("{class_name} : No name specified for TTS server".format(class_name=self.__class__.__name__))

        self.enable_detection = None

        rospy.on_shutdown(self.shutdown)
        self.action_server.start()
        rospy.loginfo("{class_name} : server for STT online initiated".format(class_name=self.__class__.__name__))

    def shutdown(self):
        """This function is executed on node shutdown."""
        rospy.loginfo("{class_name} : Stop ASRControl".format(class_name=self.__class__.__name__))
        self.action_server.set_aborted()

    def tts_action(self,speech):
        """
        This function will use the TTS ROSAction to say the specified speech.

        :param speech: Speech to say
        :type speech: string
        """
        self.goal_TTS=ttsMimic.msg.TtsMimicGoal(speech)
        self.client_TTS.send_goal(self.goal_TTS)
        self.client_TTS.wait_for_result()

    def parser_view_action_to_dic_mode(self,view_action):
        """
        This function will get the database name in the configuration file associated to the view action name.

        :param view_action: Action name of the current view
        :type view_action: string
        """
        dic_mode = ""
        if view_action in self.parser_action_database[self.scenario]:
            dic_mode = self.parser_action_database[self.scenario][view_action]
            rospy.logwarn("{class_name} : DIC MODE ".format(class_name=self.__class__.__name__) + str(dic_mode))
            self.enable_detection = True
        else:
            rospy.logwarn("{class_name} : Detection is not allowed for this action".format(class_name=self.__class__.__name__))
            self.enable_detection = False
        return dic_mode

    def load_database(self):
        """
        This function will load the verification database associated to the current scenario and the current view.
        """
        self.config_scenario = self.config[self.scenario]
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


    def recognize_speech_from_mic(self,recognizer, microphone):
        """Transcribe speech from recorded from `microphone`.

        Returns a dictionary with three keys:
        "success": a boolean indicating whether or not the API request was
                successful
        "error":   `None` if no error occured, otherwise a string containing
                an error message if the API could not be reached or
                speech was unrecognizable
        "transcription": `None` if speech could not be transcribed,
                otherwise a string containing the transcribed text
        """
        # check that recognizer and microphone arguments are appropriate type
        if not isinstance(recognizer, sr.Recognizer):
            raise TypeError("`recognizer` must be `Recognizer` instance")

        if not isinstance(microphone, sr.Microphone):
            raise TypeError("`microphone` must be `Microphone` instance")

        # adjust the recognizer sensitivity to ambient noise and record audio
        # from the microphone
        with microphone as source:
            recognizer.adjust_for_ambient_noise(source,duration=self.config_recorder['ajusting_ambient_noise_duration'])
            print("Speak now please")
            self.tts_action("Speak Now please")
            audio = recognizer.listen(source,timeout=self.config_recorder['listen_timeout'],phrase_time_limit=self.config_recorder['listen_phrase_time_limit'])

        # set up the response object
        response = {
            "success": True,
            "error": None,
            "transcription": None
        }

        # try recognizing the speech in the recording
        # if a RequestError or UnknownValueError exception is caught,
        #     update the response object accordingly
        try:
            response["transcription"] = recognizer.recognize_google(audio)

        except sr.RequestError:
            # API was unreachable or unresponsive
            response["success"] = False
            response["error"] = "API unavailable"
        except sr.UnknownValueError:
            # speech was unintelligible
            response["error"] = "Unable to recognize speech"

        return response


    def process_detection(self,transcription):
        """
        This function will process the detection received from the recognizer.

        :param transcription: detection result of the recognizer
        :type transcription: string
        """
        transcription=transcription.upper()
        data = ''
        for element in self.database_words:
            if element in transcription:
                data=element
                rospy.loginfo("{class_name} : GOOD DETECTION".format(class_name=self.__class__.__name__))
                break

        if data == '':
            rospy.loginfo("{class_name} : BAD DETECTION. Please try again".format(class_name=self.__class__.__name__))
        
        return data

    def handle_server_callback(self,goal):
        """
        ROS ActionServer Callback. This function will process the goal received from the ActionClient 
        and initiate the Speech Detection according to data contained in the goal.

        :param goal: data of current view and scenario received from ActionClient
        :type goal: string
        """
        rospy.loginfo("{class_name} : Action Online initiating ...".format(class_name=self.__class__.__name__))
        success=False

        self.action_server_feedback.stt_feedback = 'None'
        data=None

        d=json.loads(goal.order)
        view_id=d['order']
        view_action=d['action']
        self.scenario = d['scenario']

        self.dictionary_choose = self.parser_view_action_to_dic_mode(view_action)
        self.load_database()

        if self.enable_detection == True:

            while not success and not rospy.is_shutdown():
                if self.action_server.is_preempt_requested():
                    rospy.loginfo("{class_name} : Preempted request server offline".format(class_name=self.__class__.__name__))
                    self.action_server.set_preempted()
                    break

                recognizer = sr.Recognizer()
                microphone = sr.Microphone()
                guess = self.recognize_speech_from_mic(recognizer, microphone)
                print("You said: {}".format(guess["transcription"]))

                if guess["transcription"]:
                        data=self.process_detection(guess["transcription"])
                        if data != '':
                            success=True
                        continue

                if not guess["success"]:
                    continue
                print("I didn't catch that. What did you say?\n")
                if guess["error"]:
                    print("ERROR: {}".format(guess["error"]))
                    continue
        else:
            rospy.logwarn("{class_name} : Detection not available".format(class_name=self.__class__.__name__))
            rospy.logwarn("{class_name} : Preempting request server offline...".format(class_name=self.__class__.__name__))
            self.action_server.set_preempted()

        if success:
            self.action_server.publish_feedback(self.action_server_feedback)
            self.action_server_result.stt_result = data
            rospy.loginfo("{class_name} : Action STT Online succeeded".format(class_name=self.__class__.__name__))
            self.action_server.set_succeeded(self.action_server_result)

if __name__ == "__main__":
    instance_online=SpeechToTextOnline()
    while not rospy.is_shutdown():
        rospy.spin()