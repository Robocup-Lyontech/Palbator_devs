#!/usr/bin/env python

import rospy
import actionlib

import speechToTextPalbator.msg

import speech_recognition as sr
import json

class SpeechToTextOnline(object):

    def __init__(self):
        rospy.init_node("STT_online_node")

        self.action_server=actionlib.SimpleActionServer("action_STT_online", speechToTextPalbator.msg.SttOnlineAction,self.handle_server_callback,auto_start=False)
        self.action_server_feedback=speechToTextPalbator.msg.SttOnlineFeedback()
        self.action_server_result=speechToTextPalbator.msg.SttOnlineResult()

        self.action_server.start()

        self.list_to_ask_name=['my name is','i am',"i'm"]
        self.list_to_ask_drink=['my favourite drink is','i love']
        self.list_to_say_yes=['yes']
        self.list_to_say_no=['no']
        self.list_to_say_next=['next']
        rospy.loginfo("server for STT online initiated")

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
            recognizer.adjust_for_ambient_noise(source)
            audio = recognizer.listen(source,phrase_time_limit=10)

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


    def process_detection(self,transcription,action):
        data=None

        transcription=transcription.lower()

        if action=='askName':

            for element in self.list_to_ask_name:
                if element in transcription:
                    data=transcription.split(element)[1]
                    break

            if data is None:
                if len(transcription.split())==1:
                    rospy.loginfo('I am not sure but i heard '+str(transcription))
                    data=transcription


        elif action=='askDrink':
            for element in self.list_to_ask_drink:
                if element in transcription:
                    data=transcription.split(element)[1]
                    break

            if data is None:
                if len(transcription.split())==1:
                    rospy.loginfo('I am not sure but i heard '+str(transcription))
                    data=transcription

        elif action=='confirmation':
            for element in self.list_to_say_yes:
                if element in transcription:
                    data='yes'
                    break
            
            for element in self.list_to_say_no:
                if element in transcription:
                    data='no'
                    break

        elif action=='askOpenDoor':
            for element in self.list_to_say_next:
                if element in transcription:
                    data='next'
                    break

        else:
            rospy.loginfo("I don't know the context of the conversation so I don't know what to do. I am sorry.")
            data=''
            

        if data is None:
            data=''
            rospy.loginfo("I didn't understand what you wanted but I hope I will understand in the future.")
            return data

        else:
            return data

    def handle_server_callback(self,goal):
        rospy.loginfo("Action initiating ...")
        success=False

        self.action_server_feedback.stt_feedback = 'None'
        # index=0
        data=None

        d=json.loads(goal.order)
        view_id=d['order']
        view_action=d['action']
        # view_speech_to_say=d['speech']

        while not success and not rospy.is_shutdown():
            if self.action_server.is_preempt_requested():
                rospy.loginfo("Preempted request server offline")
                self.action_server.set_preempted()
                break

            recognizer = sr.Recognizer()
            microphone = sr.Microphone()

            
            # print(view_speech_to_say) ##### A IMPLEMENTER -> SERVER TTS MIMIC

                # get the guess from the user
                # if a transcription is returned, break out of the loop and
                #     continue
                # if no transcription returned and API request failed, break
                #     loop and continue
                # if API request succeeded but no transcription was returned,
                #     re-prompt the user to say their guess again. Do this up
                #     to PROMPT_LIMIT times
            print('Speak!')
            guess = self.recognize_speech_from_mic(recognizer, microphone)
            print("You said: {}".format(guess["transcription"]))

            
            if guess["transcription"]:
                    data=self.process_detection(guess["transcription"],view_action)
                    
                    # index=self.next_index(index,data)
                    if data != '':
                        success=True
                    continue

            if not guess["success"]:
                continue
            print("I didn't catch that. What did you say?\n")
            if guess["error"]:
                print("ERROR: {}".format(guess["error"]))
                continue

        if success:
            self.action_server.publish_feedback(self.action_server_feedback)
            self.action_server_result.stt_result = data
            rospy.loginfo("Action STT Online succeeded")
            self.action_server.set_succeeded(self.action_server_result)

if __name__ == "__main__":
    instance_online=SpeechToTextOnline()
    while not rospy.is_shutdown():
        rospy.spin()