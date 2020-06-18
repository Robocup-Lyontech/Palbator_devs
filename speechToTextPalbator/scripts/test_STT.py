#!/usr/bin/env python

import depend.speech_recognition as sr
import rospy

from threading import Thread
import time
from std_msgs.msg import String

class AudioThread(Thread):

    def __init__(self):

        Thread.__init__(self)
        self.recognizer = sr.Recognizer()
        self.micro = sr.Microphone()

    def run(self):
        with self.micro as source:
            self.recognizer.adjust_for_ambient_noise(source,duration=0.5)
            rospy.loginfo("Starts listening")
            audio = self.recognizer.listen(source,timeout=20,phrase_time_limit=5)
            response = {
                    "success": True,
                    "error": None,
                    "transcription": None
            }
        if not audio is None:
            try:
                rospy.loginfo("start decoding")
                response["transcription"] = self.recognizer.recognize_google(audio)

            except sr.RequestError:
                # API was unreachable or unresponsive
                response["success"] = False
                response["error"] = "API unavailable"
            except sr.UnknownValueError:
                # speech was unintelligible
                response["error"] = "Unable to recognize speech"

            rospy.loginfo(response)
        else:
            rospy.loginfo("audio is None")

class SpeechRecog():

    def __init__(self):

        rospy.init_node("Test_STT_Thomas")

        self.thread_audio = None
        self.sub_listen = rospy.Subscriber("/listen_audio",String,self.handle_listen)

        self.sub_stop_listening = rospy.Subscriber("/stop_listen",String,self.handle_stop_listen)

        rospy.loginfo('Initialized')

    def handle_listen(self,req):
        rospy.loginfo("listening request")
        self.thread_audio = AudioThread()
        self.thread_audio.start()

    def handle_stop_listen(self,req):
        rospy.loginfo("stop listening request")
        if self.thread_audio.isAlive():
            self.thread_audio.recognizer.flag_stop = True
        



if __name__ == "__main__":
    a = SpeechRecog()
    while not rospy.is_shutdown():
        rospy.spin()