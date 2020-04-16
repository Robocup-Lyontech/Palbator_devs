#! /usr/bin/env python

import rospy

import actionlib

import ttsMimic.msg
import subprocess

import os

class ttsMimicActionServer(object):

    def __init__(self):
        rospy.init_node('tts_Mimic_action_server_node')

        _action_server_name = "~action_TTS_server_name"
        if rospy.has_param(_action_server_name):
            self.action_tts_server=actionlib.SimpleActionServer(rospy.get_param(_action_server_name),ttsMimic.msg.TtsMimicAction,self.action_server_callback,auto_start=False)
            self.action_tts_server_feedback=ttsMimic.msg.TtsMimicFeedback()
            self.action_tts_server_result=ttsMimic.msg.TtsMimicResult()
        else:
            rospy.logerr("{class_name} : No name specified for TTS action server".format(class_name=self.__class__.__name__))

        _output_mode_param = "~output_mode"
        if rospy.has_param(_output_mode_param):
            self.output_mode = rospy.get_param(_output_mode_param)
        else:
            self.output_mode = 'default'

        if self.output_mode == 'record':
            self.path_to_wav=rospy.get_param("~path_dir_for_records")
            if self.path_to_wav == '':
                rospy.logerr("{class_name} : No path specified for records".format(class_name=self.__class__.__name__))

        self.action_tts_server.start()
        self.current_index=None
        rospy.loginfo("{class_name} : tts mimic server initiated".format(class_name=self.__class__.__name__))

    def get_current_index(self):
        cp=0
        for file in os.listdir(self.path_to_wav):
            if file.endswith(".wav"):
                cp=cp+1
        self.current_index=cp+1


    def action_server_callback(self,goal):
        rospy.loginfo("{class_name} : Action initiating ...".format(class_name=self.__class__.__name__))
        success=True

        self.action_tts_server_feedback.tts_feedback='None'
        text=goal.text_to_say


        if self.output_mode == 'default':
            self.process=subprocess.Popen(['mimic','-t',str(text)],stdin=subprocess.PIPE, stdout=subprocess.PIPE,close_fds=True)
        else:
            self.get_current_index()
            filename = 'record_'+str(self.current_index)+'.wav'
            self.process=subprocess.Popen(['mimic','-t',str(text),'-o',os.path.join(self.path_to_wav,filename)],stdin=subprocess.PIPE, stdout=subprocess.PIPE,close_fds=True)
        
        # self.process.stdin.write(text)
        self.process.stdin.close()
        while self.process.poll() is None:
            if self.action_tts_server.is_preempt_requested():
                rospy.loginfo('{class_name} : Preempted SayVocalSpeech Action'.format(class_name=self.__class__.__name__))
                self.process.terminate()
                self.action_tts_server.set_preempted()
                success=False
                break

        self.action_tts_server.publish_feedback(self.action_tts_server_feedback)

        if success:
            self.action_tts_server_result.tts_result=self.action_tts_server_feedback.tts_feedback
            rospy.loginfo("{class_name} : Action SayVocalSpeech succeeded".format(class_name=self.__class__.__name__))
            self.action_tts_server.set_succeeded(self.action_tts_server_result)

if __name__ == "__main__":
    instance_TTS=ttsMimicActionServer()
    while not rospy.is_shutdown():
        rospy.spin()