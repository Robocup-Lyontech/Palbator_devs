import speech_recognition as sr
import os

import ttsMimic.msg
import rospy
import actionlib

class RecorderForSpeechRecog:

    def __init__(self,config):

        self.config = config
        self.recognizer = sr.Recognizer()
        self.microphone = sr.Microphone()
        self.audio_dir = self.config['audio_record_dir']
        self.current_directory=os.path.dirname(os.path.realpath(__file__))        
        self.client_TTS=actionlib.SimpleActionClient(self.config['tts_service'],ttsMimic.msg.TtsMimicAction)
        rospy.loginfo("Waiting for TTS server ...")
        self.client_TTS.wait_for_server()
        rospy.loginfo("TTS server connected")

    def tts_action(self,speech):
        """
        This function will use the TTS ROSAction to say the speech.

        :param speech: Speech to say
        :type speech: string
        """
        self.goal_TTS=ttsMimic.msg.TtsMimicGoal(speech)
        self.client_TTS.send_goal(self.goal_TTS)
        self.client_TTS.wait_for_result()

    def record_audio(self):
        """
        This function will record an audio file and convert it in a format compatible with the PocketSphinx detection.
        """
        if not isinstance(self.recognizer, sr.Recognizer):
            raise TypeError("`recognizer` must be `Recognizer` instance")

        if not isinstance(self.microphone, sr.Microphone):
            raise TypeError("`microphone` must be `Microphone` instance")

        # adjust the recognizer sensitivity to ambient noise and record audio
        # from the microphone
        
        cp=0
        for file in os.listdir(os.path.join(self.current_directory,self.audio_dir)):
            if file.endswith(".wav"):
                cp = cp+1

        with self.microphone as source:
            # self.recognizer.adjust_for_ambient_noise(source,duration=2)
            self.recognizer.adjust_for_ambient_noise(source,duration=self.config['ajusting_ambient_noise_duration'])
            print('Speak Now my friend!!!!!')
            # self.tts_action('Speak now please')
            print('Intent '+str(cp+1))
            audio = self.recognizer.listen(source,timeout=self.config['listen_timeout'],phrase_time_limit=self.config['listen_phrase_time_limit'])

    
        wav_data=audio.get_wav_data(convert_rate=self.config['convert_rate'],convert_width=self.config['convert_width'])
        with open(self.current_directory+'/'+self.audio_dir+'/intent'+str(cp+1)+'.wav', "wb") as f:
            f.write(wav_data)
        print('Audio written successfully')
  


