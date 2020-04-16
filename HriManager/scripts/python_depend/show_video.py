import json
from flask_socketio import SocketIO, send, emit
from templates import app
from flask_cors import CORS, cross_origin
# from __main__ import socketIO

global stepCompletedJson
import os 
dir_path = os.path.dirname(os.path.realpath(__file__))

with open(dir_path+'/../templates/public/json/videos.json') as v:
    videos = json.load(v)

class ShowVideo:

    def __init__(self,socket):
        self.socket=socket

    # @staticmethod
    def start(self,js_view_key, arguments, index, dataToUse):
        # ShowVideo.action_id = arg_fetcher.get_argument(arguments, 'id')
        # if not ShowVideo.action_id:
        #     logger.log("Missing id in {0} action arguments".format(js_view_key), "Views Manager", logger.ERROR)
        #     local_manager.send_view_result(js_view_key, {'error': 400})
        
        # args = arg_fetcher.get_argument(arguments, 'args')
        # speech = arg_fetcher.get_argument(args, 'speech')
        
        text = arguments['speech']['title']
        text = arguments['arguments']['description']
        if desc:
            desc = desc.split(';')
        else:
            desc = [""]
        

        dataJsonToSendCurrentView = {
                "view": js_view_key,
                'data': {
                    'textToShow': {
                        'title': text,
                        'description': desc
                    },
                    # 'video': arg_fetcher.get_argument(args, 'video')
                    'video': videos
                },
                "step":arguments,
                "index":index
        }
        self.socket.emit('currentViewToSend',dataJsonToSendCurrentView,broadcast=True)
        # emit('currentStep',dataJsonToSendCurrentStep)
        # socketio.sleep(5)

    # @staticmethod
    # def received_data(local_manager, data):
    #     if hasattr(ShowVideo, 'action_id'):
    #         return {'id': ShowVideo.action_id}
    #     return True

    # @staticmethod
    # def stop(local_manager):
    #     if hasattr(ShowVideo, 'topic_name') and ShowVideo.topic_name:
    #         local_manager.dialog.deactivateTopic(ShowVideo.topic_name)
    #         local_manager.dialog.unloadTopic(ShowVideo.topic_name)
    #         if hasattr(ShowVideo, 'reactivateMovement'):
    #             local_manager.autonomous_life.setAutonomousAbilityEnabled("SpeakingMovement", True)
    #             delattr(ShowVideo, 'reactivateMovement')
    #         delattr(ShowVideo, "topic_name")