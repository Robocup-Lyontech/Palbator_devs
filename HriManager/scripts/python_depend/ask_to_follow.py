import json
from templates import app
from flask_cors import CORS, cross_origin
from flask_socketio import SocketIO, send, emit
# from __main__ import socketIO

global stepCompletedJson

# with open('templates/public/json/locations.json') as l:
#     locations = json.load(l)
    
class AskToFollow:

    def __init__(self,socket):
        self.socket=socket
    # @staticmethod
    def start(self,js_view_key, arguments, index, dataToUse):
        
        text = arguments['speech']['title']
        locations = arguments['arguments']['location']

        dataJsonToSendCurrentView = {
                "view": js_view_key,
                "data": {
                    'textToShow': text,
                    'location': locations
                },
                "step":arguments,
                "index":index
        }
        self.socket.emit('currentViewToSend',dataJsonToSendCurrentView,broadcast=True)
        # emit('currentStep',dataJsonToSendCurrentStep)
        # socketio.sleep(5)
        

    # @staticmethod
    # def received_data(local_manager, data):
    #     if hasattr(AskToFollow, 'topic_name'): # Means that a .top is loaded, need to check if it's done
    #         if data['data'] == "1":
    #             return {'id': AskToFollow.action_id}
    #     elif hasattr(AskToFollow, 'action_id'):
    #         return {'id': AskToFollow.action_id}
    #     return False

    # @staticmethod
    # def stop(local_manager):
    #     if hasattr(AskToFollow, 'topic_name') and AskToFollow.topic_name:
    #         local_manager.dialog.deactivateTopic(AskToFollow.topic_name)
    #         local_manager.dialog.unloadTopic(AskToFollow.topic_name)
    #         if hasattr(AskToFollow, 'reactivateMovement'):
    #             local_manager.autonomous_life.setAutonomousAbilityEnabled("SpeakingMovement", True)
    #             delattr(AskToFollow, 'reactivateMovement')
    #         delattr(AskToFollow, "topic_name")