import json
from flask_socketio import SocketIO, send, emit
from templates import app
from flask_cors import CORS, cross_origin
from __main__ import socketIO

global stepCompletedJson

class AskOpenDoor:
    @staticmethod
    def start(js_view_key, arguments, index, dataToUse):


        text = arguments['speech']['title']
        desc = arguments['speech']['description']
        
        dataJsonToSendCurrentView = {
                "view": js_view_key,
                "data": {
                    'textToShow': {
                        'title': text,
                        'description': [desc]
                    }
                },
                "step":arguments,
                "index":index
        }

        socketIO.emit('currentViewToSend',dataJsonToSendCurrentView,broadcast=True)
        


    # @staticmethod
    # def received_data(local_manager, data):
    #     if hasattr(AskOpenDoor, 'action_id'):
    #         return {'id': AskOpenDoor.action_id}
    #     return True

    # @staticmethod
    # def stop(local_manager):
    #     if hasattr(AskOpenDoor, 'topic_name') and AskOpenDoor.topic_name:
    #         local_manager.dialog.deactivateTopic(AskOpenDoor.topic_name)
    #         local_manager.dialog.unloadTopic(AskOpenDoor.topic_name)
    #         if hasattr(AskOpenDoor, 'reactivateMovement'):
    #             local_manager.autonomous_life.setAutonomousAbilityEnabled("SpeakingMovement", True)
    #             delattr(AskOpenDoor, 'reactivateMovement')
    #         delattr(AskOpenDoor, "topic_name")