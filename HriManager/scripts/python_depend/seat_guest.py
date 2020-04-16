import json
from flask_socketio import SocketIO, send, emit
from templates import app
from flask_cors import CORS, cross_origin
# from __main__ import socketIO

global stepCompletedJson


class SeatGuest:
    def __init__(self,socket):
        self.socket=socket

    # @staticmethod
    def start(self,js_view_key, arguments, index, dataToUse):
        
        text = arguments['speech']['title']
        people = arguments['arguments']['who']
        # text = arguments['speech']['name']

        dataJsonToSendCurrentView = {
                "view": js_view_key,
                "data": {
                    'textToShow': text,
                    'people': people
                },
                "step":arguments,
                "index":index
        }
        self.socket.emit('currentViewToSend',dataJsonToSendCurrentView,broadcast=True)

    # @staticmethod
    # def received_data(local_manager, data):
    #     if hasattr(SeatGuest, 'topic_name'): # Means that a .top is loaded, need to check if it's done
    #         if data['data'] == "1":
    #             return {'id': SeatGuest.action_id}
    #     elif hasattr(SeatGuest, 'action_id'):
    #         return {'id': SeatGuest.action_id}
    #     return False

    # @staticmethod
    # def stop(local_manager):
    #     if hasattr(SeatGuest, 'topic_name') and SeatGuest.topic_name:
    #         local_manager.dialog.deactivateTopic(SeatGuest.topic_name)
    #         local_manager.dialog.unloadTopic(SeatGuest.topic_name)
    #         if hasattr(SeatGuest, 'reactivateMovement'):
    #             local_manager.autonomous_life.setAutonomousAbilityEnabled("SpeakingMovement", True)
    #             delattr(SeatGuest, 'reactivateMovement')
    #         delattr(SeatGuest, "topic_name")