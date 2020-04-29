import json
from templates import app
from flask_cors import CORS, cross_origin
from flask_socketio import SocketIO, send, emit
from __main__ import socketIO


global stepCompletedJson

# La methode Start envoie au FLASK la VUE a lancer avec les bons attributs
class Wait:
    @staticmethod
    def start(js_view_key, arguments, index, dataToUse):
        
        time = arguments['arguments']['time']
        text = arguments['speech']['title']

        dataJsonToSendCurrentView = {
                "view": js_view_key,
                "data": {
                    'textToShow': text,
                    'time' : time
                },
                "step":arguments,
                "index":index
        }
        socketIO.emit('currentViewToSend',dataJsonToSendCurrentView,broadcast=True)


    # @staticmethod
    # def received_data(local_manager, data):
    #     if hasattr(Wait, 'action_id') and data['data'] != "1":
    #         return {'id': Wait.action_id}
    #     return False

    # @staticmethod
    # def stop(local_manager):
    #     if hasattr(Wait, 'topic_name') and Wait.topic_name:
    #         local_manager.dialog.deactivateTopic(Wait.topic_name)
    #         local_manager.dialog.unloadTopic(Wait.topic_name)
    #         if hasattr(Wait, 'reactivateMovement'):
    #             local_manager.autonomous_life.setAutonomousAbilityEnabled("SpeakingMovement", True)
    #             delattr(Wait, 'reactivateMovement')
    #         delattr(Wait, "topic_name")