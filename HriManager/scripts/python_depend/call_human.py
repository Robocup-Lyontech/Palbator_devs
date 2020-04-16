import json
from flask_socketio import SocketIO, send, emit
from templates import app
from flask_cors import CORS, cross_origin
# from __main__ import socketIO

global stepCompletedJson

class CallHuman:

    def __init__(self,socket):
        self.socket=socket
        
    # @staticmethod
    def start(self,js_view_key, arguments, index, dataToUse):
        # CallHuman.action_id = arg_fetcher.get_argument(arguments, 'id')
        # if not CallHuman.action_id:
        #     logger.log("Missing id in {0} action arguments".format(js_view_key), "Views Manager", logger.ERROR)
        #     local_manager.send_view_result(js_view_key, {'error': 400})

        text = arguments['speech']['title']
        # time = arguments['arguments']['time']

        dataJsonToSendCurrentView = {
                "view": js_view_key,
                'data': {
                    'textToShow': text,
                    # 'time': time
                },
                "step":arguments,
                "index":index
        }
        self.socket.emit('currentViewToSend',dataJsonToSendCurrentView,broadcast=True)
        # emit('currentStep',dataJsonToSendCurrentStep)
        # socketio.sleep(5)
   

    # @staticmethod
    # def received_data(local_manager, data):
    #     if hasattr(CallHuman, 'action_id'):
    #         return {'id': CallHuman.action_id}
    #     return True

    # @staticmethod
    # def stop(local_manager):
    #     if hasattr(CallHuman, 'topic_name') and CallHuman.topic_name:
    #         local_manager.dialog.deactivateTopic(CallHuman.topic_name)
    #         local_manager.dialog.unloadTopic(CallHuman.topic_name)
    #         if hasattr(CallHuman, 'reactivateMovement'):
    #             local_manager.autonomous_life.setAutonomousAbilityEnabled("SpeakingMovement", True)
    #             delattr(CallHuman, 'reactivateMovement')
    #         delattr(CallHuman, "topic_name")