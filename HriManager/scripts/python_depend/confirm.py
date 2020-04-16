import json
from flask_socketio import SocketIO, send, emit
from templates import app
from flask_cors import CORS, cross_origin
# from __main__ import socketIO

global stepCompletedJson


class Confirm:

    def __init__(self,socket):
        self.socket=socket

    # @staticmethod
    def start(self,js_view_key, arguments, index, dataToUse):
        # Confirm.action_id = arg_fetcher.get_argument(arguments, 'id')
        # if not Confirm.action_id:
        #     logger.log("Missing id in {0} action arguments".format(js_view_key), "Views Manager", logger.ERROR)
        #     local_manager.send_view_result(js_view_key, {'error': 400})
        

        text = arguments['speech']['title']
        if text and "{drink}" in text:
            text = text.format(drink=dataToUse)
        if text and "{name}" in text:
            text = text.format(name=dataToUse)
        
        dataJsonToSendCurrentView = {
                "view": js_view_key,
                "data": {
                    'textToShow': text
                },
                "step":arguments,
                "index":index
        }

        self.socket.emit('currentViewToSend',dataJsonToSendCurrentView,broadcast=True)

    # @staticmethod
    # def received_data(local_manager, data):
    #     data = data['data']
    #     if hasattr(Confirm, 'action_id'):
    #         if type(data) is bool:
    #             return {'id': Confirm.action_id, 'confirm': data}
    #         elif data == "True" or data == "False":
    #             return {'id': Confirm.action_id, 'confirm': data == "True"}

    # @staticmethod
    # def stop(local_manager):
    #     if hasattr(Confirm, 'topic_name') and Confirm.topic_name:
    #         local_manager.dialog.deactivateTopic(Confirm.topic_name)
    #         local_manager.dialog.unloadTopic(Confirm.topic_name)
    #         if hasattr(Confirm, 'reactivateMovement'):
    #             local_manager.autonomous_life.setAutonomousAbilityEnabled("SpeakingMovement", True)
    #             delattr(Confirm, 'reactivateMovement')
    #         delattr(Confirm, "topic_name")