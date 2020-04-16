import json
from flask_socketio import SocketIO, send, emit
from templates import app
from flask_cors import CORS, cross_origin
# from __main__ import socketIO

import os 
dir_path = os.path.dirname(os.path.realpath(__file__))


class OpenDoor:

    def __init__(self,socket):
        self.socket=socket

    def start(self,js_view_key, arguments, index, dataToUse):
        # AskName.action_id = arg_fetcher.get_argument(arguments, 'id')
        # if not AskName.action_id:
        #     logger.log("Missing id in {0} action arguments".format(js_view_key), "Views Manager", logger.ERROR)
        #     local_manager.send_view_result(js_view_key, {'error': 400})
        
        # args = arg_fetcher.get_argument(arguments, 'args')
        # speech = arg_fetcher.get_argument(args, 'speech')
        
        # text = arg_fetcher.get_argument(speech, 'title')
        text = arguments['speech']['title']
        
        dataJsonToSendCurrentView = {
                "view": js_view_key,
                "data": {
                    'textToShow': text
                },
                "step":arguments,
                "index":index
        }
        self.socket.emit('currentViewToSend',dataJsonToSendCurrentView,broadcast=True)