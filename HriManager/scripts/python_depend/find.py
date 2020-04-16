import json
from flask_socketio import SocketIO, send, emit
from templates import app
from flask_cors import CORS, cross_origin
# from __main__ import socketIO

global stepCompletedJson


class Find:
    def __init__(self,socket):
        self.socket=socket

    # @staticmethod
    def start(self,js_view_key, arguments, index, dataToUse):
        
        text = arguments['speech']['title']
        what = arguments['arguments']['what']
        pathOnTablet = arguments['arguments']['pathOnTablet']

        dataJsonToSendCurrentView = {
                "view": js_view_key,
                "data": {
                    'textToShow': text,
                    'what': what,
                    'pathOnTablet': pathOnTablet
                },
                "step":arguments,
                "index":index
        }
        self.socket.emit('currentViewToSend',dataJsonToSendCurrentView,broadcast=True)