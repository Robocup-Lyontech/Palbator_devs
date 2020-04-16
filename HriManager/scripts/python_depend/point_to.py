import json
from templates import app
from flask_cors import CORS, cross_origin
from flask_socketio import SocketIO, send, emit


class PointTo:
    def __init__(self,socket):
        self.socket=socket

    # @staticmethod
    def start(self,js_view_key, arguments, index, dataToUse):
        
        text = arguments['speech']['title']

        if arguments['arguments']['what']=='human':
            people = arguments['arguments']['who']

        elif arguments['arguments']['what']=='chair':
            people={
                "name": arguments['arguments']['what'],
                "guestPhotoPath": arguments['arguments']['pathOnTablet']
            }
        
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