import json
from flask_socketio import SocketIO, send, emit
from templates import app
from flask_cors import CORS, cross_origin
# from __main__ import socketIO

import os 
dir_path = os.path.dirname(os.path.realpath(__file__))

with open(dir_path+'/../templates/public/json/locations_clean_up.json') as p:
    rooms = json.load(p)


class AskRoomToClean:

    def __init__(self,socket):
        self.socket=socket

    def start(self,js_view_key, arguments, index, dataToUse):

        text = arguments['speech']['title']


        
        dataJsonToSendCurrentView = {
                "view": js_view_key,
                "data": {
                    'textToShow': text,
                    'rooms' : rooms
                },
                "step":arguments,
                "index":index
        }
        self.socket.emit('currentViewToSend',dataJsonToSendCurrentView,broadcast=True)