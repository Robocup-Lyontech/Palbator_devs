import json
from flask_socketio import SocketIO, send, emit
from templates import app
from flask_cors import CORS, cross_origin
from __main__ import socketIO

import os 
dir_path = os.path.dirname(os.path.realpath(__file__))


global stepCompletedJson

with open(dir_path+'/../templates/public/json/people.json') as p:
    people = json.load(p)

class AskName:
    global dataJsonToSendCurrentView
    global dataJsonToSendCurrentView
    @staticmethod
    def start(js_view_key, arguments, index, dataToUse):
        # AskName.action_id = arg_fetcher.get_argument(arguments, 'id')
        # if not AskName.action_id:
        #     logger.log("Missing id in {0} action arguments".format(js_view_key), "Views Manager", logger.ERROR)
        #     local_manager.send_view_result(js_view_key, {'error': 400})
        
        # args = arg_fetcher.get_argument(arguments, 'args')
        # speech = arg_fetcher.get_argument(args, 'speech')
        
        # text = arg_fetcher.get_argument(speech, 'title')
        text = arguments['speech']['title']
        
        
        # names = arg_fetcher.get_argument(args, 'names')
        names = people
        AskName.names = names
        
        dataJsonToSendCurrentView = {
                "view": js_view_key,
                "data": {
                    'textToShow': text,
                    'names' : names
                },
                "step":arguments,
                "index":index
        }
        socketIO.emit('currentViewToSend',dataJsonToSendCurrentView,broadcast=True)
        # emit('currentStep',dataJsonToSendCurrentStep)
        # socketio.sleep(5)
        # stepCompletedJson = {"idSteps": index}
        # emit('stepCompleted',stepCompletedJson)

    # @staticmethod
    # def received_data(local_manager, data):
    #     logger.log("AskName: data = " + str(data), "Views Manager", logger.DEBUG)
    #     if hasattr(AskName, 'action_id') and hasattr(AskName, 'names') and data['data'] in AskName.names:
    #         return {'id': AskName.action_id, 'name': data['data']}
    #     return True

    # @staticmethod
    # def stop(local_manager):
    #     if hasattr(AskName, 'topic_name') and AskName.topic_name:
    #         local_manager.dialog.deactivateTopic(AskName.topic_name)
    #         local_manager.dialog.unloadTopic(AskName.topic_name)
    #         if hasattr(AskName, 'reactivateMovement'):
    #             local_manager.autonomous_life.setAutonomousAbilityEnabled("SpeakingMovement", True)
    #             delattr(AskName, 'reactivateMovement')
    #         delattr(AskName, "topic_name")