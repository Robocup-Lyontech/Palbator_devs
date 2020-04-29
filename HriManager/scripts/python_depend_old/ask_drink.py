import json
from flask_socketio import SocketIO, send, emit
from templates import app
from flask_cors import CORS, cross_origin
from __main__ import socketIO


import os 
dir_path = os.path.dirname(os.path.realpath(__file__))


global stepCompletedJson

with open(dir_path+'/../templates/public/json/drinks.json') as d:
    drink = json.load(d)
    
class AskDrink:
    @staticmethod
    def start(js_view_key, arguments, index, dataToUse):
        # AskDrink.action_id = arg_fetcher.get_argument(arguments, 'id')
        # if not AskDrink.action_id:
        #     logger.log("Missing id in {0} action arguments".format(js_view_key), "Views Manager", logger.ERROR)
        #     local_manager.send_view_result(js_view_key, {'error': 400})
        
        drinks = drink
        AskDrink.drinks = drinks
        
        text = arguments['speech']['title']


        dataJsonToSendCurrentView = {
                "view": js_view_key,
                "data": {
                    'textToShow': text,
                    'drinks' : drinks
                },
                "step":arguments,
                "index":index
        }
        socketIO.emit('currentViewToSend',dataJsonToSendCurrentView,broadcast=True)
        # emit('currentStep',dataJsonToSendCurrentStep)
        # socketio.sleep(5)

    # @staticmethod
    # def received_data(local_manager, data):
    #     if hasattr(AskDrink, 'action_id'):
    #         if hasattr(AskDrink, 'drinks'):
    #             for drink in AskDrink.drinks:
    #                 if data['data'] == drink['name']:
    #                     return {'id': AskDrink.action_id, 'drink': drink}
    #     return True

    # @staticmethod
    # def stop(local_manager):
    #     if hasattr(AskDrink, 'topic_name') and AskDrink.topic_name:
    #         local_manager.dialog.deactivateTopic(AskDrink.topic_name)
    #         local_manager.dialog.unloadTopic(AskDrink.topic_name)
    #         if hasattr(AskDrink, 'reactivateMovement'):
    #             local_manager.autonomous_life.setAutonomousAbilityEnabled("SpeakingMovement", True)
    #             delattr(AskDrink, 'reactivateMovement')
    #         delattr(AskDrink, "topic_name")