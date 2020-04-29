import json
from flask_socketio import SocketIO, send, emit
from templates import app
from flask_cors import CORS, cross_origin
from __main__ import socketIO

global stepCompletedJson

# with open('templates/public/json/locations.json') as l:
#     locations = json.load(l)

class GoTo:
    @staticmethod
    def start(js_view_key, arguments, index, dataToUse):
        # GoTo.action_id = arg_fetcher.get_argument(arguments, 'id')
        # if not GoTo.action_id:
        #     logger.log("Missing id in {0} action arguments".format(js_view_key), "Views Manager", logger.ERROR)
        #     local_manager.send_view_result(js_view_key, {'error': 400})
        
        # args = arg_fetcher.get_argument(arguments, 'args')
        # speech = arg_fetcher.get_argument(args, 'speech')
        
        # text = arg_fetcher.get_argument(speech, 'title')
        # said = arg_fetcher.get_argument(speech, 'said')
        
        # location = arg_fetcher.get_argument(args, 'location')

        locations = arguments['arguments']['location']
        text = arguments['speech']['title']


        dataJsonToSendCurrentView = {
                "view": js_view_key,
                "data": {
                    'textToShow': text,
                    'location': locations
                },
                "step":arguments,
                "index":index
        }
        socketIO.emit('currentViewToSend',dataJsonToSendCurrentView,broadcast=True)
        # emit('currentStep',dataJsonToSendCurrentStep)
        # socketio.sleep(5)


        
    # @staticmethod
    # def received_data(local_manager, data):
    #     if hasattr(GoTo, 'topic_name'): # Means that a .top is loaded, need to check if it's done
    #         if data['data'] == "1":
    #             return {'id': GoTo.action_id}
    #     elif hasattr(GoTo, 'action_id'):
    #         return {'id': GoTo.action_id}
    #     return False

    # @staticmethod
    # def stop(local_manager):
    #     if hasattr(GoTo, 'topic_name') and GoTo.topic_name:
    #         local_manager.dialog.deactivateTopic(GoTo.topic_name)
    #         local_manager.dialog.unloadTopic(GoTo.topic_name)
    #         if hasattr(GoTo, 'reactivateMovement'):
    #             local_manager.autonomous_life.setAutonomousAbilityEnabled("SpeakingMovement", True)
    #             delattr(GoTo, 'reactivateMovement')
    #         delattr(GoTo, "topic_name")