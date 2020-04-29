import json
from flask_socketio import SocketIO, send, emit
from templates import app
from flask_cors import CORS, cross_origin
from __main__ import socketIO

global stepCompletedJson


class AskAge:
    @staticmethod
    def start(js_view_key, arguments, index, dataToUse):
        # AskAge.action_id = arg_fetcher.get_argument(arguments, 'id')
        # if not AskAge.action_id:
        #     logger.log("Missing id in {0} action arguments".format(js_view_key), "Views Manager", logger.ERROR)
        #     local_manager.send_view_result(js_view_key, {'error': 400})
        
        # args = arg_fetcher.get_argument(arguments, 'args')
        # speech = arg_fetcher.get_argument(args, 'speech')
        
        # text = arg_fetcher.get_argument(speech, 'title')
        # said = arg_fetcher.get_argument(speech, 'said')

        # name = arg_fetcher.get_argument(speech, 'name')

        # text = text.format(name=name)
        # said = said.format(name=name)

        # ages = [str(i) for i in range(99)]
        text = arguments['speech']['title']

        dataJsonToSendCurrentView = {
                "view": js_view_key,
                "data": {
                    'textToShow': text
                },
                "step":arguments,
                "index":index
        }
        socketIO.emit('currentViewToSend',dataJsonToSendCurrentView,broadcast=True)

        
        

    # @staticmethod
    # def received_data(local_manager, data):
    #     try:
    #         data['data'] = int(data['data'])
    #     except ValueError:
    #         pass
    #     if hasattr(AskAge, 'action_id') and type(data['data']) == int:
    #         return {'id': AskAge.action_id, 'age': data['data']}
    #     return True

    # @staticmethod
    # def stop(local_manager):
    #     if hasattr(AskAge, 'topic_name') and AskAge.topic_name:
    #         local_manager.dialog.deactivateTopic(AskAge.topic_name)
    #         local_manager.dialog.unloadTopic(AskAge.topic_name)
    #         if hasattr(AskAge, 'reactivateMovement'):
    #             local_manager.autonomous_life.setAutonomousAbilityEnabled("SpeakingMovement", True)
    #             delattr(AskAge, 'reactivateMovement')
    #         delattr(AskAge, "topic_name")