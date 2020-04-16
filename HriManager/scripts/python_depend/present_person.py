import json
from flask_socketio import SocketIO, send, emit
from templates import app
from flask_cors import CORS, cross_origin
# from __main__ import socketIO

global stepCompletedJson


class PresentPerson:
    def __init__(self,socket):
        self.socket=socket

    # @staticmethod
    def start(self,js_view_key, arguments, index, dataToUse):
        # PresentPerson.action_id = arg_fetcher.get_argument(arguments, 'id')
        # if not PresentPerson.action_id:
        #     logger.log("Missing id in {0} action arguments".format(js_view_key), "Views Manager", logger.ERROR)
        #     local_manager.send_view_result(js_view_key, {'error': 400})
        
        # args = arg_fetcher.get_argument(arguments, 'args')
        # speech = arg_fetcher.get_argument(args, 'speech')
        
        # text = arg_fetcher.get_argument(speech, 'title')
        # said = arg_fetcher.get_argument(speech, 'said')

        text = arguments['speech']['said']
        who = arguments['arguments']['who']
        to = arguments['arguments']['to']
        # print("WHO!!")
        # print(who)
        dataJsonToSendCurrentView = {
                "view": js_view_key,
                'data': {
                    'textToShow': text,
                    'people':{
                        'who': {
                            'drink': who['drinkObj'],
                            'name': who['name'],
                            'guestPhotoPath': who['guestPhotoPath']   
                        },
                        'to': to
                    }   
                },
                "step":arguments,
                "index":index
        }
        # dataJsonToSendCurrentStep = {
        #         "index": index
        #     }
        self.socket.emit('currentViewToSend',dataJsonToSendCurrentView,broadcast=True)
        # emit('currentStep',dataJsonToSendCurrentStep)
        # socketio.sleep(5)
        
        
    # @staticmethod
    # def received_data(local_manager, data):
    #     if hasattr(PresentPerson, 'topic_name'): # Means that a .top is loaded, need to check if it's done
    #         if data['data'] == "1":
    #             return {'id': PresentPerson.action_id}
    #     elif hasattr(PresentPerson, 'action_id'):
    #         return {'id': PresentPerson.action_id}
    #     return False

    # @staticmethod
    # def stop(local_manager):
    #     if hasattr(PresentPerson, 'topic_name') and PresentPerson.topic_name:
    #         local_manager.dialog.deactivateTopic(PresentPerson.topic_name)
    #         local_manager.dialog.unloadTopic(PresentPerson.topic_name)
    #         if hasattr(PresentPerson, 'reactivateMovement'):
    #             local_manager.autonomous_life.setAutonomousAbilityEnabled("SpeakingMovement", True)
    #             delattr(PresentPerson, 'reactivateMovement')
    #         delattr(PresentPerson, "topic_name")