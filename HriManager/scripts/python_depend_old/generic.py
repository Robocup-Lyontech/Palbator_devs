import json
from flask_socketio import SocketIO, send, emit
from templates import app
from flask_cors import CORS, cross_origin
from __main__ import socketIO

global stepCompletedJson


class Generic:
    @staticmethod
    def start(js_view_key, local_manager, arguments):
        Generic.action_id = arg_fetcher.get_argument(arguments, 'id')
        if not Generic.action_id:
            logger.log("Missing id in {0} action arguments".format(js_view_key), "Views Manager", logger.ERROR)
            local_manager.send_view_result(js_view_key, {'error': 400})
        
        args = arg_fetcher.get_argument(arguments, 'args')
        speech = arg_fetcher.get_argument(args, 'speech')

        if speech:
            text = arg_fetcher.get_argument(speech, 'title')
            said = arg_fetcher.get_argument(speech, 'said')
            content = arg_fetcher.get_argument(speech, 'content')
        list_content = arg_fetcher.get_argument(args, 'list')
        image = arg_fetcher.get_argument(args, 'image')
        video = arg_fetcher.get_argument(args, 'video')
        
        if said:
            if arg_fetcher.get_argument(speech, 'noSpeechAnimated'):
                local_manager.autonomous_life.setAutonomousAbilityEnabled("SpeakingMovement", False)
                Generic.reactivateMovement = True

            top_path = os.path.join(os.getcwd(), local_manager.lm_config['tops']['say_smth_and_return'])
            Generic.topic_name = local_manager.dialog.loadTopic(top_path)
 
        if text:
            local_manager.memory.raiseEvent(local_manager.lm_config['currentView']['ALMemory'], json.dumps({
                'view': js_view_key,
                'data': {
                    'title': text,
                    'list': list_content,
                    'content': content,
                    'image': image,
                    'video': video
                }
            }))


    # @staticmethod
    # def received_data(local_manager, data):
    #     if hasattr(Generic, 'topic_name'): # Means that a .top is loaded, need to check if it's done
    #         if data['data'] == "1":
    #             return {'id': Generic.action_id}
    #     elif hasattr(Generic, 'action_id'):
    #         return {'id': Generic.action_id}
    #     return False

    # @staticmethod
    # def stop(local_manager):
    #     if hasattr(Generic, 'topic_name') and Generic.topic_name:
    #         local_manager.dialog.deactivateTopic(Generic.topic_name)
    #         local_manager.dialog.unloadTopic(Generic.topic_name)
    #         if hasattr(Generic, 'reactivateMovement'):
    #             local_manager.autonomous_life.setAutonomousAbilityEnabled("SpeakingMovement", True)
    #             delattr(Generic, 'reactivateMovement')
    #         delattr(Generic, "topic_name")