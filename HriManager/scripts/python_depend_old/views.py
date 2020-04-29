import json
from flask_cors import CORS, cross_origin
from flask_socketio import SocketIO, send, emit
from __main__ import socketIO
from ask_age import AskAge
from ask_something import AskSomething
from ask_drink import AskDrink
from ask_name import AskName
from ask_speciality import AskSpeciality
from ask_to_follow import AskToFollow
from call_human import CallHuman
from confirm import Confirm
from generic import Generic
from go_to import GoTo
from present_person import PresentPerson
from seat_guest import SeatGuest
from show_video import ShowVideo
from ask_open_door import AskOpenDoor
from display_info import DisplayInfo
from wait import Wait


class Views:
    last_action = None
    current_scenario = None
    action_class_map = {
        'askAge': AskAge,
        'askDrink': AskDrink, # Generic with askSomething
        'askSomething': AskSomething,
        'askName': AskName, # Generic with askSomething
        'askSpeciality': AskSpeciality, # Generic with askSomething
        'displayInfo': DisplayInfo,
        'askOpenDoor': AskOpenDoor, # textToPrint
        'askToFollow': AskToFollow, # 
        'callHuman': CallHuman,
        'confirm': Confirm, # textToPrint
        'detailDrinks': None,
        'findAvailableDrinks': None,
        'findWhoWantsDrinks': None,
        'generic': Generic,
        'goTo': GoTo, # textToPrint
        'openDoor': None,
        'presentPerson': PresentPerson, # textToPrint, people: {who: {name, drinkId}, to: {name, drinkId}}
        'seatGuest': SeatGuest, # 
        'serveDrinks': None,
        'showVideo': ShowVideo,
        'wait': Wait, # time,
        'pointTo': Wait,
        'find': Wait
    }


    @staticmethod
    def start(HRIManager,step_name, arguments,  index, dataToUse):
        if step_name in Views.action_class_map and Views.action_class_map[step_name]:
            try:
                Views.last_action = step_name
                js_view_key = step_name
                if( step_name == 'askDrink' or step_name == 'askSpeciality' or step_name == 'askName'):
                    step_name = 'askSomething'
                if( step_name == 'displayInfo' or step_name == 'askOpenDoor'):
                    step_name = 'displayInfo'
                Views.action_class_map[step_name].start(js_view_key, arguments,index,dataToUse)
            except RuntimeError as ex:
                Views.last_action = None
        else:
          print('no action found')
          print(step_name)
          HRIManager.restart_hri({""})

