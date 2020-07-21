import json
from flask_cors import CORS, cross_origin
from flask_socketio import SocketIO, send, emit
# from __main__ import socketIO
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
from point_to import PointTo
from find import Find
from ask_room_to_clean import AskRoomToClean
from open_door import OpenDoor
from find_object import FindObject
from catch_object import CatchObject
from store_object import StoreObject
from release_object import ReleaseObject
from object_action import ObjectAction
from look_for_guest import LookForGuest
from found_guest import FoundGuest
from found_anyone import FoundAnyone
from look_for_known_guest import LookForKnownGuest
from main_menu_palbator import MainMenuPalbator
from found_no_object import FoundNoObject
from confirm_object_action import ConfirmObjectAction

class Views:
    def __init__(self,socket):
        self.socketIO=socket
        self.last_action = None
        self.current_scenario = None
        self.action_class_map = {
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
            'pointTo': PointTo,
            'find': Find,
            'askRoom': AskRoomToClean,
            'openDoor': OpenDoor,
            'findObject': FindObject,
            'catchObject': CatchObject,
            'storeObject': StoreObject,
            'releaseObject': ReleaseObject,
            'objectAction': ObjectAction,
            'lookForGuest': LookForGuest,
            'foundGuest': FoundGuest,
            'foundAnyone': FoundAnyone,
            'lookForKnownGuest': LookForKnownGuest,
            'mainMenuPalbator': MainMenuPalbator,
            'foundNoObject': FoundNoObject,
            'confirmObjectAction': ConfirmObjectAction
        }
        self.view=None


    
    def start(self,step_name, arguments,  index, dataToUse):
        if step_name in self.action_class_map and self.action_class_map[step_name]:
            try:
                self.last_action = step_name
                js_view_key = step_name
                if( step_name == 'askDrink' or step_name == 'askSpeciality' or step_name == 'askName'):
                    step_name = 'askSomething'
                if( step_name == 'displayInfo' or step_name == 'askOpenDoor'):
                    step_name = 'displayInfo'
                self.view=self.action_class_map[step_name](self.socketIO)
                self.view.start(js_view_key, arguments,index,dataToUse)
            except RuntimeError as ex:
                self.last_action = None
        else:
          print('no action found')
          print(step_name)
        #   HRIManager.restart_hri({""})

