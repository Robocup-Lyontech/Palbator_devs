#!/usr/bin/env python

from flask import Flask
from flask_cors import CORS, cross_origin
from flask_socketio import SocketIO, send, emit
from templates import app
from flask import jsonify
import json


app.config['SECRET_KEY'] = 'secret!'
app.config['CORS_HEADERS'] = "Content-Type"
app.config['CORS_RESOURCES'] = {r"/*": {"origins": "http://localhost:3000"}}

socketio = SocketIO(app, cors_allowed_origins="*")

#### SOCKET BRIDGE INIT #### STEP 1 on signale simplement que le socketBridge est initialise
@socketio.on('socketBridge')
@cross_origin()
def handle_my_custom_event():
    print('received from react client socketBridge init' )

#### FROM REACT to HRIM ####  STEP 2 Le REACT envoie au HRIM le scenario selectionne afin qu'il load le json necessaire au lancement du scenario
@socketio.on('askToChangeScenarioGM')
@cross_origin()
def handle_my_custom_event(json):
    socketio.emit('askToChangeScenarioHRIM',json,broadcast=True)

#### FROM HRIM to REACT ####  STEP 3 Le HRIM renvoie au REACT le json correspondant au scenario afin qu'il affiche les steps et le nom du scenario
@socketio.on('scenarioToCharged')
@cross_origin()
def handle_my_custom_event(json):
    print("SCENARIO TO CHARGE")
    socketio.emit('currentScenario',json)
    

#### FROM REACT to HRIM #### STEP 4 Le REACT nous renvoie la liste des steps ordonnees ainsi que le nom du scenario
@socketio.on('currentViewHRIM')
@cross_origin()
def handle_my_custom_event(json):
    emit('scenarioCharged', json,broadcast=True)
    
#### FROM HRIM to REACT #### STEP 5 et 6 On a une etape qui commence on envoie au REACT le currentStep
@socketio.on('stepCurrent')
@cross_origin()
def handle_my_custom_event(json):
    emit('currentStep',json,broadcast=True)

#### FROM HRIM to REACT #### STEP 5 et 6 On start le timer sur l etape courrante
@socketio.on('startTimer')
@cross_origin()
def handle_my_custom_event(json):
    emit('timerState',json,broadcast=True)

#### FROM VIEW to REACT #### STEP 7 On recoit la VUE a envoyer au REACT afin 
#### qu il le charge et les attributs dont la vue a besoin
@socketio.on('currentViewToSend')
@cross_origin()
def handle_my_custom_event(json):
    emit('currentView',json,broadcast=True)

#### FROM REACT to HRIM #### STEP 8 On recoit depuis le REACT les donnes que l utilisateur a entre
#### ou un status 200 et on envoie au HRIM
@socketio.on('dataReceived')
@cross_origin()
def handle_my_custom_event(json):
    emit('dataReceivedJS',json,broadcast=True)

@socketio.on('restartHRI')
@cross_origin()
def handle_my_custom_event():
    emit('endScenario',broadcast=True)

@socketio.on('askToResetHRIGM')
@cross_origin()
def handle_my_custom_event():
    emit('resetHRI',broadcast=True)

#### FROM HRIM to REACT #### STEP 9 On recoit depuis le HRIM l index de l etape finie et on l envoie au REACT
### On reboucle ensuite sur STEP 5 et 6
@socketio.on('CompleteStep')
@cross_origin()
def handle_my_custom_event(json):
    emit('stepCompleted',json,broadcast=True)

# from views import Views

if __name__ == '__main__':
 app.config.from_object('configurations.DevelopmentConfig')
 socketio.run(app)

