#!/usr/bin/env python

from flask import Flask
from flask_cors import CORS, cross_origin
from flask_socketio import SocketIO, send, emit
from templates import app
from flask import jsonify
import json
from python_depend.configurations import DevelopmentConfig

app.config['SECRET_KEY'] = 'secret!'
app.config['CORS_HEADERS'] = "Content-Type"
app.config['CORS_RESOURCES'] = {r"/*": {"origins": "http://localhost:3000"}}

socketio = SocketIO(app, cors_allowed_origins="*")

#### SOCKET BRIDGE INIT #### STEP 1 on signale simplement que le socketBridge est initialise
@socketio.on('socketBridge')
@cross_origin()
def handle_my_custom_event(json):
    print('SOCKET BRIDGE INIT')
    return("")


@socketio.on('sendPeopleListDebug')
@cross_origin()
def handle_my_custom_event(json):
    socketio.emit('debugGetPeopleList',json)
    return("")

@socketio.on('hideMic')
@cross_origin()
def handle_my_custom_event(json):
    socketio.emit('appVisualChange',json)
    return("")


#### FROM REACT to HRIM ####  STEP 2 Le REACT envoie au HRIM le scenario selectionne afin qu'il load le json necessaire au lancement du scenario
@socketio.on('askToChangeScenarioGM')
@cross_origin()
def handle_my_custom_event(json):
    socketio.emit('askToChangeScenarioHRIM',json,broadcast=True)
    return("")

#### FROM HRIM to REACT ####  STEP 3 Le HRIM renvoie au REACT le json correspondant au scenario afin qu'il affiche les steps et le nom du scenario
@socketio.on('scenarioToCharged')
@cross_origin()
def handle_my_custom_event(json):
    socketio.emit('currentScenario',json)
    return("")
    

#### FROM REACT to HRIM #### STEP 4 Le REACT nous renvoie la liste des steps ordonnees ainsi que le nom du scenario
@socketio.on('currentViewHRIM')
@cross_origin()
def handle_my_custom_event(json):
    socketio.emit('scenarioCharged', json,broadcast=True)
    return("")

@socketio.on('viewLaunched')
@cross_origin()
def handle_my_custom_event(json):
    socketio.emit('launchedView', json,broadcast=True)
    return("")
    
#### FROM HRIM to REACT #### STEP 5 et 6 On a une etape qui commence on envoie au REACT le currentStep
@socketio.on('stepCurrent')
@cross_origin()
def handle_my_custom_event(json):
    socketio.emit('currentStep',json,broadcast=True)
    return("")

#### FROM HRIM to REACT #### STEP 5 et 6 On start le timer sur l etape courrante
@socketio.on('startTimer')
@cross_origin()
def handle_my_custom_event(json):
    socketio.emit('timerState',json,broadcast=True)
    return("")

#### FROM VIEW to REACT #### STEP 7 On recoit la VUE a envoyer au REACT afin 
#### qu il le charge et les attributs dont la vue a besoin
@socketio.on('currentViewToSend')
@cross_origin()
def handle_my_custom_event(json):
    socketio.emit('currentView',json,broadcast=True)
    return("")

#### FROM REACT to HRIM #### STEP 8 On recoit depuis le REACT les donnes que l utilisateur a entre
#### ou un status 200 et on envoie au HRIM

@socketio.on('indexOfDataReceived')
@cross_origin()
def handle_my_custom_event(json):
    socketio.emit('indexDataReceivedJS',json,broadcast=True)
    return("")

@socketio.on('dataReceived')
@cross_origin()
def handle_my_custom_event(json):
    socketio.emit('dataReceivedJS',json,broadcast=True)
    return("")

@socketio.on('restartHRI')
@cross_origin()
def handle_my_custom_event():
    socketio.emit('endScenario',broadcast=True)
    return("")

@socketio.on('askToResetHRIGM')
@cross_origin()
def handle_my_custom_event():
    socketio.emit('resetHRI',broadcast=True)
    return("")

#### FROM HRIM to REACT #### STEP 9 On recoit depuis le HRIM l index de l etape finie et on l envoie au REACT
### On reboucle ensuite sur STEP 5 et 6
@socketio.on('CompleteStep')
@cross_origin()
def handle_my_custom_event(json):
    socketio.emit('stepCompleted',json,broadcast=True)
    return("")

# from views import Views

if __name__ == '__main__':
    # app.config.from_object('configurations.DevelopmentConfig')
    app.config.from_object(DevelopmentConfig)
    socketio.run(app, host='0.0.0.0')
    #socketio.run(app)

