# PALBATOR-HRI
HRI python for Palbator, including Flask server and python client ( HRI Manager )

## Dependencies for FLASK server
- Sudo apt-get install python3-dev python3-pip
- Sudo pip install flask
- Sudo pip install -U flask-cors
- Sudo pip install flask-socketio

## Dependencies for FLASK client
- Sudo pip install -U socketIO-client

## Launch
- First, launch the flask server with : "python multiplexer.py"
- Then, launch the python client with : "python HRIManager.py"

## Uses
In order to completely launch the HRI,

- Start the palbator python `before` the react
- Start the palbator js by following the instructions in the specifi repo
