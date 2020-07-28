#Installation procedure for Palbator_devs module

## Dependencies for MULTIPLEXER (FLASK server)
- Sudo apt-get install python3-dev python3-pip
- Sudo pip install flask
- Sudo pip install -U flask-cors
- Sudo pip install flask-socketio
- Sudo apt-get install python-eventlet

## Dependencies for HRI MANAGER (FLASK client)
- Sudo pip install -U socketIO-client

## Dependencies for ttsMimic
- sudo apt-get install gcc make pkg-config automake libtool libicu-dev libpcre2-dev libasound2-dev git
- git clone https://github.com/MycroftAI/mimic.git
- cd mimic
- ./autogen.sh
- ./configure --prefix="/usr/local"
- make -j4
- make check
- sudo make install

## Dependencies for speechToTextPalbator
- sudo apt-get install -y python python-dev python-pip build-essential swig libpulse-dev git
- sudo pip install pyttsx3
- sudo apt-get install libasound-dev
- sudo apt-get install portaudio19-dev python-all-dev python3-all-dev && sudo pip install pyaudio
- sudo apt-get install swig
- sudo pip install pocketsphinx

If there are issues with pyaudio, those command lines could solve them:
- sudo pip uninstall pyaudio or sudo apt-get purge --auto-remove python-pyaudio
- sudo apt-get install portaudio19-dev python-all-dev python3-all-dev && sudo pip install pyaudio

