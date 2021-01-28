# Installation procedure for Palbator_devs module

## Dependencies for MULTIPLEXER (FLASK server)
```bash
sudo apt-get install python3-dev python3-pip python-eventlet
sudo pip install flask flask-socketio
sudo pip install -U flask-cors
```

## Dependencies for HRI MANAGER (FLASK client)
```bash
sudo pip install -U socketIO-client
```

## Dependencies for ttsMimic
```bash
sudo apt-get install gcc make pkg-config automake libtool libicu-dev libpcre2-dev libasound2-dev git
```
In a folder away from your workspace :
```bash
git clone https://github.com/MycroftAI/mimic.git
cd mimic
./autogen.sh
./configure --prefix="/usr/local"
make -j`nproc`
make check
sudo make install
```

## Dependencies for speechToTextPalbator
```bash
sudo apt-get install -y python python-dev python-pip build-essential swig libpulse-dev git
sudo pip install pyttsx3
sudo apt-get install libasound-dev
sudo apt-get install portaudio19-dev python-all-dev python3-all-dev && sudo pip install pyaudio
sudo apt-get install swig
sudo pip install pocketsphinx
```
If there are issues with pyaudio (version problems), those command lines could solve them:
```bash
sudo pip uninstall pyaudio or sudo apt-get purge --auto-remove python-pyaudio
sudo apt-get install portaudio19-dev python-all-dev python3-all-dev && sudo pip install pyaudio
```

## Dependencies for JSONDecodeError lib not included in JSON

sudo pip install simplejson
