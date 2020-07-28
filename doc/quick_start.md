# Quick start for Palbator_devs modules

## Test speechToTextPalbator
```bash
roslaunch speechToTextPalbator test_procedure.launch 
```
To run correctly the procedure, please follow the steps which will be displayed in your terminal.

## Test Mimic is well installed
```bash
mimic -t "Hello world"
```
If there is some trouble with Mimic functionning, please refer to [Mimic installation](https://github.com/Robocup-Lyontech/Palbator_devs/blob/master/doc/installation.md#dependencies-for-ttsmimic).

## Global launch : flask server + HRIManager + STT + TTS
```bash
roslaunch HriManager hri_manager.launch
```
