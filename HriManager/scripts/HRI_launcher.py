#!/usr/bin/env python
import rospy
 
from HriManager.msg import DataToSay
from std_msgs.msg import String
import time
from socketIO_client import SocketIO, LoggingNamespace
import json as js
import weakref
from HRIManager import HRIManager

class HRILauncher(object):

  def __init__(self):
    rospy.init_node("hri_manager_node",anonymous=True)

    self.pub_restart=rospy.Publisher("HRI_restart_request", String,queue_size=1)
    
    self.hri=HRIManager(socketIO)

    #callback pour revenir au menu principal sur la tablette
    socketIO.on('resetHRI', self.restart_hri)


  def restart_hri(self,json):
    rospy.logwarn("RESTART REQUEST ------------------------------")
    # message_to_VoiceManager=DataToSay()
    # dataJsonView={
    #   "order": 0,
    #   "action": None
    # }
    # message_to_VoiceManager.json_in_string=js.dumps(dataJsonView)
    # message_to_VoiceManager.data_to_use_in_string=''
    # self.hri.pub_current_view.publish(message_to_VoiceManager)


    

    self.hri.action_GM_TO_HRI_server.set_aborted()
    # time.sleep(2)
    # # del self.hri
    # # self.hri=None
    # time.sleep(2)
    self.hri=HRIManager(socketIO)
    self.pub_restart.publish("RESTART")
    time.sleep(1)
    socketIO.emit('restartHRI',broadcast=True)
    rospy.loginfo("RESTART HRI")
  
    

if __name__ == '__main__':
  
  
  socketIO = SocketIO('http://127.0.0.1', 5000, LoggingNamespace)
  
  hri_launcher=HRILauncher()


  while not rospy.is_shutdown():
    # socketIO.wait(seconds=0.1)
    rospy.Rate(1).sleep()