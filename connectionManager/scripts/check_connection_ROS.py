#!/usr/bin/env python
import urllib
import time
import rospy

from std_msgs.msg import String

class ConnectionManager(object):

    def __init__(self):
        rospy.init_node('Connection_manager_node')
        self.pub=rospy.Publisher('connection_state',String,queue_size=1)
        self.rate=rospy.Rate(1)
        self.connection_ON=None
        self.new_state=None

    def check_connection_state(self):
        try :
            stri = "https://www.google.co.in"
            data = urllib.urlopen(stri)
            print "Connected"
            # self.new_state=True
            # if self.new_state!=self.connection_ON:
            #     self.connection_ON=self.new_state
            self.pub.publish("Connected")
        except Exception as e:
            print("not connected : "+str(e))
            # self.new_state=False
            # if self.new_state!=self.connection_ON:
            #     self.connection_ON=self.new_state
            self.pub.publish("Disconnected")
                
            

if __name__ == "__main__":
    instance_connection=ConnectionManager()
    while not rospy.is_shutdown():
        instance_connection.check_connection_state()
        instance_connection.rate.sleep()

    
