#!/usr/bin/env python
import socket
import time
import rospy

from std_msgs.msg import String

class ConnectionManager(object):

    def __init__(self):
        rospy.init_node('Connection_manager_node')

        _publisher_name_param = "~connection_state_publisher_name"
        if rospy.has_param(_publisher_name_param):
            self.pub=rospy.Publisher(rospy.get_param(_publisher_name_param),String,queue_size=1)
        else:
            rospy.logerr("{class_name} : No publisher name specified ".format(class_name=self.__class__.__name__))

        self.rate=rospy.Rate(1)

        _hostname_server_param = "~hostname_server"
        if rospy.has_param(_hostname_server_param):
            self.hostname_server = rospy.get_param(_hostname_server_param)
        else:
            rospy.logerr("{class_name} : No hostname server specified ".format(class_name=self.__class__.__name__))

        _port_param = "~port"
        if rospy.has_param(_port_param):
            self.socket_port = rospy.get_param(_port_param)
        else:
            rospy.logerr("{class_name} : No port specified ".format(class_name=self.__class__.__name__))

        _timeout_param = "~connection_timeout"
        if rospy.has_param(_timeout_param):
            self.connection_timeout = rospy.get_param(_timeout_param)
        else:
            self.connection_timeout = 2

    def is_connected(self):
        try:
            # see if we can resolve the host name -- tells us if there is
            # a DNS listening
            host = socket.gethostbyname(self.hostname_server)
            # connect to the host -- tells us if the host is actually
            # reachable

            s = socket.create_connection((host, self.socket_port), self.connection_timeout)
            s.close()
            return True
        except:
            pass
        return False
                
            

if __name__ == "__main__":
    instance_connection=ConnectionManager()
    while not rospy.is_shutdown():
        a=instance_connection.is_connected()
        if a==True:
            instance_connection.pub.publish("Connected")
        else:
            instance_connection.pub.publish("Disconnected")
        instance_connection.rate.sleep()

    
