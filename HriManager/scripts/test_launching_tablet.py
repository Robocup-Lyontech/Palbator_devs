#!/usr/bin/env python

import rospy
import subprocess
import os
import socket

class Test():

    def __init__(self):

        rospy.init_node("test_tablet_remote")

        path_to_js_app = "/home/student/Bureau/copie_git_HRI/robocup_palbator-hri_js/"

        self.process=subprocess.Popen(['npm','start','--prefix',str(path_to_js_app)],stdin=subprocess.PIPE, stdout=subprocess.PIPE,close_fds=True)
        self.process.stdin.close()

        self.tablet_ON = False

        # while self.tablet_ON == False:
        #     self.tablet_ON = self.isOpen("127.0.0.1",3000)

        # rospy.logwarn('TABLEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEET')


    def isOpen(self,ip,port):
        s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        try:
            s.connect((ip, int(port)))
            s.shutdown(2)
            return True
        except:
            return False



if __name__ == "__main__":
    a = Test()
    while not rospy.is_shutdown():
        rospy.spin()
    rospy.logwarn("killing tablet process")
    a.process.terminate()
