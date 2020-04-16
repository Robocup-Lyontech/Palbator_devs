#!/usr/bin/env python

import rospy
from std_msgs.msg import String

class Test():

    def __init__(self):

        rospy.init_node("test")

        scenario = 'cleanup'
        action = 'askDrink'
        path_to_action_database = None

        print('----------------')
        if rospy.has_param("~hmm"):
            print("HMM "+str(rospy.get_param("~hmm")))

        try:
            if rospy.has_param("~parser_action_database"):
                data = rospy.get_param("~parser_action_database")

                database_name = data[scenario][action]
                path_to_action_database = rospy.get_param("~config")[scenario]['database'][database_name]

                print(database_name)
                print("*******************")
                print(path_to_action_database)
        except Exception as e:
            rospy.logerr("Scenario config couldn't be load correctly.")
            rospy.logerr(e.__class__.__name__ + ": " + e.message)

        # if rospy.has_param("~"+req.data):
        #     a=rospy.get_param("~"+req.data)
        #     print("Scenario "+str(a))

if __name__ == "__main__":
    a=Test()
    
