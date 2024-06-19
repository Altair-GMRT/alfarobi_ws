#!/usr/bin/env python

import rospy
import os
import subprocess
from std_msgs.msg import String

node_keeper = ['/main_service']

def buttonCallback(msg):
    global buttonStatus
    buttonStatus = msg.data

    print(buttonStatus)


    if(buttonStatus == 'L2'):
        launchNode() 
    elif(buttonStatus == 'L3'):
        terminateNode()
        # buat matiin
    
        # buat jalanin vision monitor


# def killNode():
#     rospy.loginfo('KILL')
#     nodes = os.popen("rosnode list").readlines()

#     for i in range(len(nodes)):
#         nodes[i] = nodes[i].replace("\n", "")
#         print(nodes[i])

#     for i in range(len(nodes)):
#         if(nodes[i] != '/main_service' and nodes[i] != '/arduino_controller_node' and nodes[i] != '/rosout' and nodes[i] != '/server_node' and nodes[i] != '/client_node'):
#             os.system("rosnode kill " + nodes[i])


# def startNode():
#     # main_launch = 'roslaunch alfarobi passer_service.launch'    
#     main_launch = 'roslaunch vision_monitor vision_monitor.launch'    
#     rospy.loginfo('START')

#     os.system(main_launch)

proc = None
def launchNode():
    global proc
    proc = subprocess.Popen(["roslaunch", "alfarobi", "passerA.launch"])
def terminateNode():
    proc.terminate()

def main():
    rospy.init_node('main_service', anonymous=False)
    rospy.Subscriber('/arduino_controller/button', String, buttonCallback)

    rospy.spin()  


if __name__ == '__main__':
    main()
