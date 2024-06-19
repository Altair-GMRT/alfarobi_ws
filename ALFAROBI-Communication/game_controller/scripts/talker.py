#!/usr/bin/env python

import time
from socket import *
import rospy
import threading

from std_msgs.msg import String

pub = rospy.Publisher('game_controller/stateReferee', String, queue_size=10)
rospy.init_node('viewStateReferee',anonymous=True)
rate = rospy.Rate(10)
WarnaTimAlfarobi= rospy.get_param("warna_tim") #0 = biru, 1=merah
def runConnection():
    addrs = "255.255.255.255"
    ports = 3838
    clientSocket = socket(AF_INET,SOCK_DGRAM)
    clientSocket.setsockopt(SOL_SOCKET,SO_REUSEADDR,1)
    clientSocket.settimeout(0.5)
    clientSocket.bind((addrs,ports))
    counter = 0
    t_start = time.time()
    
    #WarnaTimAlfarobi= 0 #0 = biru, 1=merah
    while not rospy.is_shutdown():
        stateNow=0
        last_data=0
        ct=0       
        try:
            data,server = clientSocket.recvfrom(157)
            t_end = time.time()
            elapsed = t_start - t_end
            if((data) and (len(data)==116)):
                datax = [ord(c) for c in data]
                if(ct == 0 and datax[9]==0):
                    ct=1
                elif(ct==1 and datax[9]==0):
                    datax[9]=last_data
                if(data[9]!=0):
                    last_data=datax[9]
                #print("STATE : ",last_data)
                if(last_data == 0):
                    stateNow = "INITIAL"
                    publish(stateNow)
                elif(last_data == 1):
                    stateNow = "READY"
                    publish(stateNow)
                elif(last_data == 2 ):
                    stateNow = "SET"
                    publish(stateNow)
                elif(last_data == 3  and datax[11]==WarnaTimAlfarobi):
                    stateNow = "PLAYING_NO_DELAY"
                    #print(datax[11])
                    publish(stateNow)
                elif(last_data == 3 and datax[11]!=WarnaTimAlfarobi):
                    stateNow = "PLAYING_WITH_DELAY"
                    #print(datax[11])
                    publish(stateNow)
                elif(last_data == 4):
                    stateNow = "FINISHED"
                    publish(stateNow)
                else:
                    stateNow = "TIMEOUT"
                    publish(stateNow)
                #time.sleep(1)
        except timeout:
            counter = counter+1
            stateNow = "TIMEOUT"
            if counter > 20:
                stateNow = "REFERE_BOX_MATI"
                counter = 0
            publish(stateNow)
            #print("TIME OUT")

def publish(state):
    rospy.loginfo(str(state))
    pub.publish(str(state))
    rate.sleep()

def isGoal(state):
    pass

def isDropBall(state):
    #otw code
    pass
    

if __name__ == '__main__':
    try:
        #print(WarnaTimAlfarobi)
        runConnection()
        #thread if needed, but cannot killed
       
        #t = threading.Thread(target=runConnection)
        #t.start()
        
    except rospy.ROSInterruptException:
        pass
        #t.do_run = False
