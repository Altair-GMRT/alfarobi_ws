#!/usr/bin/env python
#-*- coding:utf-8 -*-

import socket
import time
import logging
from StateCom import BaseStationData, RobotName, RobotStatus, BallStatus
import random
from enum import Enum
import geometry_msgs, std_msgs
from geometry_msgs.msg import Pose2D, Point
from std_msgs.msg import Int8

import rospy, rospkg, rosparam
from team_communication.msg import robot_data, team_data
import yaml

rospack = rospkg.RosPack()

DEFAULT_ADDR = '255.255.255.255'
BASE_ADDR='192.168.0.104'
DEFAULT_PORT_SEND = 20902
DEFAULT_PORT_RECV = 20901

logger = logging.getLogger('team_comm')
logger.setLevel(logging.DEBUG)

console_handler = logging.StreamHandler()
console_handler.setFormatter(logging.Formatter("%(asctime)s %(message)s"))
logger.addHandler(console_handler)

class RobotStateSenderNReceiver(object):
    
    #contructor
    def __init__(self, addr=DEFAULT_ADDR, port_recv=DEFAULT_PORT_RECV,port_send=DEFAULT_PORT_SEND):
        self.robot_id = None
        self.robot_name = None
        self.addr = (addr,port_send)
        self.port_recv = port_recv
        self.port_send = port_send
        self.data = None
        self.socket = None
        self.state = None
        self.running = True
        self.friend = {}
        self.seq = 0
        self.init_socket()
        self.team_data = team_data
        self.base_monitor = False

    def init_socket(self):
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
        self.socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.socket.bind(self.addr)
        self.socket.settimeout(0.5)
        
        self.socket2 = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
        self.socket2.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        #self.socket2.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
        self.socket2.settimeout(0.2)
        self.socket2.bind(("",5444))
    def build_data(self):
        #self.data = BaseStationData.build(dict(robot_id=self.robot_id,robot_pos=dict(xpos=12,ypos=12,wpos=12.2),ball_pos=dict(xpos=12,ypos=12,wpos=123.2)))
        data_write = dict(robot_id=self.robot_id,
                            robot_data=dict(xpos=random.randrange(0, 101, 2),ypos=2,wpos=3,
                                robot_status=random.randint(0,5), ball_status=random.randint(0,1), is_moving=False),
                            ball_pos=dict(xpos=2,ypos=2,wpos=3))
        self.data = BaseStationData.build(data_write)
        return self.data

    def receiver(self):
        try:
            data, peer = self.socket.recvfrom(BaseStationData.sizeof())
            #print(len(data))
            parsed_state = BaseStationData.parse(data)
            self.state = parsed_state
            self.time = time.time()

        except AssertionError as ae:
            rospy.logerr(ae.message)
        except socket.timeout:
            rospy.logwarn("Socket timeout")
        except Exception as e:
            logger.exception(e)

    def requestSend(self, data):
        data_write = dict(robot_id=self.robot_id,
                            robot_data=dict(xpos=data.robot_pos.x,ypos=data.robot_pos.y,wpos=data.robot_pos.theta,
                                            robot_status=1, robot_strategy=data.robot_strategy.data, is_moving=data.is_moving.data),
                            ball_pos=dict(xpos=data.ball_local.x,ypos=data.ball_local.y,wpos=data.ball_local.z))
        # self.data = BaseStationData.build(data_write)
        # return self.data
           

    def send_data(self):
        self.data = self.build_data()
        try:
            destination = BASE_ADDR, self.port_send
            
            self.socket2.sendto(self.data, destination)
	    time.sleep(0.5)
        except Exception as e:
            rospy.logwarn("Network Error: %s" % str(e))

    def parser(self):
        try:
            if self.state==None:
                print("state not ready yet, check network!")
            elif self.state.header=="UGMx" and RobotName.get(self.state.robot_id)!=self.robot_id:         
                print("Friend List")

                self.reporting(self.state)
                #print("Data :"+ str(self.state.robot_id))
                #print(self.state)
            else:
                pass
                #state sendiri bisa disini
        except Exception as e:
            logger.exception(e)

    def reporting(self,data):
        print(type(self.state.robot_id))
        print(RobotName.get(self.state.robot_id))
        print(type(RobotName.get(self.state.robot_id)))
        self.friend[data.robot_id] = {'name': data.robot_id,'robot_data':data.robot_data,'ball_position':data.ball_pos}

    def print_report(self):
        if(self.friend):
            print('====REPORT=====')
            for i in self.friend:
                print('Nama Teman :'+self.friend[i]['name'])
                print('Posisi Robot')
                print(self.friend[i]['robot_data'])
                print('Posisi Bola')
                print(self.friend[i]['ball_position'])
                print('====EOF=====')
            # time.sleep(0.5)
        else:
            print("Teman tidak tersedia :(")
        
    def run(self):
        rospy.init_node("team_communication_node")
        rate = rospy.Rate(60)
        rospy.Subscriber("team_communication/request_send", robot_data, self.requestSend)
        team_data_pub = rospy.Publisher('team_communication/team_data', team_data, queue_size=10)

        path = rospack.get_path('op3_manager') + '/config/GlobalConfig.yaml'
        rospy.loginfo('loaded from file {}'.format(path))
        config_ = yaml.load(file(path, 'r'))
        general = config_['general']
        self.robot_id = general['robot_id']
        self.robot_name= general['robot_name']
        

        if rospy.has_param('base_monitor'):
            self.robot_id = 4
            self.base_monitor = True

        while not rospy.is_shutdown():
        	
            try:
                self.seq = self.seq+1
                self.receiver()
                if(self.robot_id!=4):
                    self.send_data()
                self.parser()
                self.print_report()
                rate.sleep()
            except IOError as e:
                rospy.logdebug("Main job error!" + str(e))

if __name__ == '__main__':
    try:
        RobotConn = RobotStateSenderNReceiver()
        RobotConn.run()

    except rospy.ROSInterruptException:
        pass
