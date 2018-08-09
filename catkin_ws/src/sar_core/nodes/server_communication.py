import json
import requests
import time
import traceback
import os
import json

class Packet(object):
    LVL_NORMAL = 0
    LVL_IMPORTANT = 1
    LVL_URGENT = 2

    def __init__(self, m, l,r):
        self.packet = {
        	"message" : m,
        	"level" : l,
            "timestamp" : str(int(time.time())),
            "robot" : r,
            "data" : []
        }
    def add(self, d=None):
        if not(d==None):
            for dat in d:
                self.packet['data'].append(dat)
        return self
    def getData(self):
        return self.packet
    def send(self):
        Communication.sendData(self.packet)

class Communication:
    SERVER_BASE_URL = "http://caliper.cs.yale.edu"
    #SERVER_BASE_URL = "http://0.0.0.0"
    SERVER_POST_URL = SERVER_BASE_URL + "/api/packet"
    @staticmethod
    def sendData(packet):
        try:
            r = requests.post(Communication.SERVER_POST_URL, json = packet)
            if(r.status_code != 200):
                #server_comm.writeDataHold(out)
                print("could not send: ")
                print(packet)
            else:
                print("\033[92m Successfully sent a dump to the server \033[0m")
                #with open(DIR + 'data_hold.json', 'w+') as fo:
                    #pass #To empty contents of file when successfully sent
        except Exception as e:
            print(e)
            print(traceback.print_exc())
