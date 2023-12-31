#!/usr/bin/env python3

import rospy
from std_msgs.msg import String
from mavros_msgs.msg import *
from nav_msgs.msg import *
from harpia_msgs.msg import *
from geometry_msgs.msg import *
from sensor_msgs.msg import NavSatFix, Imu, BatteryState


# --- 
from libs.anomalyDetection import checkAnomaly, pca_model, scaler_model
from libs import anomalyDetection,clustering,loadData, NoiseGenerator

from libs import uavAction as action
#S,NoiseGenerator #user libraries
# import numpy as np
# import pandas as pd
# import colorama;colorama.init(autoreset=True)
# import termcolor
# from colorama import Fore, Back, Style
# import progressbar
import time
import math
import numpy as np


import sys
import select  

def sequential_noise(data):
    sequential = {'roll':[],
                          'pitch':[],
                          'yaw':[],
                          'heading':[], 
                          'rollRate':[],
                          'pitchRate':[],
                          'yawRate':[],
                          'groundSpeed':[],
                          'climbRate':0, # ?
                          'altitudeRelative':[],
                          'throttlePct':[]}
    for key in sequential:
        sequential[key] = NoiseGenerator.noisyData(data,key, 1., 5.)

    return sequential


# Classes

class UAV(object):
    def __init__(self):
        self.sub_pose   = rospy.Subscriber('/drone_info/pose'  , DronePose, self.pose_callback)
        self.sequential = {'roll':-math.inf,
                          'pitch':-math.inf,
                          'yaw' : -math.inf,
                          'heading':-math.inf, 
                          'rollRate':-math.inf,
                          'pitchRate':-math.inf,
                          'yawRate':-math.inf,
                          'groundSpeed':-math.inf,
                          'climbRate':-math.inf, 
                          'altitudeRelative':-math.inf,
                          'throttlePct':-math.inf}
        # self.weightedCluster = []

    
    def pose_callback(self, data): 
        self.sequential['roll'] = data.roll
        self.sequential['pitch'] = data.pitch
        self.sequential['yaw'] = data.yaw
        self.sequential['heading'] = data.heading
        self.sequential['rollRate'] = data.rollRate
        self.sequential['pitchRate'] = data.pitchRate
        self.sequential['yawRate'] = data.yawRate
        self.sequential['groundSpeed'] = data.groundSpeed
        self.sequential['throttlePct'] = data.throttle
        self.sequential['altitudeRelative'] = data.altRelative
        self.sequential['climbRate'] = data.climbRate


    def get_uav(hardware):
        # uav.frame.weight = hardware['fault_time']['user_response']
        # uav.frame.max_velocity = hardware['fault_time']['classifier_time']
        # uav.frame.efficient_velocity = hardware['fault_time']['action_time']

        print('DO IT')

def land():
    mavros_cmd(
        '/mavros/cmd/land',
        CommandTOL,
        error_msg="Landing failed",
        altitude=10, latitude=0, longitude=0, min_pitch=0, yaw=0
    )



def listener():

    rospy.init_node('listener', anonymous=True)
    kenny = UAV()


    startMain = time.time()

    uav_stat = {
        'normal':0,
        'noise': 0,
        'mild': 0,
        'abnormal': 0, 
        'data': [],
        'last_state': 'none'
    }

    uav_action = []

    pct_normal = 0
    pct_noise = 0
    pct_mild = 0
    pct_abnormal = 0

    #deixar a janela de tempo no uav
    time_win = 10 
    # noise_limit = 0.85 # percent
    # mild_limit = 0.5 # percent
    # abnormal_limit = 0.1 # percent

    start = time.time()
    module1 = anomalyDetection.pca_model()
    end = time.time()
    print("\n>> Loaded PCA Module in {} seconds.".format(round((end-start),3)))


    start = time.time()
    module2 = anomalyDetection.scaler_model()
    end = time.time()
    print("\n>> Loaded Scaler Module in {} seconds.".format(round((end-start),3)))

    start = time.time()
    module3 = anomalyDetection.tree_model()
    end = time.time()
    print("\n>> Loaded Tree Module in {} seconds.".format(round((end-start),3)))

    # while not rospy.is_shutdown():
    startMain = time.time()
    win = 0 
    last_win = 0
    uav_stat, flag = anomalyDetection.checkAnomaly(kenny.sequential,uav_stat, module1, module2, module3, win)

    while flag:
        win = int(round((time.time()-startMain),3)//time_win)
        if last_win != win:
            # print(win)
            total = uav_stat['normal'] + uav_stat['noise'] + uav_stat['mild'] + uav_stat['abnormal']
            pct_normal = uav_stat['normal'] / total
            pct_noise = uav_stat['noise'] / total
            pct_mild = uav_stat['mild'] / total
            pct_abnormal = uav_stat['abnormal'] / total

            # ok -> nao tem abnormal e ruido (noise+mild)< 40%
            # base -> abnormal < 5% e mild < 40%
            # land -> abnormal > 5% ou mild > 50/70%  

            # [ok ok ok ok ok] -> segue
            # [ok base ok base ok] 

            # a cada 30 segundos 
            # ver se percentuais estao crescendo

            # uav_stat_decision.append()
            # print(pct_normal)
            # print(pct_noise)
            # print(pct_mild)
            # print(pct_abnormal)
            # print(total)
            

            #formalizar isso matematicamente
            if (pct_noise <= 0.7 and pct_abnormal == 0.0 and pct_mild <= 0.05):
                uav_action.append(0) # ok

            elif(pct_noise > 0.7 and pct_abnormal == 0.0 and pct_mild <= 0.1):
                uav_action.append(1) # soft warning 

            elif(0.1 < pct_mild <= 0.4): 
                uav_action.append(2) # warning

            elif(pct_abnormal< 0.05 and 0.4<pct_mild<=0.6):
                uav_action.append(4) # base

            elif(pct_abnormal< 0.1 or pct_mild>0.6):
                uav_action.append(8) # base

            elif(pct_abnormal<0.2):
                uav_action.append(16) # base

            else:
                uav_action.append(32) # land 

            #reset params
            last_win = win

            uav_stat['normal'] = 0
            uav_stat['noise'] = 0
            uav_stat['mild'] = 0
            uav_stat['abnormal'] = 0
            uav_stat['data'] = []
    
            # print(np.sum(uav_action[-5:]))
    
        if(time.time()-startMain > 1000) and (time.time()-startMain < 120):
            print('noisyData')

            
            error_data = sequential_noise(kenny.sequential)
            uav_stat, flag = anomalyDetection.checkAnomaly(error_data,uav_stat, module1, module2, module3, win)
        elif(time.time()-startMain > 300):
            break
        else:
            uav_stat, flag = anomalyDetection.checkAnomaly(kenny.sequential,uav_stat, module1, module2, module3, win)

        #deixar a janela de tempo no uav
        flag_action = np.sum(uav_action[-6:])
        
        # stop mission node
        
        if(flag_action>32):
            print("Do you want to continue? (y/[n])")
            start_input = time.time()

            while True:
                elapsed_time = time.time() - start_input

                # definir tempo de espera no arquivo uav
                if elapsed_time >= 30:
                    print("\nTimeout reached. Exiting.")
                    if(flag_action>48):
                        print('land')
                        action.land()
                        break
                    else:
                        print('base')
                        try:
                            action.go_to_base()
                        except:
                            action.land()
                        break

                if sys.stdin in select.select([sys.stdin], [], [], 0)[0]:
                    user_input = input().lower()
                    if user_input == 'y':
                        print("Continuing...")
                    elif user_input == 'n':
                        if(flag_action>25):
                            print('land')
                            action.land()
                            break
                        else:
                            print('base')
                            try:
                                action.go_to_base()
                            except:
                                action.land()
                            break
                time.sleep(0.1)
        
        rospy.Rate(1)
        

    # print(uav_stat['normal'])
    # print(uav_stat['noise'])
    # print(uav_stat['mild'])
    # print(uav_stat['abnormal'])
    # print(len(uav_stat['data']))
    print(pct_normal)
    print(pct_noise)
    print(pct_mild)
    print(pct_abnormal)
    print(uav_action)
    print(uav_action[-5:])


    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()
