import sys
import time
import matplotlib.pyplot as plt
import numpy as np
import math
from itertools import chain

import time
import onnx
import onnxruntime as ort
#   ROS2 python 
import rclpy
from rclpy.node import Node
from rclpy.qos_event import SubscriptionEventCallbacks
from rclpy.parameter import Parameter
from rclpy.qos import QoSDurabilityPolicy
from rclpy.qos import QoSHistoryPolicy
from rclpy.qos import QoSProfile
from rclpy.qos import QoSReliabilityPolicy
from rclpy.qos import qos_profile_sensor_data


from .PathFollowing.PF_GPR import PF_GPR

from px4_msgs.msg import Timesync
from msg_srv_act_interface.srv import PathFollowingGPR


class PF_GPR_Module(Node):
    def __init__(self):
        super().__init__('pf_gpr_module')
        ##  Input
        self.requestFlag = False    #   bool
        self.requestTimestamp = 0   #   uint
        self.outNDO = []    #   double
        ##  Output
        self.response_timestamp = 0 #   uint
        self.gpr_output_data = []
        self.gpr_output = []
        self.gpr_output_index = 0
        ##  Function
        self.qosProfileGen()
        self.declare_subscriber_px4()
        self.PFGPRService_ = self.create_service(PathFollowingGPR, 'path_following_gpr', self.PFGPRCallback)
        print("===== Path Following Attitude Command Node is Initialize =====")
        
#################################################################################################################

    def PFGPRCallback(self, request, response):
        print("===== Request Path Following Guidance Node =====")
        '''
        uint64 request_timestamp	# time since system start (microseconds)
        bool request_gpr
        float64[] out_ndo
        '''
        self.requestFlag = request.request_gpr
        self.requestTimestamp = request.request_timestamp
        self.outNDO = request.out_ndo
        
        if self.requestFlag is True : 
            ############# Algirithm Start - PF_GPR  #############
            if self.InitFlag == 0:
                self.InitTime  =   self.requestTimestamp * 10**(-6)
                self.InitFlag   =   1
            Time   =   self.requestTimestamp * 10**(-6) - self.InitTime
            # function
            GPR_out = self.PF_GPR_MOD.PF_GPR_Module(Time, self.outNDO)
            # output
            self.GPR_out = GPR_out.copy()
            ############# Algirithm  End  - PF_GPR  #############
            print("===== Path Following Guidance Generation !! =====")
            '''
            uint64 response_timestamp	# time since system start (microseconds)
            bool response_gpr
            float64[] gpr_output_data
            uint32 gpr_output_index
            '''
            self.gpr_output_data = list(chain.from_iterable(self.GPR_out))
            self.gpr_output_index = 3
            response.response_timestamp = self.response_timestamp
            response.response_gpr = True
            response.gpr_output_data = self.gpr_output_data
            response.gpr_output_index = self.gpr_output_index
            print("===== Response Path Following Guidance Node =====")
            return response
        else : 
            response.response_timestamp = self.response_timestamp
            response.response_gpr = False
            response.gpr_output = self.gpr_output
            print("===== Can't Response Path Following Guidance Node =====")
            return response

    def TimesyncCallback(self, msg):
        self.response_timestamp = msg.timestamp
        
    def qosProfileGen(self):
        #   Reliability : 데이터 전송에 있어 속도를 우선시 하는지 신뢰성을 우선시 하는지를 결정하는 QoS 옵션
        #   History : 데이터를 몇 개나 보관할지를 결정하는 QoS 옵션
        #   Durability : 데이터를 수신하는 서브스크라이버가 생성되기 전의 데이터를 사용할지 폐기할지에 대한 QoS 옵션
        self.QOS_Sub_Sensor = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=5,
            durability=QoSDurabilityPolicy.VOLATILE)
        
        self.QOS_Service = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
            durability=QoSDurabilityPolicy.VOLATILE)
        
    def declare_subscriber_px4(self):
        #   init PX4 MSG Subscriber
        self.TimesyncSubscriber_ = self.create_subscription(Timesync, '/fmu/time_sync/out', self.TimesyncCallback, self.QOS_Sub_Sensor)
        # self.EstimatorStatesSubscriber_ = self.create_subscription(EstimatorStates, '/fmu/estimator_states/out', self.EstimatorStatesCallback, self.QOS_Sub_Sensor)
        # self.VehicleAngularVelocitySubscriber_ = self.create_subscription(VehicleAngularVelocity, '/fmu/vehicle_angular_velocity/out', self.VehicleAngularVelocityCallback, self.QOS_Sub_Sensor)
        print("====== px4 Subscriber Open ======")