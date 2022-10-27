import sys
import time
import matplotlib.pyplot as plt
import numpy as np
import math

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

from px4_msgs.msg import Timesync
from msg_srv_act_interface.srv import MapInit


class MapService(Node):
    def __init__(self):
        super().__init__('map_service')
        self.qosProfileGen()
        self.declare_service_client_custom()
        
    def declare_service_client_custom(self):
        self.MapSequenceServiceClient_ = self.create_client(MapInit, 'map_sequence')
        while not self.MapSequenceServiceClient_.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Map Generation not available, waiting again...')    
    
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
        
    def RequestMapGeneration(self,flag):
        # print("---- Debug ----")
        '''
        bool request_init_map
        ---
        bool map_sequence_init

        '''
        self.map_sequence_request = MapInit.Request()
        self.map_sequence_request.request_init_map = flag
        self.future = self.MapSequenceServiceClient_.call_async(self.map_sequence_request)
        